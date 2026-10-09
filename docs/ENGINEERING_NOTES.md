# LuckySort — Engineering Notes

Demo talking points: the problems the in-editor runs exposed, how each was diagnosed, and how it was fixed and
pinned down with a test. Full spec: `LuckySort_Idea_Plan.md`. Engine facts: `LUCKY_API_NOTES.md`.

---

## 1. Architecture in one picture

```text
 TaskConfig + seed
        │
        ▼
 TaskGenerator ──► TaskDefinition ──► StackPlanner ──► PlannedStep[]          (what, in which order)
 (Generation/)      (Core/)            (Planning/)            │
                                                              ▼
                                                     TaskStateMachine          (how, with checks)
                                                     (StateMachine/)
                                                       │           ▲
                                           IManipulator│           │IWorldState
                                                       ▼           │
                                          PandaManipulator     SceneWorldState
                                          (motion graph IK,    (MuJoCo body poses
                                           actuator8 gripper)   via entity transforms)
                                                       │           ▲
                                                       ▼           │
                                                  ── Lucky Engine / MuJoCo ──
                                                              │
                                                              ▼
                                                       StackEvaluator           (did it physically work?)
                                                       (Evaluation/)
```

- Everything above the engine line except `PandaManipulator` / `SceneWorldState` is **engine-independent**
  (System.Numerics only) and compiled straight into the xunit project.
- `FakeSim` implements both `IManipulator` and `IWorldState`: a kinematic stand-in for the arm and the cubes, with
  fault injection (slips, pushes, drop offsets, yaw convention, tracking error). Whole episodes run offline in
  milliseconds, so every in-editor failure gets reproduced as a unit test before it is fixed.
- The **planner reads task metadata**, the **state machine closes the loop on measured state**, and the
  **evaluator reads only physical state**. None of them trusts the layer above it.

## 2. The pick cycle

```text
SelectTarget → MoveAboveObject → AlignGripper → CenterOverObject → MoveToObject → Grasp → Lift → VerifyGrasp
            → MoveToStack → AlignGripper → Place → Release → Retreat → VerifyPlacement → SelectTarget …
any check fails → Recovery (open, back off up, re-read the object, retry within the attempt budget) → Failure
```

Every state has an entry action, a completion condition, a timeout and a failure path. Time only advances through
`Update(dt)` (sim time), with no wall-clock waits.

The two calibration states, `AlignGripper` (yaw) and `CenterOverObject` (position), exist because of the
in-editor failures below.

---

## 3. Problems found in the editor, and what fixed them

### 3.1 Unknown gripper-yaw convention → closed-loop yaw (`AlignGripper`)

**Symptom.** Seed 12345: the open fingers came down rotated relative to a Large cube and pushed it away.
Seed 1234: the arm stalled 51 mm above a cube.

**Diagnosis.** The motion graph takes a target orientation in Euler degrees. How "add θ to the Y component" maps
to the physical rotation of the fingers about world Y (sign, and a fixed offset from the hand's −45° mount)
wasn't documented. A 60 mm cube only fits the ~80 mm jaw within roughly ±20° of square.

**Fix.** Measure instead of assuming:
- `GripperYaw` reads `grasp_site`'s world quaternion and takes whichever local axis is most horizontal. With
  the gripper pointing down, that gives the yaw **modulo 90°**, and modulo 90° is all a cube grasp needs.
- Model `physical = sign · command + offset (mod 90°)`. After each move, `AlignGripper` dwells briefly, measures,
  and re-solves the offset. If a correction made the error *worse*, the sign guess was wrong and is flipped.
- Commands stay within ±45° (`NearestCubeYaw`) so the wrist never winds up.

**Talking point.** This calibrates the actuator convention at runtime instead of hard-coding it, the same way
you'd treat an unknown sensor/actuator frame on a real robot.

**Tests.** `YawConventionIsLearnedFromMeasurement` (several sign/offset combinations × 5 seeds, zero collisions),
`WithoutClosedLoopYawAWrongSignCollidesAndIsDetected`.

### 3.2 Tracking error vs. finger clearance → closed-loop centring (`CenterOverObject`)

**Symptom.** With yaw fixed, seed 12345 stacked Blue and Green, then failed `LS_Obj_Yellow_Large` three times:

```text
Collision: LS_Obj_Yellow_Large pushed 8 mm during descent (gripper yaw -144.4°, object yaw 37.0°)
FAILED (GraspFailed): LS_Obj_Yellow_Large not picked after 3 attempts.
```

**Diagnosis.**
- Yaw was *not* the problem: −144.4° vs 37.0° is 1.4° apart modulo 90°.
- The clearance budget: Panda jaw opens to 80 mm, a Large cube is 60 mm, so there's **~10 mm per finger**.
- A move counted as done when the grasp site was within `MoveTolerance` = **15 mm**. The arm's steady-state
  tracking error (from the IK solution, gravity and controller gains) varies with arm configuration. This
  cube was the closest Large one to the base (r = 0.42 m), which is a different pose from the ones that worked.
- So a few mm of error was invisible to the move check but enough to land a fingertip on the cube's edge. Each
  failed attempt nudged the cube further (its yaw drifted 32° → 37°).

**Fix.** A pre-grasp hover:
1. Drop to 40 mm above the cube's top (`PreGraspClearance`), close enough that the arm is in nearly its grasp pose.
2. Dwell, then measure the grasp site's horizontal error against the object's **live** position.
3. If the error is > 3 mm (`CenterTolerance`), add it to an accumulated `m_PosCorrection`, re-command, and
   re-measure (up to `MaxCenterIterations` = 3). This is integral-style correction of a steady-state bias.
4. The final descent and the lift use the same correction, so the fingers come straight down around the cube.

The correction is reset on every approach because the bias depends on the arm's configuration. Each
`Centring:` log line shows how far off the arm actually was.

**Talking point.** "Converged" (within tolerance) isn't the same as "accurate enough for this grasp". The
fix measures where the arm actually is near the point where precision matters, rather than tightening a global
tolerance, which would make every move slower and more prone to timeouts.

**Tests.** `FakeSim.TrackingBias` adds a steady-state offset, and the fake fingers now push a cube when the
grasp site is more than `(FingerOpening − edge) / 2` off-centre.
- `TrackingErrorWithoutCentringPushesTheLargeCube`: an 11 mm bias, inside the 15 mm move tolerance, reproduces
  the in-editor failure.
- `TrackingErrorIsCentredOutBeforeDescent`: the same bias on 3 seeds finishes with zero collisions.

### 3.3 Detecting a bad grasp before it happens → push detection

The object's position is recorded at the start of the descent. If it moves more than `PushTolerance` (8 mm)
while the fingers come down, that's a collision. The arm aborts and backs off, and the retry reads the object
from its **new** position. The log line records gripper yaw, object yaw, the grasp site's offset and its height
above the cube, which is what made §3.2 diagnosable.

**Tests.** `PushedObjectIsReapproachedFromItsNewPosition`.

### 3.4 Placing on the stack as it physically is (`StackPlacement`)

Placement height and XZ come from the **live** pose of the top verified cube (`top.Y + top.Edge/2 + edge/2 + gap`),
not from precomputed heights. Small drift of lower cubes is followed instead of compounding. The cube is released
3 mm above contact with a short gripper-open ramp so it isn't flicked. `VerifyPlacement` waits for the cube to
settle, then checks XZ/Y tolerances, and fails the episode as `StackCollapsed` if any earlier cube on that stack
has moved.

**Tests.** `PlacementHeightComesFromTheLiveStack`, `MisplacedObjectIsPickedUpAgain`, `KnockedOverStackIsReportedAsCollapse`.

---

## 4. Independent evaluation (`StackEvaluator`, Stage 8)

The state machine checks its own work after every placement, but those checks use its own bookkeeping (which
objects it believes it placed) and its own tolerances. A benchmark can't grade itself that way, so the
evaluator is a separate judge:

- **Inputs:** the `TaskDefinition` (what the stacks *should* be) and `IWorldState` (where every object
  *physically is*). It never reads the plan, the state machine, or its placement list.
- **Attribution by physics:** each task object is assigned to the nearest stack base within 60 mm, and each
  column is sorted by height. Objects in no column are reported as unplaced.
- **Checks per stack** (plan §12): `ColorCorrect`, `ObjectCountCorrect`, `SizeOrderCorrect` (strictly smaller
  going up), `PositionWithinTolerance` (XZ from the stack centre), `SupportedByPreviousObject` (resting at
  contact height, centre over the cube below, and **flat**: tilt ≤ 10°), `StackStable` (nothing moved more than
  2 mm during a 1 s watch window), `AllObjectsPlaced`. `TaskSuccess` is the AND of all of them.
- **Why a tilt check:** a 60 mm cube leaning 15° on an edge only raises its centre ~7 mm, which is inside the
  height tolerance. Tilt is measured from the body's quaternion as the angle between world up and the closest
  face normal, so a cube resting flat on *any* face counts as 0°.
- **Stability needs time:** the evaluator is driven by `Update(dt)` like everything else. It snapshots poses,
  watches for `StableWindow` of sim time, then judges.
- **Partial credit:** `CorrectlyStacked` counts, per stack, the run of objects from the bottom that are the
  right object, in place, supported and still. That's useful for benchmark scoring later.
- **Disagreement is logged:** if the state machine says `Success` but the evaluator says FAIL (or the reverse),
  `EpisodeManager` warns, and the evaluator wins.

### Failure reasons (Milestone 5)

A pass/fail bit isn't enough to benchmark with; a failure has to say *why*.

- **`EvalFailure`**, ordered from most to least fundamental: `WrongColor`, `WrongOrder`, `Unstable`,
  `Unsupported`, `OutOfTolerance`, `Incomplete`. A report lists every category present, and the first one is
  the `PrimaryFailure`. The order matters because failures cascade: a cube on the wrong stack is also missing
  from its own, and the root cause is `WrongColor`, not `Incomplete`.
- **`EpisodeResult`** joins the two verdicts. The robot's `EpisodeMetrics` (attempts, duration, and its
  `FailureReason` if it gave up) is paired with the evaluator's report. `Success` comes from the evaluator.
  `Category` is:
  - `Success`;
  - the robot's own reason (`GraspFailed`, `Unreachable`, `StackCollapsed`, …) when it gave up, because it knows
    more specifically what went wrong;
  - `Eval.<EvalFailure>` when the robot believed it succeeded and the scene disagrees. These are the most
    interesting failures in a benchmark: silent ones.
- Each episode ends with one parseable line, which benchmark mode will aggregate:
  ```text
  [Result] seed=… task=ACF5… result=PASS category=Success stacked=15/15 robot=Success picks=15 places=15 duration=212.4s
  ```

**Talking point.** "The planner says I placed Green Small; the evaluator says Green Small isn't on the Green
stack." That's what makes this a benchmark rather than a scripted animation. Two ways to show it:
- **Offline:** `EvaluatorCatchesWhatTheStateMachineMissed` runs a clean episode, then a cube slides off after the
  robot's last check. The state machine still says `Success`, and the evaluator fails it.
- **Live in the editor:** set `EpisodeManager → Debug → DropPlanSteps` to 2. The state machine completes its
  shortened plan and reports Success. The evaluator reports `FAIL … (Incomplete)` with the two missing cubes,
  the disagreement warning fires, and the result line says `category=Eval.Incomplete`.

Real editor output (2026-10-09, same seed both times, 4 colours × 4 sizes, 16 cubes):

```text
[Result] seed=-1399230583 task=D8A130C84A4FE2C7 result=PASS category=Success stacked=16/16 robot=Success picks=16 places=16 duration=185.7s

# DropPlanSteps = 2
[Task] ReturnHome -> Success
[Eval] FAIL: 14/16 objects correctly stacked (Incomplete)
[Eval]   YELLOW FAIL: Yellow Medium < Yellow Large
[Eval]     - 2 objects, expected 4
[Eval]     - missing: LS_Obj_Yellow_Small
[Eval]     - missing: LS_Obj_Yellow_Tiny
[Eval] State machine reported Success but the evaluator says FAIL. The evaluator is the source of truth.
[Result] seed=-1399230583 task=D8A130C84A4FE2C7 result=FAIL category=Eval.Incomplete stacked=14/16 robot=Success picks=14 places=14 duration=163.3s
```

The first run also confirms that `GetObjectTilt` reads 0° for MuJoCo cubes resting flat: all 16 passed the
support check.

---

## 5. How each failure was worked

1. **Read the log, not the video.** Each failure path emits one line with the numbers needed to explain it
   (distances, yaws, heights, attempt counts).
2. **Check the assumptions with arithmetic.** Yaw modulo 90°, clearance budget vs. tolerance, reach radius.
3. **Reproduce in `FakeSim`** with a minimal fault model (a yaw convention, a tracking bias) and write the test
   that fails the same way the editor did.
4. **Fix in the engine-independent state machine**, re-run the full suite, then confirm in the editor.

## 6. Where to look in the code

| Concern | File |
|---|---|
| Task generation (seeded, collision-free layout) | `Generation/TaskGenerator.cs` |
| Pick/place order | `Planning/StackPlanner.cs` |
| Live stack placement | `Planning/StackPlanner.cs` (`StackPlacement`) |
| State machine, calibration, recovery | `StateMachine/TaskStateMachine.cs` |
| Engine-side arm | `Robot/PandaManipulator.cs` |
| Independent physical evaluation | `Evaluation/StackEvaluator.cs` |
| Episode orchestration | `Episodes/EpisodeManager.cs` |
| Offline arm + physics stand-in | `Tests/LuckySort.Core.Tests/FakeSim.cs` |
