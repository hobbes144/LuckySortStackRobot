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

## 5. Recording (Milestone 6)

Each episode is recorded in two complementary parts:

| Part | Holds | Written by |
|---|---|---|
| Lucky **Observer** dataset | robot joint state, actions, camera frames (engine-side Parquet + video) | the engine, with episode boundaries and labels driven by `ObserverSession` |
| **Episode record** (`Recordings/<session>/episode_NNNN.json`) | seed, config, fingerprint, objects (start pose, target, final physical pose and tilt), plan, sim-time timeline, full evaluation, result | `EpisodeRecorder` |
| **Session index** (`episodes.jsonl`) | one `EpisodeResult` line per episode | `EpisodeRecorder` |

Design decisions:
- **The Observer's success flag is the evaluator's verdict.** A dataset labelled by the robot's own opinion
  would mark the `DropPlanSteps` episode as a success. The training data inherits the benchmark's ground truth,
  not the controller's belief.
- **Per-pick labels.** Each distinct pick (`Pick Large Red -> RED[0]`) is registered as an Observer task, and
  `SetCurrentTaskIndex` switches it at the start of each pick attempt, so the dataset is segmented into
  sub-tasks rather than one long blob.
- **The sidecar exists because the Observer has no metadata API.** Without seed and config, a recorded
  episode can't be reproduced. With them, any episode can be regenerated exactly (Stage 12).
- **Sim time everywhere.** Timeline stamps come from the state machine's sim-time clock. Wall clock is used
  only to name the session folder.
- **Engine-independent and tested.** `EpisodeRecorder` and the record types live in `Recording/` and run against
  `FakeSim`. `SameSeedRecordsTheSameEpisode` shows that two runs of one seed produce byte-identical records
  (offline), so determinism is checked end to end, not just for task generation.
- **A record is just data.** Reloading `episode_NNNN.json` gives the seed, config, outcome and per-check
  breakdown that benchmark mode (Milestone 7) aggregates.

**Evidence of determinism in the engine itself.** Seed −1399230583 was run twice in the editor, at 14:45 and at
16:34. Both runs took exactly **185.7 s** of sim time with 16 picks and 16 places, and the same fingerprint
`D8A130C84A4FE2C7`. With a fixed timestep and all randomness seeded, MuJoCo plus the state machine reproduced the
whole episode, not just the task layout.

**Engine obstacles hit on the way** (details in `LUCKY_API_NOTES.md` §10):
- Observer recording is refused unless signed in to Lucky Hub. `ObserverSession` checks `Auth.IsAuthenticated()`
  and says so, rather than failing silently.
- Video capture needs FFmpeg (Tools → FFmpeg Settings).
- Scripts can't see the project folder (working directory is `System32`, and the assembly is loaded from
  memory). The project root is found from the compile-time `[CallerFilePath]`.
- The Observer writes a **LeRobot v3** dataset to `DataSessions/session_<time>/`: per-frame joint state and
  actions at 50 Hz, a `task_index` per frame, per-episode `success`, and camera video. Its per-scene
  `ObserverConfig` cut the first recording at **90 s** (`MaxEpisodeDurationSeconds`) at 256×256. Per-pick labels
  only take effect if registered **before** `StartRecording()`, so all 40 possible labels are registered up front.

---

## 6. Benchmark mode (Milestone 7)

One Play session runs N episodes on seeds `Seed, Seed+1, …` and ends with a summary.

```text
first frame: calibrate resetter (bodies, position frame, orientation frame) on the saved scene poses
episode 1:   qpos writes ─► settle ─► placement check ─► run ─► evaluate ─► record
episode k>1: ResetToInitialState + qpos writes + re-home arm ─► settle ─► placement check ─► run ─► evaluate ─► record
end:        BenchmarkSummary ─► log + benchmark_summary.json/.txt
```

**Resetting in place (`SceneResetter`).** Restarting Play per episode would be slow and would lose script state,
so the scene is reset between episodes. That isn't teleporting during operation (rule 7): it happens at the
episode boundary, like a real lab resetting a cell. Nothing about how MuJoCo bodies map to scene entities, or
how Hazel's frame maps to MuJoCo's, is documented, so the resetter **measures instead of assuming**:
- Bodies are matched to cubes by position.
- The frame rotation is either `GetAxisHzToMj()` or its inverse, whichever fits every cube with one translation.
- The orientation convention `C` (in `q_mj = R ⊗ q_hz ⊗ C`) is solved from every cube's saved pose and must agree across all 16.
- The fit spread and the qpos-vs-body agreement are logged. If they're off by more than 2 mm, it refuses to reset
  rather than silently mis-placing cubes.
- Every reset then goes through the same **placement check** as the first layout (position within 5 mm, yaw
  within 3°). A bad reset stops the benchmark instead of producing results from a wrong layout.
- `Debug → ResetSpike` cycles three layouts without running the robot, so the reset can be verified in seconds
  before committing to a long benchmark.
- Every layout, including the first, uses the same qpos write. Transform writes are only a logged fallback.

### 6.1 Bug story: "the layout didn't land" (three stacked bugs, 2026-10-09 → 10-12)

The in-Play reset failed in the editor on every run. It turned out to be **three separate bugs layered on top of each
other**. Each fix exposed the next, and each was found by a self-check rather than by watching the viewport.
**Resolved 2026-10-12:** `ResetSpike` passes on seeds 5555–5557 and then 5555–5564 (10/10 layouts `placed by qpos reset — OK`).

**Bug 1: a cube without its own physics body (scene authoring).**
- *Symptom.* One cube, `LS_Obj_Red_Small`, "landed" 430 mm from its target, at the same pose to the millimetre on every run.
- *Caught by.* The resetter's body count: **15 MuJoCo free bodies for 16 cubes**, so calibration refused to run, as designed.
- *Root cause.* In the editor hierarchy, Red_Small had been dragged under `LS_Obj_Yellow_Large`. A child entity compiles
  into its parent's MuJoCo body, so the cube lost its free joint and rode along rigidly with Yellow_Large.
- *Found by.* MuJoCo names bodies `lucky_body_<entity ID>`; mapping them back to the scene showed which cube had no free joint.
- *Dead ends.* Overlapping transform writes "flinging" cubes, then a stale compiled-model cache. Both were dropped as
  soon as the evidence (exact repeatable pose, body count) contradicted them.
- *Fix + hardening.* Re-rooted the cube; `ReadWorkspace` now refuses to start if any pool cube has a parent, and names it.

**Bug 2: every cube in the right place, every rotated cube with the wrong yaw (frame convention).**
- *Symptom.* `Placement check FAILED: 7/16`. All 16 positions were correct.
- *The clue.* The 7 that passed were **exactly the 7 parked cubes, which are always written with yaw 0**. All 9 task
  cubes (random yaw) failed. A bug that only shows up at non-zero yaw points at the rotation write, not at physics or position.
- *Root cause.* Hazel is Y-up and MuJoCo is Z-up. Calibration measured the position frame as a +90° turn about X
  (`HzToMj = (0.707, 0, 0, 0.707)`, offset 0, spread 0.00 mm). The rotation write assumed a cube's MuJoCo quaternion is
  simply "yaw about MuJoCo's up", which is only true if the engine converts orientations as `R·q·R⁻¹`. The engine
  uses a different convention. Yaw 0 writes the identity quaternion, which is right or wrong-but-symmetric under any
  convention, so the parked cubes hid the bug.
- *Measured answer.* `C = [w 1, x 0, y 0, z 0]` → **`q_mj = R ⊗ q_hz`**, with all 16 cubes agreeing to 0.00°. The engine
  converts only the world frame, not the body's local axes. So Hazel yaw θ is `R ⊗ Ry(θ)` in MuJoCo; the old code wrote
  `Rz(θ) = R ⊗ Ry(θ) ⊗ R⁻¹`, which differs by a 90° tilt about X. On a cube, that tilt turns Hazel's "forward" axis
  vertical, so the yaw reading is meaningless, while the cube still looks flat (a cube looks the same on any face).
- *Fix.* Measure the orientation frame instead of assuming it, as the position frame already was. `Calibrate` solves
  `q_mj = R ⊗ q_hz ⊗ C` for `C` on every cube's saved scene pose (editor rotation vs. the qpos quaternion). It requires
  all 16 cubes to agree within 2° and refuses to reset otherwise. Writes become `R ⊗ yaw ⊗ C`. The pool cubes rest
  in arbitrary saved orientations, which makes this measurement well-conditioned.
- *A misleading test along the way.* An earlier "axis self-test" wrote a 28.6° yaw to one cube and read it straight
  back: **0.0°**. That looked like a failed write, but the readback came from the same frame, before MuJoCo recomputed
  body poses, so it would have read 0° whatever was written. It was removed; the new check uses only existing state,
  so there is nothing to read back stale.

**Bug 3: the arm slumped before doing anything (reset ordering).**
- *Symptom.* At the start of Play the arm sagged into a bent pose and never moved. It looked like a stacking or IK regression.
- *Root cause.* No stacking code had run: the placement check (bug 2) stopped the episode before `StartTask`. The arm
  was disturbed by `ResetToInitialState`, which the new benchmark code called **on the first frame of every Play**, even
  for episode 1. That reset restores MuJoCo's state, actuator controls included, underneath the motion graph, and with
  no task running, nothing re-commanded the arm. The last working version (before benchmark mode) never called it.
- *Fix.* Episode 1 skips the MuJoCo reset, since the state is already initial and only the cube writes are needed.
  Episodes 2+ do the full reset, then `RehomeArm()` commands the grasp site back to its start-of-Play pose with the
  gripper open, and the placement check waits `RehomeSeconds` for it.

**Outcome: first full benchmark (2026-10-12).** With all three fixes in, `BenchmarkEpisodes = 10` on seeds
5555–5564 (4 colours × 4 sizes, random positions and yaws) ran unattended in one Play session:

```text
Success rate 100 % (10/10), 95% CI 72 %–100 %
Categories: Success 10
Duration (all):     mean 144.5s ± 18.0  (min 112.8, median 141.0, max 175.4, n=10)
Picks per object (successful episodes) 1.00, objects correctly stacked 100.0 %, robot/evaluator disagreements 0
```

Every episode after the first went through `ResetToInitialState`, the measured qpos writes and `RehomeArm()`, and no
object needed a retry. Per-episode records and the summary are in `Recordings/20261012_005521/`. Note the honest reading:
10/10 supports "at least ~72 %" with 95% confidence, not "100 % reliable"; a longer run narrows that.

**Talking points.**
- **Self-checks turned silent corruption into loud, early failures.** The body count caught bug 1 and the placement
  check caught bug 2, before either could produce a benchmark full of mysterious "robot failed" episodes.
- **Look at which cases pass, not just which fail.** "Passes at yaw 0, fails otherwise" pinned bug 2 to one line of maths.
- **A symptom on the robot doesn't mean a robot bug.** Bug 3 looked like an arm/IK problem; reading the log order
  showed the task had never started.
- **Measure undocumented conventions, then verify against data you didn't write.** Position, body mapping and now
  orientation are all calibrated from the saved scene, and each refuses to run when the measurements disagree.
- **Check a test before trusting it.** The same-frame readback "proved" a failure that wasn't there.

### 6.2 50-seed benchmark and failure analysis (2026-10-12)

`BenchmarkEpisodes = 50`, seeds 5555–5604, 4 colours × 4 sizes, random positions and yaws, one unattended Play session
(`Recordings/20261012_013747/`):

```text
Success rate 94 % (47/50), 95% CI 84 %–98 %
Categories: Success 47, GraspFailed 2, Unreachable 1
Duration (all):     mean 141.5s ± 32.3  (min 29.2, median 145.6, max 177.0, n=50)
Duration (success): mean 148.2s ± 18.2  (min 111.2, median 146.7, max 177.0, n=47)
Picks per object (successful episodes) 1.00, objects correctly stacked 94.5 %, robot/evaluator disagreements 0
Failed seeds (replay with Seed = …): 5569, 5594, 5604
```

**How to read it.**
- **94 %, 95% CI 84–98 %.** The 10-seed run's "100 %" (CI 72–100 %) was consistent with this; more seeds narrowed the
  claim and surfaced the rare failures. That's why the summary reports an interval, not just a rate.
- **Per pick, it's ~99.5 % reliable.** The 50 tasks held 619 objects in total; 3 picks failed beyond recovery. One bad pick
  ends the episode, so a ~0.5 % per-pick failure rate compounds to ~6 % over a 10–16-object task.
- **0 robot/evaluator disagreements.** Every failure was one the robot itself noticed and reported correctly; none was a
  silent "robot says success, scene says otherwise".
- **Failures are short.** All three ended at 29–39 s on the 2nd–3rd object. The robot gives up cleanly rather than flailing.

**The three failures** (from each `episode_NNNN.json` timeline; replay any of them by setting `Seed`):

| Seed | Category | Object | What the log shows |
|---|---|---|---|
| 5569 | GraspFailed | Red_Tiny (30 mm), r = 0.63 m | First descent stalled 25 mm short and nudged the cube 7 mm. The retry came down with the grasp site 24 mm off the cube's centre and pushed it 8 mm. Yaw was right (160° ≡ −20° mod 90°). |
| 5604 | GraspFailed | Red_Small (40 mm), r = 0.63 m | Two descents, both pushed the cube 8 mm, with the grasp site ~13 mm off centre and yaw within 1° (mod 90°). |
| 5594 | Unreachable | Red_Small (40 mm), r = 0.31 m | Pre-grasp hover target 90 mm above the surface; the arm stopped 45 mm above it, 50 mm from target, on every attempt. The cube is near the inner edge of the reach annulus (0.28 m), where IK can't bring the hand down vertically. |

**The pattern: retries repeat the same motion.** In every failed episode, each retry logged the *same* numbers (same
50 mm hover error, same 8 mm push at the same offset). The recovery path re-runs an identical approach, so a deterministic
failure stays failed. Candidate fixes, in order of expected value (not yet implemented):
1. **Vary the retry.** A cube is symmetric under 90° turns, so a retry can approach at yaw + 90°. That changes the wrist
   configuration, and with it both the IK solution (5594) and the tracking error (5569, 5604).
2. **Re-centre closer to the object** on retries (a lower `PreGraspClearance`), so less drift accumulates during the final descent.
3. **Tighten the generator's reach annulus** only if a longer run shows failures clustering by radius. In this run, objects
   at r ≈ 0.30–0.35 m and 0.60–0.70 m were stacked at much the same rate as mid-range ones, so 3 failures aren't enough evidence.

**The summary (`BenchmarkSummary`, engine-free).**
- **Success rate with a 95% Wilson interval.** At n = 10, "9/10" is a 60–98% claim, not "90%". The Wilson interval
  stays sensible at 0% and 100%, where the normal approximation collapses to ±0.
- **Failure categories**, most frequent first: the robot's own reason when it gave up, `Eval.*` when it thought
  it succeeded. **Robot/evaluator disagreements** are counted separately, since those are the silent failures.
- **Duration** (mean ± sample std-dev, min/median/max), for all episodes and for successful ones only.
- **Picks per object** over successful episodes (1.00 = never retried), and the **fraction of objects correctly
  stacked** across all episodes as partial credit.
- **Failed seeds**: every failure can be replayed exactly by setting `Seed`.
- Seeds must be ≥ 0. `System.Random(seed)` uses |seed|, so −n and n would silently be the same task.

**Tested offline.** `OfflineBenchmarkSummaryMatchesInjectedFaults` runs 40 seeds through `FakeSim` with a
deterministic fault per seed (always-slipping grasp → `GraspFailed`, dropped plan step → `Eval.Incomplete`) and
checks every count, the rate, the disagreements and the failed-seed list against the known answer.
`OfflineBenchmarkIsReproducible` checks that two runs give identical summaries.

---

## 7. How each failure was worked

1. **Read the log, not the video.** Each failure path emits one line with the numbers needed to explain it
   (distances, yaws, heights, attempt counts).
2. **Check the assumptions with arithmetic.** Yaw modulo 90°, clearance budget vs. tolerance, reach radius.
3. **Reproduce in `FakeSim`** with a minimal fault model (a yaw convention, a tracking bias) and write the test
   that fails the same way the editor did.
4. **Fix in the engine-independent state machine**, re-run the full suite, then confirm in the editor.

## 8. Where to look in the code

| Concern | File |
|---|---|
| Task generation (seeded, collision-free layout) | `Generation/TaskGenerator.cs` |
| Pick/place order | `Planning/StackPlanner.cs` |
| Live stack placement | `Planning/StackPlanner.cs` (`StackPlacement`) |
| State machine, calibration, recovery | `StateMachine/TaskStateMachine.cs` |
| Engine-side arm | `Robot/PandaManipulator.cs` |
| Independent physical evaluation | `Evaluation/StackEvaluator.cs` |
| Episode orchestration | `Episodes/EpisodeManager.cs` |
| Episode record + session index (engine-free) | `Recording/EpisodeRecorder.cs`, `Recording/EpisodeRecord.cs` |
| Observer episode boundaries + per-pick labels | `Episodes/ObserverSession.cs` |
| In-Play scene reset (benchmark) | `Episodes/SceneResetter.cs` |
| Benchmark seeds + summary maths (engine-free) | `Benchmark/BenchmarkSummary.cs` |
| Offline arm + physics stand-in | `Tests/LuckySort.Core.Tests/FakeSim.cs` |
