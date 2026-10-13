# LuckySort

**A procedural robotic manipulation benchmark built on [Lucky Engine](https://docs.luckyrobots.com/stable/).**
A simulated Franka Panda arm sorts randomly generated coloured cubes into one stack per colour, largest at the bottom.
Every task comes from a seed, the arm is driven by IK on real MuJoCo rigid bodies (no teleporting), and the result is
judged by an **independent evaluator that only looks at the physical outcome**, never at what the robot believes it did.

It was built as an engineering demo of robotics systems design: closed-loop control against an under-documented engine,
reproducible tasks, honest evaluation, and failure analysis driven by recorded data.

> **Status:** Milestones 1-8 complete (pick/place through benchmark mode and presentation tooling).
> Vision (Milestone 9) is planned but not started. See [Status and limitations](#status-and-limitations).

## Results

Benchmark runs use 4 colours x 4 sizes (up to 16 cubes per task), random positions and yaws, clutter off, one unattended
Play session per run. Success is the evaluator's verdict, and intervals are 95% Wilson intervals.

| Run | Seeds | Passed | 95% CI | Notes |
|---|---|---|---|---|
| First full benchmark | 5555-5564 | 10/10 | 72-100% | mean 144.5 s per episode, 1.00 picks per object |
| 50-seed benchmark, before grasp-axis fix | 5555-5604 | 47/50 (94%) | 84-98% | `GraspFailed` x2, `Unreachable` x1, 6 collisions |
| After fixes, on seeds never used to design them | 6000-6028, 7000-7029 | 59/59 | 94-100% | 0 collisions, 0 robot/evaluator disagreements |

Across the 50-seed run that is roughly a 99.5% per-pick success rate, which compounds to about 6% failure over a task of
10-16 objects. A "100%" over 10 episodes only supports "at least about 72%", which is why the summary reports an
interval rather than just a rate. Details and the failure analysis are in
[`docs/ENGINEERING_NOTES.md`](docs/ENGINEERING_NOTES.md) section 6.

## What it does

- **Procedural tasks.** `TaskGenerator` turns a `TaskConfig` and a seed into a `TaskDefinition`: 1-4 colours, 1-4 size
  tiers (Large 60 mm, Medium 50, Small 40, Tiny 30), random positions and rotations, with a collision-free layout. The same
  seed always gives the same task, and `TaskDefinition.Fingerprint()` identifies it.
- **A closed-loop pick-and-place state machine.** `TaskStateMachine` runs
  `SelectTarget -> MoveAboveObject -> AlignGripper -> CenterOverObject -> MoveToObject -> Grasp -> Lift -> VerifyGrasp ->
  MoveToStack -> Place -> Release -> Retreat -> VerifyPlacement`. Every state has an entry action, a completion condition,
  a timeout and a failure path, and failed checks go to a recovery path with a retry budget.
- **Runtime calibration instead of assumptions.** The gripper-yaw convention, the arm's steady-state tracking error and
  the physical finger axis are all measured in the loop, then corrected (`AlignGripper`, `CenterOverObject`,
  `PandaManipulator.FingerAxis`).
- **Clearance-aware grasping.** Before each pick, both candidate finger axes are scored against live neighbour positions,
  and the grasp is turned 90 degrees if the fingers would land on another cube (`ChooseGraspAxis`, `CorrectFingerAxis`).
- **Live stack placement.** Place height and position come from the live pose of the top verified cube, so drift in lower
  cubes is followed instead of compounded (`StackPlacement.PlaceCentre`).
- **Independent evaluation.** `StackEvaluator` checks colour, count, size order, position tolerance, support (including
  tilt), stability over a 1 s watch window, and completeness. Failures carry a reason (`EvalFailure`), and when the robot
  and the evaluator disagree the evaluator wins and a warning is logged.
- **Recording.** Each episode produces a Lucky Observer dataset (LeRobot v3, joint state and actions at 50 Hz, camera video,
  per-pick sub-task labels, success label = evaluator verdict) plus a JSON episode record with the seed, config,
  fingerprint, object poses, plan, timeline and full evaluation.
- **Benchmark mode.** One Play session runs N episodes on consecutive seeds with an in-place scene reset between episodes
  (`SceneResetter`), then writes a summary: success rate with a Wilson interval, failure categories, duration statistics,
  picks per object, partial credit, robot/evaluator disagreements, and the list of failed seeds to replay.
- **Presentation tooling.** An on-screen overlay (seed, fingerprint, state, step, attempt, per-stack progress, the
  evaluator's checklist, benchmark tally), world-space markers (stack bases, current target, arm target, chosen finger axis
  and landing points), switchable camera views, and replay of any seed.
- **Retry variation and deferral.** Retries approach from the other side of the cube (`VaryRetries`), and a pick that runs
  out of attempts is deferred along with its stack so other stacks are built first (`DeferFailedPicks`).

## Architecture

```text
 TaskConfig + seed
        |
        v
 TaskGenerator --> TaskDefinition --> StackPlanner --> PlannedStep[]        (what, in which order)
 (Generation/)      (Core/)           (Planning/)           |
                                                            v
                                                   TaskStateMachine         (how, with checks)
                                                   (StateMachine/)
                                                     |           ^
                                         IManipulator|           |IWorldState
                                                     v           |
                                        PandaManipulator     SceneWorldState
                                        (motion-graph IK,    (MuJoCo body poses
                                         actuator8 gripper)   via entity transforms)
                                                     |           ^
                                                     v           |
                                                ---- Lucky Engine / MuJoCo ----
                                                            |
                                                            v
                                                     StackEvaluator          (did it physically work?)
                                                     (Evaluation/)
```

- The **planner reads task metadata**, the **state machine closes the loop on measured state**, and the **evaluator reads
  only physical state**. None of them trusts the layer above it.
- Everything above the engine line except `PandaManipulator` and `SceneWorldState` is **engine-independent**
  (`System.Numerics` only) and compiled straight into the xunit project.
- `FakeSim` implements both `IManipulator` and `IWorldState`: a kinematic stand-in for the arm and cubes with fault
  injection (slips, pushes, drop offsets, yaw convention, tracking bias, fingers hitting neighbours). Whole episodes run
  offline in milliseconds, so each in-editor failure was reproduced as a failing unit test before it was fixed.

## Engineering highlights

Each of these was found in a real editor run, diagnosed from logs and recorded data, reproduced in `FakeSim`, and fixed.
Full write-ups are in [`docs/ENGINEERING_NOTES.md`](docs/ENGINEERING_NOTES.md).

- **Unknown gripper-yaw convention.** Measured at runtime (yaw modulo 90 degrees is all a cube grasp needs) and corrected
  in a loop instead of hard-coded.
- **Tracking error vs. finger clearance.** A Large cube leaves about 10 mm per finger, but moves were accepted within
  15 mm. A pre-grasp hover now measures the real error near the cube and corrects it.
- **"The layout didn't land": three stacked bugs.** A cube re-parented in the editor lost its physics body (caught by a
  body count), a wrong orientation convention in the reset only appeared at non-zero yaw (caught by a placement check), and
  a reset on frame 1 made the arm slump. Each was caught by a self-check, and the resetter now calibrates conventions from
  data and refuses to run if the measurements disagree.
- **Failures caused by objects that were never picked.** The generator spaced cubes by their own size, not by the open
  gripper's reach. The evidence was in the final positions of cubes the plan never touched. The fix was a clearance-aware
  grasp axis, validated on 59 unseen seeds.

## Requirements

- Windows x64 and **Lucky Engine 2026.1.2**. The project uses the engine's C# scripting, MuJoCo physics, and an
  undocumented-in-places API surface; confirmed behaviour is recorded in [`docs/LUCKY_API_NOTES.md`](docs/LUCKY_API_NOTES.md).
- .NET 9 SDK, for building the scripts, the scene tool and the tests.
- For recording only: a Lucky Hub sign-in (the Observer refuses to record otherwise) and FFmpeg configured in the editor
  (Tools -> FFmpeg Settings).

## Getting started

1. **Run the tests** (no engine needed, since the logic under test is engine-independent):
   ```text
   dotnet test Tests/LuckySort.Core.Tests
   ```
2. **Generate the workspace** into the scene. The Lucky editor must be closed, because editor autosave overwrites the
   scene file:
   ```text
   dotnet run --project Tools/SceneBuilder -- LuckySortStack
   ```
   This writes the mat, stack markers, and the 16-cube object pool (4 colours x 4 sizes, tagged `LS_Obj_<Color>_<Size>`).
   Generated entities are tagged `LS_*` with deterministic IDs, and re-running replaces them.
3. **Build the scripts** (the editor hot-reloads the output):
   ```text
   dotnet build LuckySortStack/Assets/Scripts/Client/LuckySortStack.csproj
   ```
4. **Open `LuckySortStack` in the Lucky editor.** Turn on *Settings -> Scene -> Scripts -> Enable Free Step Update Scripts*
   (without it, markers flicker and key presses are missed), then press **Play**.

A single `EpisodeManager` in the scene reads the workspace, generates the task, runs the state machine, evaluates, and
records. Pool cubes must be root entities in the scene hierarchy, because a child entity is compiled into its parent's
physics body. `EpisodeManager` refuses to start if one has a parent.

## Using it

Settings live on the `EpisodeManager` component in the inspector.

| To do this | Set |
|---|---|
| Run one task on a chosen seed | `Seed` (non-negative), `BenchmarkEpisodes = 1` |
| Run a benchmark | `BenchmarkEpisodes = N`. Episodes use seeds `Seed ... Seed+N-1` |
| Reproduce the published numbers | `PileCount = 0`, 4 colours, 4 size tiers, and seeds from the table above |
| Check the scene reset without running the robot | `Debug -> ResetSpike` |
| See the evaluator catch a silent failure | `Debug -> DropPlanSteps = 2` |
| A/B a robustness feature | `Motion -> ChooseGraspAxis`, `VaryRetries`, `DeferFailedPicks` |

While playing: **C** cycles camera views (authored, overview, front, top-down, follow), **R** replays the current seed,
**F** replays the last failed seed, and **P** replays `ReplaySeed`. Seeds must be 0 or greater, because `System.Random`
uses the absolute value and -n and n would silently be the same task.

Output locations (both gitignored):

- `LuckySortStack/Recordings/<session>/`: `episode_NNNN.json`, `episodes.jsonl`, `benchmark_summary.json` and `.txt`.
- `LuckySortStack/DataSessions/session_<time>/`: the Observer's LeRobot dataset.

Each episode also logs one parseable line:

```text
[Result] seed=... task=... result=PASS category=Success stacked=15/15 robot=Success picks=15 places=15 duration=212.4s
```

## Repository layout

```text
docs/                         plan, engineering notes, engine API notes, design docs
LuckySortStack/               the Lucky Engine project
  Assets/Scenes/Main.hscene   start scene
  Assets/Scripts/Client/Source/
    Core/ Generation/ Planning/ StateMachine/ Evaluation/ Recording/ Benchmark/ Presentation/ Robot/Abstractions/
                              engine-independent logic (System.Numerics only), unit-tested
    Robot/PandaManipulator.cs IManipulator over the motion graph and the gripper actuator
    Episodes/                 scene-side orchestration: EpisodeManager, SceneWorldState, SceneResetter, ObserverSession
    UI/                       DebugOverlay (HUD and world markers), PresentationCamera
  Assets/ContentVault/Robots/Panda/   Panda MJCF and motion graph (see its own license)
Tools/SceneBuilder/           generates the workspace into the scene file
Tests/LuckySort.Core.Tests/   xunit tests plus FakeSim
```

## Documentation

| Document | What it covers |
|---|---|
| [`docs/ENGINEERING_NOTES.md`](docs/ENGINEERING_NOTES.md) | Problems found in the editor, how each was diagnosed and fixed, benchmark results and failure analysis |
| [`docs/LUCKY_API_NOTES.md`](docs/LUCKY_API_NOTES.md) | Confirmed Lucky Engine APIs and behaviours, including undocumented ones |
| [`docs/LuckySort_Idea_Plan.md`](docs/LuckySort_Idea_Plan.md) | The original full specification and milestone plan |
| [`docs/CLUTTER_DESIGN.md`](docs/CLUTTER_DESIGN.md) | Design and status of cluttered starts |
| [`docs/VISION_PLAN.md`](docs/VISION_PLAN.md) | Build plan for camera-based perception (not started) |
| [`CLAUDE.md`](CLAUDE.md) | Working rules and a milestone checklist used during development |

## Status and limitations

**Complete:** single pick/place, one stack, multiple colours, procedural tasks, robust evaluation with failure reasons,
recording, benchmark mode, and the on-screen overlay.

**Implemented, with verification still to do in the editor:**
- Camera views and replay build and pass their unit tests, but have not yet been checked in Play.
- Deferring failed picks (`DeferFailedPicks`) is unit-tested but has not been benchmarked in the editor.
- Cluttered starts (`PileCount`, `MaxPileHeight`, `AllowInvertedPiles`, `Planning/FreeSpace`) are generated, unstacked and
  unit-tested, and the scene reset survives them, but the real-arm behaviour is not yet benchmarked. All published results
  are with clutter off. Note that the `EpisodeManager` field default for `PileCount` is non-zero, so set it to 0 to
  reproduce them.

**Planned, not started:** vision (Milestone 9, see [`docs/VISION_PLAN.md`](docs/VISION_PLAN.md)). Today the robot reads
object positions from the engine rather than perceiving them with its camera.

**Known limitations:**
- Cubes only, drawn from a fixed pre-authored pool of 16, because runtime spawning is undocumented in this engine.
- Windows and Lucky Engine 2026.1.2 only. Several engine behaviours were reverse-engineered, so a different engine version
  may need recalibration.
- The scene's Observer config cuts recordings at 90 s and 256x256 by default, which is shorter than a full episode.
  Raise `MaxEpisodeDurationSeconds` and the recording camera resolution for full-length datasets.
- Benchmark episodes take about 2.5 minutes of simulated time each, so large runs take a while.

## Acknowledgements

The Franka Emika Panda description under `Assets/ContentVault/Robots/Panda/` is a third-party MJCF model and carries its
own license and changelog. Lucky Engine is a product of Lucky Robots.
