# LuckySort --- Procedural Robotic Color & Size Stacking Benchmark

## Build Plan / Claude Execution Specification

**Project goal:** Build a polished Lucky Engine robotics simulation that
demonstrates strong understanding of robotics simulation, engine
architecture, procedural task generation, robot control, physics,
evaluation, determinism, and data recording.

**Primary demonstration:** A robot is placed in a workspace containing a
procedurally generated collection of colored objects with different
physical sizes. The robot must pick up the objects and construct one
stack per color, with the largest object at the bottom and progressively
smaller objects above it.

The important distinction is that this should **not** be a single
hard-coded robot animation. It should be a reusable benchmark/task
system that can generate different valid tasks, execute them, evaluate
the result, and record the episode.

------------------------------------------------------------------------

# 1. Project Vision

## 1.1 Core task

Each generated episode contains:

-   A robot arm with a gripper.
-   A workspace/table.
-   A configurable number of colors.
-   A configurable number of object sizes.
-   One or more objects for each color/size combination.
-   Randomized starting positions and orientations.
-   A target stacking area for each color.
-   A planner/controller that determines the required pick/place
    sequence.
-   A success evaluator that verifies the completed stacks.

### Example

A generated episode might contain:

  Color   Objects
  ------- ----------------------------
  Red     Large, Medium, Small
  Blue    Large, Small
  Green   Large, Medium, Small, Tiny

The required result is:

``` text
RED STACK       BLUE STACK       GREEN STACK

   Small           Small             Tiny
   Medium                           Small
   Large           Large            Medium
                                   Large
```

The exact number of colors and sizes should be configurable.

## 1.2 What the demo should demonstrate

The project should communicate that the developer understands:

1.  Entity/component-based engine architecture.
2.  C# scripting and engine lifecycle.
3.  MuJoCo physics and physical object interaction.
4.  RobotControllerComponent.
5.  Motion-graph IK.
6.  Gripper control.
7.  Fixed-timestep simulation.
8.  Procedural scene/task generation.
9.  State-machine/task planning.
10. Physics-aware success evaluation.
11. Deterministic randomization and replay.
12. Sensor integration.
13. Observer/data recording.
14. Experiment statistics.
15. Optional Python/gRPC automation.

The emphasis should be on **systems design**, not on creating the most
sophisticated AI possible.

------------------------------------------------------------------------

# 2. Architecture

The intended architecture is:

``` text
                  ┌─────────────────────┐
                  │   Task Generator    │
                  │ seed / colors /     │
                  │ sizes / positions   │
                  └──────────┬──────────┘
                             │
                             ▼
                  ┌─────────────────────┐
                  │ Procedural Scene    │
                  │ objects + targets   │
                  └──────────┬──────────┘
                             │
                             ▼
┌───────────────┐   ┌─────────────────────┐
│ Camera/Sensor │──►│   Task Planner      │
└───────────────┘   │ grouping + ordering │
                    └──────────┬──────────┘
                               │
                               ▼
                    ┌─────────────────────┐
                    │ Task State Machine  │
                    │ search → grasp →    │
                    │ lift → move → place │
                    └──────────┬──────────┘
                               │
                               ▼
                    ┌─────────────────────┐
                    │ Robot Controller    │
                    │ IK + gripper        │
                    └──────────┬──────────┘
                               │
                               ▼
                    ┌─────────────────────┐
                    │      MuJoCo         │
                    │ physics / contacts  │
                    └──────────┬──────────┘
                               │
                               ▼
                    ┌─────────────────────┐
                    │   Stack Evaluator   │
                    │ color / order /     │
                    │ stability / count   │
                    └──────────┬──────────┘
                               │
                    ┌──────────┴──────────┐
                    ▼                     ▼
             Success / Failure      Episode Metrics
                    │                     │
                    └──────────┬──────────┘
                               ▼
                         Lucky Observer
                               │
                               ▼
                         Dataset / Stats
```

Optional advanced path:

``` text
Lucky Engine
     │
     │ gRPC
     ▼
Python Experiment Runner
     │
     ├── generate seeds
     ├── reset episodes
     ├── step simulation
     ├── collect metrics
     └── aggregate results
```

------------------------------------------------------------------------

# 3. Development Principles

## 3.1 Build the simulation before the AI

Do not begin by attempting machine learning or computer vision.

First make the deterministic scripted system work.

The project should prove:

> Given a generated task, the system can reason about the required
> ordering, manipulate the objects, and determine whether the physical
> result is correct.

A learned policy can be added later if time allows.

## 3.2 Separate task logic from robot control

Do not put the entire task into one giant `SortingTask.cs`.

Separate:

-   task generation
-   object metadata
-   planning
-   robot control
-   state machine
-   evaluation
-   episode management
-   randomization
-   recording

This is important both for maintainability and for demonstrating
engineering judgment.

## 3.3 Treat the task as data

The task should be represented explicitly.

Conceptually:

``` text
TaskDefinition
    seed
    colors[]
    objects[]
    targetStacks[]
    difficulty
```

Each object should have information such as:

``` text
ObjectDefinition
    color
    size
    physical dimensions
    initial position
    initial rotation
    target stack
    target order
```

The physical geometry must correspond meaningfully to the logical size.

------------------------------------------------------------------------

# 4. Stage 0 --- Project Setup and Lucky Familiarization

## Goal

Create a clean Lucky Engine project and understand the reference
examples before implementing custom logic.

## Tasks

1.  Create the project.
2.  Install/select a suitable robot.
3.  Open the built-in pick-and-place example.
4.  Open the Piper Pattern Stacking example.
5.  Identify:
    -   robot entity
    -   RobotControllerComponent
    -   motion graph
    -   gripper
    -   scene objects
    -   scripts
    -   Observer recording
6.  Create the LuckySort scene.
7.  Verify a simple C# script executes.
8.  Verify Play/Stop/Reset behavior.

## Lucky elements to understand

-   Entity
-   Component
-   Script component
-   Transform
-   RobotControllerComponent
-   Motion graph
-   MuJoCo physics
-   Scene hierarchy
-   Content Vault
-   Time Manager
-   Observer

## Definition of done

A blank LuckySort scene exists with:

-   robot
-   table/workspace
-   camera
-   one test object
-   one target
-   functioning C# script
-   robot can execute a simple movement

------------------------------------------------------------------------

# 5. Stage 1 --- Physical Workspace

## Goal

Create the physical environment in which all later systems operate.

## Tasks

Build:

-   table/work surface
-   workspace boundary
-   robot base
-   object spawn region
-   stack target locations
-   optional visual target markers
-   lighting/camera

Create reusable object assets for:

-   Large
-   Medium
-   Small
-   optional Tiny

Each object should have:

-   visual geometry
-   collision geometry
-   rigid-body physics
-   material/color
-   physical dimensions
-   mass appropriate to its size

## Important principle

Do not represent size purely as a variable.

The MuJoCo geometry should actually become physically larger/smaller.

This allows the size ordering requirement to have physical meaning.

## Lucky considerations

Use Lucky's physics/MuJoCo representation for the objects rather than
manually teleporting them during normal gameplay.

The robot should interact with actual rigid bodies.

## Definition of done

Pressing Play produces a stable physical scene where:

-   objects fall/settle naturally.
-   objects collide correctly.
-   the robot can reach the workspace.
-   objects can physically be grasped/moved.
-   stack locations are visually identifiable.

------------------------------------------------------------------------

# 6. Stage 2 --- Object and Task Data Model

## Goal

Represent the sorting problem independently of the scene.

Create a clean data model.

Suggested conceptual types:

``` csharp
enum ObjectColor
{
    Red,
    Blue,
    Green,
    Yellow,
    Purple
}

enum ObjectSize
{
    Tiny,
    Small,
    Medium,
    Large
}

class SortObjectData
{
    ObjectColor Color;
    ObjectSize Size;
    Entity Entity;
    int TargetStack;
    int TargetOrder;
}

class StackTarget
{
    ObjectColor Color;
    Vector3 Position;
    List<SortObjectData> ExpectedObjects;
}

class TaskDefinition
{
    int Seed;
    List<ObjectColor> Colors;
    List<SortObjectData> Objects;
    List<StackTarget> Targets;
}
```

Exact API/types should be adapted to Lucky's current scripting API.

## Definition of done

The application can print/log a generated task such as:

``` text
Seed: 183742

RED:
  Large
  Medium
  Small

BLUE:
  Large
  Small

GREEN:
  Large
  Medium
  Small
  Tiny
```

without requiring the robot to move.

------------------------------------------------------------------------

# 7. Stage 3 --- Procedural Task Generator

## Goal

Generate different valid tasks from a seed.

## Parameters

At minimum:

``` text
Seed
Number of colors
Minimum objects per color
Maximum objects per color
Number of size tiers
Randomize positions
Randomize rotations
Workspace bounds
```

Optional:

``` text
Object spacing
Stack spacing
Difficulty
Obstacle count
Physics randomization
```

## Generation algorithm

1.  Select colors.
2.  Select the number of objects for each color.
3.  Assign sizes.
4.  Create object definitions.
5.  Assign each color to a target stack.
6.  Generate random positions.
7.  Generate random rotations.
8.  Spawn physical objects.
9.  Record the expected stack order.

## Important constraint

A generated task must always have a valid solution.

For each color:

``` text
largest → ... → smallest
```

should produce the intended stack.

## Determinism

The same seed must produce the same task.

Example:

``` text
Seed 1234
    ↓
same colors
same object sizes
same spawn positions
same rotations
same target locations
```

This is extremely valuable for debugging and regression testing.

## Definition of done

Changing the seed produces different valid layouts.

Reusing the same seed produces the same layout.

------------------------------------------------------------------------

# 8. Stage 4 --- Task Planner

## Goal

Convert the generated task into a concrete sequence of robot actions.

The planner should not care about individual animation details.

Its job is to answer:

> What object should the robot manipulate next, and where should it put
> it?

## Planning algorithm

For every color:

1.  Gather all objects belonging to that color.
2.  Sort by physical/logical size descending.
3.  Produce the required stack sequence.

Example:

``` text
GREEN:
    Large
    Medium
    Small
    Tiny
```

becomes:

``` text
Pick Large Green
Place Large Green
Pick Medium Green
Place Medium Green
Pick Small Green
Place Small Green
Pick Tiny Green
Place Tiny Green
```

Then move to the next color.

## Important design decision

The planner should use task metadata, while the evaluator should inspect
the actual physical result.

This prevents the system from declaring success simply because its
internal plan was correct.

------------------------------------------------------------------------

# 9. Stage 5 --- Robot Controller

## Goal

Use Lucky's actual robot-control systems to perform manipulation.

Lucky's `RobotControllerComponent` is the primary control surface for
robot policies and motion-graph IK. Current documentation shows motion
graphs being used to drive IK targets while policies can control other
joints. citeturn0search2turn0search3

## Control abstraction

Create a wrapper such as:

``` text
RobotManipulator
    MoveTo(position, orientation)
    OpenGripper()
    CloseGripper()
    Attach/verify grasp
    Lift()
    MoveToStack()
    Release()
    ReturnHome()
```

The exact methods should be implemented around the robot's actual Lucky
components.

## Do not

Avoid directly teleporting the robot's transform to objects.

The purpose of the project is to demonstrate actual robotic
manipulation.

## IK

Use the robot's motion graph to:

-   move end effector toward object
-   descend
-   grasp
-   lift
-   move to stack
-   descend
-   release

The exact motion-graph input names are robot-specific, so inspect the
selected robot's `.hmograph` rather than assuming universal input names.
citeturn0search3

## Gripper

The gripper should be controlled through the robot's actual
actuator/control interface.

## Definition of done

The robot can reliably:

1.  approach an object
2.  reach it
3.  close the gripper
4.  lift it
5.  move it
6.  release it
7.  return to a safe position

------------------------------------------------------------------------

# 10. Stage 6 --- Task State Machine

## Goal

Make robot behavior explicit and debuggable.

Suggested states:

``` text
Idle
  ↓
SelectTarget
  ↓
MoveAboveObject
  ↓
MoveToObject
  ↓
Grasp
  ↓
VerifyGrasp
  ↓
Lift
  ↓
MoveToStack
  ↓
Place
  ↓
Release
  ↓
VerifyPlacement
  ↓
SelectNextTarget
```

Failure paths:

``` text
VerifyGrasp ──failure──► Recovery
VerifyPlacement ──failure──► Recovery
Recovery ──unrecoverable──► EpisodeFailure
```

## Why this matters

This is one of the best places to demonstrate engineering maturity.

Instead of:

``` csharp
if (timer > 3.2f)
    MoveRobot();
```

prefer explicit state transitions and conditions.

Example conceptual interface:

``` csharp
enum TaskState
{
    Idle,
    SelectTarget,
    MoveAboveObject,
    MoveToObject,
    Grasp,
    VerifyGrasp,
    Lift,
    MoveToStack,
    Place,
    VerifyPlacement,
    Recovery,
    Success,
    Failure
}
```

Each state should have:

-   entry behavior
-   update behavior
-   completion condition
-   timeout
-   failure handling

------------------------------------------------------------------------

# 11. Stage 7 --- Stack Placement Logic

## Goal

Make stacking physically meaningful.

For a stack:

``` text
Large
Medium
Small
```

the next placement position should be derived from the current stack
rather than hard-coded.

Conceptually:

``` text
stack base
    +
sum of previous object heights
    +
small placement tolerance
```

This makes the system work for variable object dimensions.

## Important physical considerations

Account for:

-   object height
-   object radius/footprint
-   gripper clearance
-   table height
-   stack center
-   collision geometry
-   settling time

The robot should release the object and allow physics to settle before
judging success.

------------------------------------------------------------------------

# 12. Stage 8 --- Success Evaluator

## Goal

Determine success by examining the actual scene.

Create a dedicated evaluator.

Suggested checks:

``` text
ColorCorrect
ObjectCountCorrect
SizeOrderCorrect
PositionWithinTolerance
SupportedByPreviousObject
StackStable
AllObjectsPlaced
```

Overall:

``` text
TaskSuccess =
    ColorCorrect
    AND ObjectCountCorrect
    AND SizeOrderCorrect
    AND PositionWithinTolerance
    AND SupportedByPreviousObject
    AND StackStable
    AND AllObjectsPlaced
```

## Critical rule

The evaluator must inspect physical state, not simply the planner's
expected state.

For example:

``` text
Planner:
    "I placed Green Small."

Evaluator:
    "Green Small is not actually in the Green stack."

Result:
    failure
```

This is what makes the project a simulation benchmark instead of a
scripted animation.

------------------------------------------------------------------------

# 13. Stage 9 --- Failure and Recovery

## Goal

Demonstrate that the system can handle imperfect simulation outcomes.

Potential failures:

-   grasp missed
-   object slipped
-   object knocked over
-   object placed outside tolerance
-   stack became unstable
-   object landed on the wrong stack
-   robot reached an unreachable pose
-   object collision prevented placement
-   timeout

## Initial recovery

Keep recovery simple.

Examples:

``` text
failed grasp
    → open gripper
    → move away
    → retry

failed placement
    → inspect object
    → retry placement

unstable stack
    → mark episode failure
```

Do not build an enormous recovery system unless the core project is
already stable.

------------------------------------------------------------------------

# 14. Stage 10 --- Camera / Sensor Layer

## Goal

Use at least one meaningful sensor to demonstrate awareness of the
robotics stack.

Lucky currently supports camera and LiDAR systems, with RGB/depth camera
data and recorded sensor outputs. citeturn0search11

## Recommended first sensor

Use an RGB camera positioned above/near the workspace.

Use it initially for:

-   visualization
-   recorded observations
-   optional object detection later

Do NOT make computer vision required for the first working version.

The deterministic planner can use task metadata.

## Stretch goal

Add a perception layer:

``` text
Camera
   ↓
Object Detection
   ↓
Color Classification
   ↓
Size Estimation
   ↓
Task Planner
```

This would create a much stronger robotics story, but it should be
considered an advanced milestone.

------------------------------------------------------------------------

# 15. Stage 11 --- Observer / Dataset Recording

## Goal

Record complete episodes and demonstrate that the simulation can produce
useful robotics data.

Lucky's Observer records robot state, actions, contact events, and
camera frames and writes structured datasets, including Parquet data and
video. citeturn0search1turn0search8

## Record

At minimum:

-   task seed
-   task configuration
-   episode success/failure
-   robot state
-   actions
-   object placements
-   contact events
-   camera frames if available

## Episode lifecycle

Conceptually:

``` text
StartRecording
    ↓
Generate episode
    ↓
Run task
    ↓
Evaluate
    ↓
EndCurrentEpisode(success)
    ↓
Reset
    ↓
Next episode
```

## Dataset goal

Do not just record a single demonstration.

Generate multiple episodes:

``` text
Episode 001 → Success
Episode 002 → Success
Episode 003 → Failed grasp
Episode 004 → Success
Episode 005 → Unstable stack
...
```

This gives the project experimental value.

------------------------------------------------------------------------

# 16. Stage 12 --- Deterministic Replay / Regression

## Goal

Prove that the benchmark can reproduce tasks.

For each episode save:

``` text
seed
task configuration
engine configuration
difficulty
```

Then:

``` text
Seed 48291
    ↓
Generate task
    ↓
Run episode
    ↓
Result: Success
```

Running the same seed again should reproduce the same task.

## Why this matters

This is especially valuable for an engine-focused interview because it
demonstrates:

-   reproducibility
-   debugging
-   regression testing
-   simulation determinism
-   controlled experimentation

------------------------------------------------------------------------

# 17. Stage 13 --- Experiment / Benchmark Mode

## Goal

Turn the demo into a small benchmark.

Add a mode that runs many generated tasks.

Example:

``` text
Episodes:       100
Colors:         2–4
Objects/color:  2–5
Sizes:          2–4
Randomization:  ON
```

Collect:

``` text
Success Rate
Average Episode Time
Average Pick Attempts
Average Placement Attempts
Grasp Failure Rate
Placement Failure Rate
Stack Failure Rate
Timeout Rate
```

Example output:

``` text
LUCKYSORT BENCHMARK
-------------------

Episodes:             100
Success Rate:          87%
Average Duration:      42.7 sec

Failures:
  Grasp:               5
  Placement:           4
  Stack Stability:     3
  Timeout:             1
```

This is a major upgrade over a single demonstration.

------------------------------------------------------------------------

# 18. Stage 14 --- Optional Python / gRPC Runner

## Goal

Use Lucky's external control interface to automate experiments.

Lucky exposes a gRPC server and Python SDK for programmatically driving
simulations. The current API includes simulation stepping and
robot-control surfaces. citeturn0search4

Potential Python runner:

``` text
experiment_runner.py

for seed in seeds:
    connect()
    reset()
    configure(seed)

    while not done:
        step()

    collect_metrics()

summarize_results()
```

Use this only after the in-engine version is stable.

## Why it is valuable

It demonstrates the distinction between:

``` text
Engine-side simulation
```

and

``` text
External experiment orchestration
```

This is directly relevant to robotics simulation and ML workflows.

------------------------------------------------------------------------

# 19. Stage 15 --- Difficulty Scaling

Implement explicit difficulty levels.

## Level 1 --- Basic

-   2 colors
-   2 sizes
-   fixed positions
-   no obstacles

## Level 2 --- Randomized

-   2--3 colors
-   2--3 sizes
-   random positions
-   random rotations

## Level 3 --- Variable Tasks

-   variable colors
-   variable object counts
-   variable sizes
-   deterministic seeds

## Level 4 --- Physics

-   realistic mass
-   realistic friction
-   unstable placements
-   stack settling

## Level 5 --- Adversarial

Optional:

-   obstacles
-   tighter workspace
-   clutter
-   partially occluded objects
-   difficult grasp orientations

The system should be able to increase difficulty without rewriting the
core planner.

------------------------------------------------------------------------

# 20. Stage 16 --- Debug / Presentation UI

## Goal

Make the engineering visible during the demonstration.

A small overlay should show:

``` text
LUCKYSORT

Episode: 17 / 100
Seed: 92831

Difficulty: 3

Current Color: GREEN
Current Object: SMALL

State:
    MoveToStack

Expected:
    GREEN → LARGE → MEDIUM → SMALL

Placed:
    2 / 3

--------------------------------
Task Status: RUNNING
```

After completion:

``` text
TASK COMPLETE

✓ Correct colors
✓ Correct object count
✓ Correct size order
✓ All objects supported
✓ Stack stable

Episode Success
```

## Optional debug visualization

Show:

-   object labels
-   target stack markers
-   current target
-   expected placement order
-   robot end-effector target
-   state-machine state
-   collision/contact indicators
-   success/failure reason

This is especially useful during an interview demo because it lets the
interviewer see the architecture rather than only watching a robot move.

------------------------------------------------------------------------

# 21. Lucky Engine Elements --- Reference Checklist

The implementation should explicitly investigate and use the following
Lucky systems.

  -----------------------------------------------------------------------------
  Lucky element              Role in project                           Priority
  -------------------------- --------------------- ----------------------------
  Entity / Component system  Scene and object                          REQUIRED
                             architecture          

  C# scripting               Task and controller                       REQUIRED
                             logic                 

  RobotControllerComponent   Robot control                             REQUIRED

  Motion Graph / IK          Arm manipulation                          REQUIRED

  Gripper control            Pick/place                                REQUIRED

  MuJoCo physics             Physical interaction                      REQUIRED

  Time Manager / runners     Fixed timestep and                        REQUIRED
                             update scheduling     

  Scene / Editor             Authoring and                             REQUIRED
                             debugging             

  Observer                   Episode recording             STRONGLY RECOMMENDED

  Camera                     Observation /                          RECOMMENDED
                             presentation          

  Randomization              Procedural benchmark                      REQUIRED

  gRPC / Python SDK          External experiment                       OPTIONAL
                             runner                

  Policies / ONNX            Learned controller                        OPTIONAL

  LiDAR                      Advanced perception                       OPTIONAL

  Lucky Hub                  Dataset/team workflow                     OPTIONAL
  -----------------------------------------------------------------------------

Lucky's current Time Manager uses fixed-rate runners on a shared
timeline, with the default Robot runner at 50 Hz and a separate Capture
runner available for different-rate work. This should be considered when
deciding where task logic, recording, and sensor processing execute.
citeturn0search14

------------------------------------------------------------------------

# 22. Time Manager / Simulation Timing Considerations

Do not treat the simulation like a normal variable-frame-rate game loop.

Lucky uses fixed-rate time runners. The current scripting documentation
describes fixed simulation steps and multiple runners sharing one
timeline. citeturn0search12turn0search14

## Design implication

Separate:

``` text
Decision frequency
Control frequency
Physics frequency
Recording frequency
Camera frequency
```

when appropriate.

For example:

``` text
Physics / Robot runner: 50 Hz
Camera / Capture:       30 Hz
High-level planner:     10–20 Hz
```

Exact rates should be selected after profiling and should remain
configurable.

## Important

Do not use arbitrary wall-clock delays such as:

``` csharp
Thread.Sleep(...)
```

inside simulation scripts.

Use simulation time/state conditions.

------------------------------------------------------------------------

# 23. Recommended Code Organization

Suggested structure:

``` text
LuckySort/
│
├── Assets/
│   ├── Scenes/
│   │   └── LuckySort.scene
│   │
│   ├── Robots/
│   │
│   ├── Objects/
│   │   ├── Large/
│   │   ├── Medium/
│   │   ├── Small/
│   │   └── Tiny/
│   │
│   └── Materials/
│
├── Scripts/
│   ├── Core/
│   │   ├── TaskDefinition.cs
│   │   ├── SortObjectData.cs
│   │   └── StackTarget.cs
│   │
│   ├── Generation/
│   │   ├── TaskGenerator.cs
│   │   └── WorkspaceRandomizer.cs
│   │
│   ├── Planning/
│   │   └── StackPlanner.cs
│   │
│   ├── Robot/
│   │   ├── RobotManipulator.cs
│   │   └── GripperController.cs
│   │
│   ├── StateMachine/
│   │   ├── TaskState.cs
│   │   └── TaskStateMachine.cs
│   │
│   ├── Evaluation/
│   │   └── StackEvaluator.cs
│   │
│   ├── Episodes/
│   │   ├── EpisodeManager.cs
│   │   └── EpisodeMetrics.cs
│   │
│   ├── Recording/
│   │   └── RecordingManager.cs
│   │
│   └── UI/
│       └── DebugOverlay.cs
│
└── Python/
    └── experiment_runner.py
```

The exact folder structure can be adjusted to match Lucky's generated
project conventions.

------------------------------------------------------------------------

# 24. Recommended Milestones

## Milestone 1 --- Robot Pick/Place

**Target:** Robot successfully picks up one object and places it.

Must have:

-   robot
-   MuJoCo object
-   IK
-   gripper
-   basic state machine

**Do not proceed until reliable.**

------------------------------------------------------------------------

## Milestone 2 --- One Stack

**Target:** Robot builds:

``` text
Large
Medium
Small
```

Must have:

-   stack target
-   physical placement
-   size ordering
-   evaluator

------------------------------------------------------------------------

## Milestone 3 --- Multiple Colors

**Target:**

``` text
RED:   Large → Medium → Small
BLUE:  Large → Small
GREEN: Large → Medium → Small
```

Robot selects correct target stack based on color.

------------------------------------------------------------------------

## Milestone 4 --- Procedural Tasks

Add:

-   random seed
-   variable colors
-   variable object counts
-   random positions
-   random rotations

Same seed must reproduce the same task.

------------------------------------------------------------------------

## Milestone 5 --- Robust Evaluation

Add:

-   color validation
-   count validation
-   order validation
-   position tolerance
-   support validation
-   stability validation
-   failure reasons

------------------------------------------------------------------------

## Milestone 6 --- Recording

Add Observer.

Record:

-   successful episodes
-   failed episodes
-   task metadata
-   robot state
-   camera observations if available

------------------------------------------------------------------------

## Milestone 7 --- Benchmark Mode

Run:

``` text
10 → 25 → 100 episodes
```

Report:

-   success rate
-   failure categories
-   average duration

------------------------------------------------------------------------

## Milestone 8 --- Polish

Add:

-   debug UI
-   state visualization
-   task summary
-   clean camera
-   reset/replay
-   deterministic seed display

At this point the project is already a strong interview demonstration.

------------------------------------------------------------------------

# 25. Stretch Goals --- Only After Core Stability

## Stretch A --- Vision

Replace metadata-based object identification with camera perception.

**Build plan: `docs/VISION_PLAN.md` (Milestone 9, planned 2026-10-12).** C# can capture camera RGBA through the
undocumented `CaptureBridge.TryCaptureCameraRgba` (see LUCKY_API_NOTES §11); no depth.

Pipeline:

``` text
Camera
 ↓
Image
 ↓
Object detection
 ↓
Color classification
 ↓
Size estimation
 ↓
Planner
```

## Stretch B --- Python Experiment Runner

Run hundreds of seeds automatically.

## Stretch C --- Learned Policy

Train/use a policy to perform some portion of manipulation.

Do not make this a prerequisite for the demo.

## Stretch D --- Replanning

If an object is knocked over:

``` text
Observe failure
    ↓
Update world state
    ↓
Re-plan
    ↓
Continue
```

## Stretch E --- Sim-to-Real-Oriented Randomization

Randomize:

-   friction
-   mass
-   object dimensions
-   sensor noise
-   camera position
-   lighting

The goal is to make the benchmark more representative of real-world
uncertainty.

------------------------------------------------------------------------

# 26. What NOT to Build

To keep the project in scope, avoid initially building:

-   a custom robotics physics engine
-   custom inverse kinematics solver
-   custom renderer
-   complex reinforcement-learning training pipeline
-   full computer-vision stack
-   sophisticated autonomous navigation
-   multi-robot coordination
-   huge UI framework
-   cloud deployment
-   real hardware integration

Lucky already provides the infrastructure for many of these areas.

The point of the project is to demonstrate that you can **use and extend
an engine intelligently**, not rebuild Lucky.

------------------------------------------------------------------------

# 27. Definition of the Final Demo

The final demo should ideally work like this:

``` text
1. Launch Lucky Engine.

2. Open LuckySort.

3. Show configuration:
      Colors: 3
      Size tiers: 3
      Objects/color: Variable
      Randomization: ON
      Seed: 48291

4. Generate Task.

5. Scene populates with randomized objects.

6. Debug panel shows:
      RED: Large → Medium → Small
      BLUE: Large → Small
      GREEN: Large → Medium → Small

7. Press Start.

8. Robot autonomously:
      finds target
      approaches
      grasps
      lifts
      moves
      places
      verifies
      repeats

9. Stacks physically settle.

10. Evaluator checks the result.

11. UI displays:
      ✓ Color
      ✓ Count
      ✓ Order
      ✓ Stability

12. Episode is recorded.

13. Generate another seed.

14. A completely different task appears.

15. Run benchmark mode.

16. Show aggregate statistics.
```

The strongest final message is:

> "This isn't a scripted stacking animation. It's a procedural
> manipulation benchmark. The task generator creates the problem, the
> planner determines the required action sequence, Lucky's robot-control
> and physics systems execute it, and an independent evaluator
> determines whether the physical result satisfies the task."

------------------------------------------------------------------------

# 28. Claude Execution Instructions

When giving this document to Claude, Claude should follow these rules.

## Rule 1 --- Inspect before implementing

Before writing substantial code:

-   inspect the Lucky project
-   inspect the installed/example robot
-   inspect the Piper Pattern Stacking example
-   inspect the selected robot's motion graph
-   inspect generated C# API/types
-   determine exact available methods for:
    -   robot control
    -   IK
    -   gripper
    -   physics state
    -   scene/entity creation
    -   recording
    -   reset

Do not invent Lucky APIs.

## Rule 2 --- Prefer official/current Lucky APIs

The Lucky documentation API is generated from the engine's C# source, so
use the actual installed version/API available to the project rather
than relying on guessed method names.

## Rule 3 --- Implement incrementally

Do not attempt the entire project in one pass.

Recommended sequence:

``` text
Robot movement
    ↓
Pick/place
    ↓
Single stack
    ↓
Multiple colors
    ↓
Task generation
    ↓
Evaluation
    ↓
Randomization
    ↓
Recording
    ↓
Benchmarking
    ↓
Polish
```

## Rule 4 --- Keep the architecture modular

Do not create one giant script.

Keep:

``` text
Generation
Planning
Robot Control
State Machine
Evaluation
Episodes
Recording
UI
```

separate.

## Rule 5 --- Test each milestone

After every milestone:

1.  Build.
2.  Run the scene.
3.  Verify behavior.
4.  Fix errors.
5.  Only then proceed.

## Rule 6 --- Do not over-engineer

If a simple implementation demonstrates the concept clearly, use it.

Only introduce additional abstraction when it improves:

-   reuse
-   testing
-   readability
-   debugging
-   extensibility

## Rule 7 --- Preserve determinism

Whenever randomization is added, use an explicit seed.

The same seed should generate the same task.

## Rule 8 --- Document discoveries

As Lucky APIs are explored, maintain a small project note documenting:

``` text
Lucky API / Component
Purpose
How it is used
Important constraints
```

This becomes useful both for debugging and for the interview
presentation.

------------------------------------------------------------------------

# 29. Priority Order

If time becomes limited, prioritize exactly in this order:

### Tier 1 --- Must Work

1.  Robot
2.  Physics
3.  Pick/place
4.  Single stack
5.  Multiple colors
6.  Size ordering
7.  Success evaluator

### Tier 2 --- Strong Interview Demo

8.  Procedural generation
9.  Seeded randomization
10. State machine
11. Failure handling
12. Debug UI
13. Observer recording

### Tier 3 --- Excellent

14. Benchmark mode
15. Aggregate statistics
16. Camera observations
17. Deterministic replay

### Tier 4 --- Wow Factor

18. Python/gRPC automation
19. Perception
20. Replanning
21. Learned policy
22. Advanced domain randomization

**Do not sacrifice Tier 1--2 stability to reach Tier 4.**

------------------------------------------------------------------------

# 30. Final Success Criteria

The project is complete when:

-   [ ] LuckySort opens and runs reliably.
-   [ ] Robot uses Lucky's robot-control system.
-   [ ] Robot uses IK/motion graph rather than transform teleportation.
-   [ ] Objects use actual MuJoCo physics.
-   [ ] Objects have meaningful physical sizes.
-   [ ] Multiple colors are supported.
-   [ ] Multiple size tiers are supported.
-   [ ] Object counts can vary.
-   [ ] Spawn positions can vary.
-   [ ] Tasks are generated from a seed.
-   [ ] Same seed reproduces the same task.
-   [ ] Planner produces correct size ordering.
-   [ ] Robot builds one stack per color.
-   [ ] Placement is physically simulated.
-   [ ] Independent evaluator checks success.
-   [ ] Failure reasons are reported.
-   [ ] State machine is visible/debuggable.
-   [ ] At least one complete episode can be recorded.
-   [ ] Multiple episodes can be generated.
-   [ ] Project can show meaningful benchmark statistics.
-   [ ] The code is separated into logical systems.
-   [ ] A new task configuration can be added without rewriting the
    robot controller.

------------------------------------------------------------------------

# 31. Interview Story

The project should ultimately be presented as an engine/robotics
engineering problem:

### Problem

> How do we create a reusable simulation benchmark for robotic
> manipulation rather than a single hard-coded demo?

### Solution

> I built a procedural color-and-size stacking environment. The task
> generator creates randomized object configurations, the planner
> converts those configurations into a valid manipulation sequence,
> Lucky's robot controller and IK perform the physical interaction,
> MuJoCo determines the actual physical outcome, and a separate
> evaluator verifies whether the resulting stacks satisfy the task
> constraints.

### Engineering challenges

Discuss:

-   separating task planning from robot control
-   fixed timestep behavior
-   physical grasping
-   IK target management
-   stack placement tolerances
-   deterministic randomization
-   failure recovery
-   simulation state vs task state
-   data recording
-   benchmark design

### Strongest engineering point

The project is valuable because the **task changes without the core
robot-control system changing**.

That demonstrates reusable systems thinking rather than one-off
scripting.

------------------------------------------------------------------------

# 32. Official Lucky Documentation References

Use the current Lucky Engine documentation as the source of truth for
exact APIs and behavior:

-   Lucky Engine documentation: `https://docs.luckyrobots.com/stable/`
-   Getting Started: `https://docs.luckyrobots.com/stable/get-started/`
-   Scripting: `https://docs.luckyrobots.com/stable/scripting/`
-   Robot control / IK:
    `https://docs.luckyrobots.com/2026.2/scripting/guides/controlling-robots/`
-   Robots: `https://docs.luckyrobots.com/2026.2/robots/`
-   Observer / recording:
    `https://docs.luckyrobots.com/stable/scripting/guides/recording/`
-   Sensors: `https://docs.luckyrobots.com/2026.2/sensors/`
-   Time runners:
    `https://docs.luckyrobots.com/2026.2/scripting/getting-started/time-runners/`
-   gRPC API: `https://docs.luckyrobots.com/2026.2/grpc-api/`
-   Example projects:
    `https://docs.luckyrobots.com/2026.2/example-projects/`

------------------------------------------------------------------------

# 33. First Task for Claude

Before implementing the benchmark, Claude should perform a **Lucky
Engine reconnaissance pass**.

Deliver a short report containing:

1.  Which robot is being used.
2.  Which motion graph it uses.
3.  How to set an IK target.
4.  How to open/close the gripper.
5.  How to create/duplicate/spawn objects at runtime.
6.  How to read an object's transform/physics state.
7.  How to detect contacts/collisions.
8.  How to reset the scene.
9.  How to seed/randomize task generation.
10. How to start/end Observer episodes.
11. How to access camera observations.
12. Which Time Manager runner should own each major subsystem.
13. Which APIs are confirmed from the installed Lucky version.

Only after this reconnaissance should implementation begin.

**Primary implementation objective:**

> Build a robust, deterministic, procedurally generated robotic
> color-and-size stacking benchmark using Lucky Engine's native ECS, C#
> scripting, robot-control/IK, MuJoCo physics, fixed-timestep
> simulation, and Observer recording systems.
