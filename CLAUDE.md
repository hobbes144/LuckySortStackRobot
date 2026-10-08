# LuckySort

Procedural robotic color-and-size stacking benchmark built on **Lucky Engine**
(C# scripting, ECS, MuJoCo physics, RobotControllerComponent + motion-graph IK,
Observer recording). Intended as an interview demo of engine/robotics systems design.

The full spec is in `docs/LuckySort_Idea_Plan.md`. Read the relevant section before
starting any stage.

## Current status

- [x] Stage 0a: Lucky reconnaissance pass (plan §33) → `docs/LUCKY_API_NOTES.md`
- [ ] Stage 0b: smoke test (`Panda.cs`) verified in editor; update VERIFY items in the notes
- [ ] Stage 1: workspace (table, MuJoCo object pool per color × size, stack markers)
- [ ] Milestone 1: single-object pick/place
- [ ] Milestones 2–8 (see plan §24)

Update this checklist as milestones complete.

## Rules (from plan §28)

1. **Inspect before implementing.** Never invent Lucky APIs. Confirm every method
   against the installed engine version or the docs at https://docs.luckyrobots.com/stable/.
   Record confirmed APIs in `docs/LUCKY_API_NOTES.md`.
2. **Incremental.** Robot movement → pick/place → single stack → multiple colors →
   generation → evaluation → randomization → recording → benchmark → polish.
   Don't move on until the current milestone is reliable.
3. **Modular.** Keep Generation, Planning, Robot Control, State Machine, Evaluation,
   Episodes, Recording and UI in separate scripts/folders (plan §23).
4. **Planner uses task metadata; evaluator inspects physical state.** Never let the
   evaluator trust the planner.
5. **Determinism.** All randomness flows from an explicit seed. Same seed → same task.
6. **Fixed timestep.** No `Thread.Sleep` or wall-clock delays; use sim time/state
   conditions and the Time Manager runners.
7. **No teleporting** the robot or objects during normal operation; use IK + gripper
   on real MuJoCo rigid bodies.
8. **Don't over-engineer.** Tier 1–2 stability before Tier 3–4 features (plan §29).

## Layout

- `docs/` — plan and API notes (read `LUCKY_API_NOTES.md` before touching engine code)
- `LuckySortStack/` — the Lucky Engine project (LuckyEngine 2026.1.2, Franka Panda)
  - `Assets/Scenes/Main.hscene` — start scene
  - `Assets/Scripts/Client/Source/` — C# scripts (namespace `LuckySortStack`)
  - `Assets/ContentVault/Robots/Panda/` — MJCF + motion graph
- `Python/` — optional gRPC experiment runner (late stage only)

## Build

- Engine install: `C:\Users\nick_\Downloads\LuckyEngine-2026.1.2-Windows-x64\LuckyEngine-2026.1-Windows-x64`
  (engine docs for agents: `Data/Resources/AgentDocs/`)
- Compile check: `dotnet build LuckySortStack/Assets/Scripts/Client/LuckySortStack.csproj`
  (output goes to `Assets/Scripts/Binaries`, which the editor hot-reloads)
- The `.csproj` lists files explicitly. When adding a `.cs` file, add a `<Compile Include>` entry,
  or regenerate with `Win-CreateScriptProjects.bat`.
- Play mode and scene authoring happen in the editor; the user presses Play.

## Key engine facts (details in docs/LUCKY_API_NOTES.md)

- World is Y-up; IK targets are world metres, orientation in degrees.
- Gripper = actuator `actuator8`, 255 open / 0 closed; set `.IK = false` first.
- One `SetInputTrigger` per motion step; each trigger cancels the in-flight move.
- Manipulables must be MuJoCo bodies. Runtime spawning is undocumented, so use a pre-authored object pool.
- `Hazel.Random` can't be seeded; use `System.Random(seed)`.
