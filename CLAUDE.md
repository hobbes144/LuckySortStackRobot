# LuckySort

Procedural robotic color-and-size stacking benchmark built on **Lucky Engine**
(C# scripting, ECS, MuJoCo physics, RobotControllerComponent + motion-graph IK,
Observer recording). Intended as an interview demo of engine/robotics systems design.

The full spec is in `docs/LuckySort_Idea_Plan.md`. Read the relevant section before
starting any stage.

## Current status

- [ ] Stage 0: Lucky reconnaissance pass (plan §33). Write findings to `docs/LUCKY_API_NOTES.md`.
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

- `docs/` — plan and API notes
- Lucky project files (scenes, assets, `Scripts/`) will be created by the Lucky Engine
  editor in this folder; follow its generated conventions.
- `Python/` — optional gRPC experiment runner (late stage only)
