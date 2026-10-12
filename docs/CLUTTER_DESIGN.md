# Cluttered starts: design and status (2026-10-12)

**Status: built (engine-free parts unit-tested in `ClutterTests`), awaiting `dotnet test` and an editor check.**
Decisions (Nick, 2026-10-12): inverted piles behind a separate flag, off by default; piles up to 3 high; clutter is a new
knob that leaves every existing seed unchanged when off. Set-aside location: first 3 dedicated pads, then **replaced (same
day) by free-space search**: the robot sets a cube down at the nearest open spot on the mat, so there are no pads to run out of.

What was built, against the plan below:
- **Generator:** `TaskConfig.PileCount` / `MaxPileHeight` / `AllowInvertedPiles`; `BuildPiles` runs after every existing draw
  and only when on; `SortObjectDefinition.InitialSupport`; mat extent in `WorkspaceSpec.Mat*` (read from LS_Workspace).
- **Planner:** instead of a separate online planner, the state machine resolves blockers when it selects each step
  (`ResolveBlocker`): it reads `SupportGraph.Topmost` from live positions and inserts one move in front of the step. That
  move goes to the blocker's own stack if it's next there (productive), otherwise to the nearest open spot on the mat:
  `Planning/FreeSpace` searches a 1 cm grid over the whole mat against every cube's *live* position (footprint + 30 mm
  clearance, inside reach, 110 mm from every stack base). Re-evaluated before every step, so 3-high piles unstack
  top-down. Worst case tested: 16 cubes, 4 piles 3 high, 10 seeds (`DenseWorstCaseClutterStillSorts`).
  Deferral (§6.5 of the notes) stays as is; a buffer move itself is never deferred.
- **Arm:** grasp-axis scoring ignores cubes entirely below the target (its support in a pile).
- **Scene:** inspector `PileCount` / `MaxPileHeight` / `AllowInvertedPiles` under Task; overlay shows `X Y -> buffer` and
  `buffered N`; markers outline the chosen spot in amber; set-aside moves have no Observer label; the episode record
  stores `Deferrals` and `BufferMoves`.
- **Not done yet:** the plan-efficiency metric (moves vs. lower bound), and the fingertip-vs-support risk in §4 needs
  checking in the editor (a Tiny cube picked off a Large one).


Difficulty level 5 in plan §19 ("clutter", "difficult grasp orientations"), combined with Stretch D (replanning).
Episodes start with some cubes **already piled on each other, mixed colours, wrong order**, so sorting becomes a
rearrangement problem (blocks world) instead of an ordering problem. Status: **design only, needs Nick's decisions (§6)
before building.**

## 1. What changes, and what doesn't

| Part | Today | With clutter |
|---|---|---|
| Generator | every cube on the mat, apart | some cubes piled 2–3 high at a shared spot |
| Planner | fixed order from task metadata | **online**: re-plans after every move from the observed scene |
| State machine | pick → place on stack | pick → place on stack **or** on a buffer spot |
| Evaluator | judges the final stacks | unchanged (a cube left in the buffer is `Incomplete`) |
| Reset / placement check | qpos writes at rest height | same writes, at pile heights |

Constraint carried over from rule 5 (determinism): **with clutter off, every existing seed must generate exactly the same
task** (`GoldenSeedIsUnchanged` keeps passing). All new random draws happen after the existing ones and only when
clutter is on.

## 2. Generator

New `TaskConfig` fields: `PileCount` (0 = off, the default), `MaxPileHeight` (2–3), `AllowInvertedPiles`.

1. Generate the task exactly as today (colours, sizes, stacks, positions, yaws).
2. If `PileCount > 0`: pick `PileCount` base cubes; for each, move 1..`MaxPileHeight−1` other random cubes onto it
   (any colour). Order within the pile is random, subject to the stability rule (§6, decision 1).
3. New `SortObjectDefinition.InitialSupport` (pool tag of the cube underneath, or null) and the pile's stacked
   `InitialPosition.Y`. The pile's footprint is its largest cube, so the existing spacing rule still applies.
4. Pile cubes share the base's yaw (± a few degrees), so they rest flat on each other.

## 3. Planner: online, from the observed scene

Today `StackPlanner.Plan` returns a fixed list from metadata. With clutter the scene has to be observed (rule 4: the
planner may use metadata for *what* is wanted, but must look at the scene for *where things are*).

- **World model** (engine-free, `Planning/SupportGraph`): from live positions, cube B is on A if their XZ distance is
  < A.edge/2 and B.y ≈ A.y + (A.edge + B.edge)/2. That gives "clear" (nothing on top) for every cube.
- **Next move** (greedy, the standard blocks-world heuristic, within 2× of optimal):
  1. If some stack's next expected cube is clear → move it to its stack (productive move).
  2. Else take the topmost cube on a pile that buries a needed cube. If it's the next expected for *its* stack,
     move it there; otherwise move it to the nearest free **buffer spot**.
- Re-plan after every verified placement, so a pile that shifted or a push is simply observed and handled.
- **Absorbs today's deferral:** a cube that runs out of attempts is marked unavailable for one round, and the
  re-planner picks the next best move automatically.
- **Metric for the demo:** moves used vs. the lower bound (objects + cubes that must be unstacked) = plan efficiency.

## 4. State machine and arm

- `PlannedStep` gains `Destination` = Stack(index) | Buffer(spot). Buffer placement reuses `Place`/`Release`/`VerifyPlacement`
  with the spot's surface height.
- **Picking from a pile:** `MoveToObject` already descends to the cube's *live* centre, so height is handled.
  **Risk:** the fingertips extend below the grasp site; on a Tiny cube (15 mm half-height) sitting on a Large one they
  may touch the cube underneath. Verify in editor; the fix would be a per-size grasp height (`GraspOffsetY` by edge).
- Grasp-axis choice (§6.2 of the notes) also scores the cube *underneath* for a pile pick.

## 5. Phases

1. **Generator + layout** (engine-free + editor check): piles generated, `FakeSim` settles cubes onto their supports,
   `ResetSpike` places piles that stay up. Gate: piles survive the 0.5 s settle on 10 seeds.
2. **SupportGraph + online planner** (engine-free, tests on FakeSim): "a Red Large under a Blue Tiny" cases, optimal-move
   lower bound, deferral folded in.
3. **Buffer moves in the state machine** + overlay (`Blue Tiny -> buffer`).
4. **Benchmark**: clutter levels (0, 1, 2 piles) on the same seeds; report success, moves/lower bound, failures.

## 6. Decisions for Nick

1. **Inverted piles?** Allow a larger cube on a smaller one (Large on Tiny)? It's the hardest and most "adversarial"
   case, but a 60 mm cube on a 30 mm one may topple at reset. Suggest: off at first, a separate flag.
2. **Buffer spots**: dedicated marked pads (clear, easy to show on screen), or any free space on the mat (more general,
   more planning)? Suggest: 3 dedicated pads near the robot, drawn by the overlay markers.
3. **Max pile height**: 2 or 3? Suggest 3 (it forces multi-step unstacking, the interesting case).
4. **Seeds**: confirm clutter is a new knob that leaves all existing seeds unchanged when off (recommended).
