using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using LuckySortStack.Core;
using LuckySortStack.Evaluation;
using LuckySortStack.Generation;
using LuckySortStack.Planning;
using LuckySortStack.StateMachine;
using Xunit;

namespace LuckySort.Core.Tests
{
	// Cluttered starts (docs/CLUTTER_DESIGN.md): piles from the generator, and unstacking in the state machine.
	public class ClutterTests
	{
		private static TaskConfig Config(int seed, int piles, int colors = 4, int tiers = 4, int maxHeight = 3, bool inverted = false) => new TaskConfig
		{
			Seed = seed, NumColors = colors, MinObjectsPerColor = 2, MaxObjectsPerColor = tiers, NumSizeTiers = tiers,
			PileCount = piles, MaxPileHeight = maxHeight, AllowInvertedPiles = inverted,
		};

		private static TaskDefinition Generate(TaskConfig c) => TaskGenerator.Generate(c, WorkspaceSpec.Default);

		private sealed class Run
		{
			public TaskDefinition Task = null!;
			public FakeSim Sim = null!;
			public TaskStateMachine Machine = null!;
			public List<string> Log = new();
		}

		private static Run Execute(TaskDefinition task, Action<MotionSettings>? configure = null)
		{
			var settings = new MotionSettings();
			configure?.Invoke(settings);
			var sim = new FakeSim(task, settings.Home);
			var machine = new TaskStateMachine(task, StackPlanner.Plan(task), sim, sim, settings);
			var run = new Run { Task = task, Sim = sim, Machine = machine };
			machine.Log += run.Log.Add;
			machine.Start();
			for (int i = 0; i < 400_000 && !machine.IsFinished; i++) { sim.Step(0.01f); machine.Update(0.01f); }
			Assert.True(machine.IsFinished, "state machine did not finish: " + string.Join(" | ", run.Log.TakeLast(5)));
			return run;
		}

		private static EvaluationReport Judge(Run run)
		{
			var evaluator = new StackEvaluator(run.Task, run.Sim, new EvaluationSettings());
			for (int i = 0; i < 1000 && !evaluator.IsComplete; i++) { run.Sim.Step(0.01f); evaluator.Update(0.01f); }
			return evaluator.Report!;
		}

		// ── Generator ───────────────────────────────────────────────────────

		[Fact]
		public void ClutterOffLeavesEveryTaskUnchanged()
		{
			for (int seed = 0; seed < 25; seed++)
			{
				var plain = Generate(Config(seed, piles: 0));
				var knobs = Generate(Config(seed, piles: 0, maxHeight: 2, inverted: true));   // other clutter knobs, but off
				Assert.Equal(plain.Fingerprint(), knobs.Fingerprint());
				Assert.All(plain.Objects, o => Assert.Null(o.InitialSupport));
			}
		}

		[Fact]
		public void PilesAreBuiltDeterministically()
		{
			for (int seed = 0; seed < 20; seed++)
			{
				var a = Generate(Config(seed, piles: 2));
				var b = Generate(Config(seed, piles: 2));
				Assert.Equal(a.Fingerprint(), b.Fingerprint());
				Assert.Equal(2, a.PileCount);
				Assert.NotEqual(Generate(Config(seed, piles: 0)).Fingerprint(), a.Fingerprint());
			}
		}

		[Fact]
		public void PileGeometryIsStableAndWithinLimits()
		{
			var ws = WorkspaceSpec.Default;
			for (int seed = 0; seed < 30; seed++)
			{
				var task = Generate(Config(seed, piles: 3, maxHeight: 3));
				var byTag = task.Objects.ToDictionary(o => o.PoolTag);
				foreach (var o in task.Objects.Where(o => o.InitialSupport != null))
				{
					var under = byTag[o.InitialSupport!];
					Assert.True(o.Edge <= under.Edge, $"seed {seed}: {o.PoolTag} is larger than {under.PoolTag} under it");
					Assert.Equal(under.InitialPosition.X, o.InitialPosition.X, 5);
					Assert.Equal(under.InitialPosition.Z, o.InitialPosition.Z, 5);
					Assert.Equal(under.InitialPosition.Y + (under.Edge + o.Edge) / 2f + ws.SpawnDropHeight, o.InitialPosition.Y, 5);
					Assert.Equal(under.InitialYaw, o.InitialYaw, 5);
				}
				// Height: no chain longer than MaxPileHeight.
				foreach (var o in task.Objects)
				{
					int height = 1;
					for (var c = o; c.InitialSupport != null; c = byTag[c.InitialSupport]) height++;
					Assert.InRange(height, 1, 3);
				}
				// Piles only ever start in the pick area, never on or near a stack base.
				foreach (var o in task.Objects)
					Assert.True(o.InitialPosition.Z < 0f, $"seed {seed}: {o.PoolTag} starts outside the pick area");
			}
		}

		[Fact]
		public void FakeSimSettlesPilesCubeByCube()
		{
			var task = Generate(Config(3, piles: 2));
			var sim = new FakeSim(task, new MotionSettings().Home);
			foreach (var o in task.Objects.Where(o => o.InitialSupport != null))
			{
				var under = task.Objects.First(u => u.PoolTag == o.InitialSupport);
				Assert.Equal(sim.Bodies[under.PoolTag].Position.Y + (under.Edge + o.Edge) / 2f, sim.Bodies[o.PoolTag].Position.Y, 4);
			}
		}

		// ── Unstacking ──────────────────────────────────────────────────────

		[Fact]
		public void ClutteredTasksAreSortedEndToEnd()
		{
			int buffered = 0;
			for (int seed = 0; seed < 12; seed++)
			{
				var run = Execute(Generate(Config(seed, piles: 2, colors: 3, tiers: 3)));
				Assert.True(run.Machine.State == TaskState.Success, $"seed {seed}: {run.Machine.Metrics}");
				Assert.True(Judge(run).Success, $"seed {seed}: evaluator disagrees");
				buffered += run.Machine.Metrics.BufferMoves;
			}
			Assert.True(buffered > 0, "no seed needed a buffer move; the test isn't exercising unstacking");
		}

		// Two stacks, two sizes. Put a cube from stack 1 on stack 0's first cube, so stack 0's first pick is blocked.
		private static TaskDefinition HandBuiltPile(int blockerOrder)
		{
			var task = Generate(new TaskConfig { Seed = 11, NumColors = 2, MinObjectsPerColor = 2, MaxObjectsPerColor = 2, NumSizeTiers = 2 });
			var bottom = task.Targets[0].ExpectedObjects[0];
			var blocker = task.Targets[1].ExpectedObjects[blockerOrder];
			blocker.InitialPosition = new Vector3(bottom.InitialPosition.X, bottom.InitialPosition.Y + (bottom.Edge + blocker.Edge) / 2f, bottom.InitialPosition.Z);
			blocker.InitialYaw = bottom.InitialYaw;
			blocker.InitialSupport = bottom.PoolTag;
			return task;
		}

		[Fact]
		public void BlockerThatIsNextForItsStackGoesStraightThere()
		{
			var run = Execute(HandBuiltPile(blockerOrder: 0));   // the blocker is stack 1's bottom cube

			Assert.Equal(TaskState.Success, run.Machine.State);
			Assert.Equal(0, run.Machine.Metrics.BufferMoves);
			Assert.Contains(run.Log, l => l.StartsWith("Unstacking:") && l.Contains("next for its stack"));
			Assert.True(Judge(run).Success);
		}

		[Fact]
		public void BlockerThatIsNotNextIsSetAsideAndComesBackLater()
		{
			var run = Execute(HandBuiltPile(blockerOrder: 1));   // stack 1's top cube: its bottom cube isn't placed yet

			Assert.Equal(TaskState.Success, run.Machine.State);
			Assert.Equal(1, run.Machine.Metrics.BufferMoves);
			Assert.Contains(run.Log, l => l.StartsWith("Unstacking:") && l.Contains("setting it aside at the nearest open spot"));
			Assert.Contains(run.Log, l => l.StartsWith("Set aside "));
			Assert.True(Judge(run).Success);
		}

		[Fact]
		public void DenseWorstCaseClutterStillSorts()
		{
			// Every pool cube in play (4 colours x 4 sizes), 4 piles up to 3 high: the most crowded table the generator makes.
			int setAside = 0;
			for (int seed = 0; seed < 10; seed++)
			{
				var task = Generate(new TaskConfig { Seed = seed, NumColors = 4, MinObjectsPerColor = 4, MaxObjectsPerColor = 4,
					NumSizeTiers = 4, PileCount = 4, MaxPileHeight = 3 });
				var run = Execute(task);
				Assert.True(run.Machine.State == TaskState.Success, $"seed {seed}: {run.Machine.Metrics} | {string.Join(" | ", run.Log.TakeLast(3))}");
				Assert.True(Judge(run).Success, $"seed {seed}: evaluator disagrees");
				setAside += run.Machine.Metrics.BufferMoves;
			}
			Assert.True(setAside > 0);
		}

		// ── FreeSpace: where a cube can be set aside ───────────────────────

		private static void AssertSpotIsValid(Vector2 spot, SortObjectDefinition cube, TaskDefinition task, FakeSim sim, float clearance)
		{
			var ws = WorkspaceSpec.Default;
			float own = FreeSpace.Footprint(cube.Edge);
			Assert.InRange(spot.X, ws.MatMinX + own, ws.MatMaxX - own);
			Assert.InRange(spot.Y, ws.MatMinZ + own, ws.MatMaxZ - own);
			Assert.InRange(spot.Length(), ws.MinReach, ws.MaxReach);
			foreach (var o in task.Objects.Where(o => o != cube))
			{
				Vector3 p = sim.Bodies[o.PoolTag].Position;
				float d = Vector2.Distance(spot, new Vector2(p.X, p.Z));
				Assert.True(d >= own + FreeSpace.Footprint(o.Edge) + clearance - 1e-4f, $"{cube.PoolTag} would touch {o.PoolTag} ({d * 1000f:F0} mm)");
			}
			foreach (var t in task.Targets)
				Assert.True(Vector2.Distance(spot, new Vector2(t.BasePosition.X, t.BasePosition.Z)) >= 0.11f - 1e-4f, "too close to a stack");
		}

		[Fact]
		public void OpenSpotsNeverOverlapCubesOrStacksAndStayInReach()
		{
			for (int seed = 0; seed < 10; seed++)
			{
				var task = Generate(new TaskConfig { Seed = seed, NumColors = 4, MinObjectsPerColor = 4, MaxObjectsPerColor = 4,
					NumSizeTiers = 4, PileCount = 4, MaxPileHeight = 3 });
				var sim = new FakeSim(task, new MotionSettings().Home);
				foreach (var cube in task.Objects)
				{
					Vector3 from = sim.Bodies[cube.PoolTag].Position;
					Vector2? spot = FreeSpace.Find(cube, task.Objects, sim, WorkspaceSpec.Default, task.Targets.Select(t => t.BasePosition),
						new Vector2(from.X, from.Z));
					Assert.True(spot.HasValue, $"seed {seed}: no open spot for {cube.PoolTag}");
					AssertSpotIsValid(spot!.Value, cube, task, sim, new FreeSpace.Settings().Clearance);
				}
			}
		}

		[Fact]
		public void OpenSpotIsTheNearestOneToTheCube()
		{
			var task = HandBuiltPile(blockerOrder: 1);
			var sim = new FakeSim(task, new MotionSettings().Home);
			var blocker = task.Targets[1].ExpectedObjects[1];
			Vector3 p = sim.Bodies[blocker.PoolTag].Position;
			var near = new Vector2(p.X, p.Z);
			Vector2 spot = FreeSpace.Find(blocker, task.Objects, sim, WorkspaceSpec.Default, task.Targets.Select(t => t.BasePosition), near)!.Value;

			// The closest legal position is right next to the pile: one footprint pair + clearance from the cube below.
			var below = task.Targets[0].ExpectedObjects[0];
			float minimum = FreeSpace.Footprint(blocker.Edge) + FreeSpace.Footprint(below.Edge) + new FreeSpace.Settings().Clearance;
			Assert.InRange(Vector2.Distance(spot, near), minimum - 0.001f, minimum + 2f * FreeSpace.GridStep);
		}

		[Fact]
		public void NoOpenSpaceMeansNoSpot()
		{
			var task = HandBuiltPile(blockerOrder: 1);
			var sim = new FakeSim(task, new MotionSettings().Home);
			var blocker = task.Targets[1].ExpectedObjects[1];
			var tiny = WorkspaceSpec.Default;   // a mat barely bigger than the pile under the blocker
			Vector3 b = sim.Bodies[task.Targets[0].ExpectedObjects[0].PoolTag].Position;
			tiny.MatMinX = b.X - 0.06f; tiny.MatMaxX = b.X + 0.06f; tiny.MatMinZ = b.Z - 0.06f; tiny.MatMaxZ = b.Z + 0.06f;

			Assert.Null(FreeSpace.Find(blocker, task.Objects, sim, tiny, task.Targets.Select(t => t.BasePosition), new Vector2(b.X, b.Z)));
		}

		[Fact]
		public void SupportGraphReadsPilesFromLivePositions()
		{
			var task = HandBuiltPile(blockerOrder: 1);
			var sim = new FakeSim(task, new MotionSettings().Home);
			var bottom = task.Targets[0].ExpectedObjects[0];
			var blocker = task.Targets[1].ExpectedObjects[1];

			Assert.Same(blocker, SupportGraph.Topmost(bottom, task.Objects, sim));
			Assert.Null(SupportGraph.Topmost(blocker, task.Objects, sim));
			sim.Bodies[blocker.PoolTag].Position += new Vector3(0.1f, 0f, 0f);   // knocked off
			Assert.Null(SupportGraph.Topmost(bottom, task.Objects, sim));
		}
	}
}
