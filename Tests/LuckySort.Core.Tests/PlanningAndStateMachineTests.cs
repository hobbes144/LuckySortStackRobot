using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using LuckySortStack.Core;
using LuckySortStack.Generation;
using LuckySortStack.Planning;
using LuckySortStack.StateMachine;
using Xunit;

namespace LuckySort.Core.Tests
{
	public class PlanningAndStateMachineTests
	{
		private static TaskDefinition Task(int seed, int colors = 3, int maxPerColor = 3, int tiers = 3) =>
			TaskGenerator.Generate(new TaskConfig { Seed = seed, NumColors = colors, MaxObjectsPerColor = maxPerColor, NumSizeTiers = tiers }, WorkspaceSpec.Default);

		private sealed class Run
		{
			public TaskDefinition Task = null!;
			public FakeSim Sim = null!;
			public TaskStateMachine Machine = null!;
			public List<string> Log = new();
		}

		private static Run Setup(TaskDefinition task, Action<FakeSim>? arrange = null, Action<MotionSettings>? configure = null)
		{
			var settings = new MotionSettings();
			configure?.Invoke(settings);
			var sim = new FakeSim(task, settings.Home);
			arrange?.Invoke(sim);
			var machine = new TaskStateMachine(task, StackPlanner.Plan(task), sim, sim, settings);
			var run = new Run { Task = task, Sim = sim, Machine = machine };
			machine.Log += run.Log.Add;
			return run;
		}

		private static Run Execute(Run run, float dt = 0.01f, int maxSteps = 200_000)
		{
			run.Machine.Start();
			for (int i = 0; i < maxSteps && !run.Machine.IsFinished; i++)
			{
				run.Sim.Step(dt);
				run.Machine.Update(dt);
			}
			Assert.True(run.Machine.IsFinished, "state machine did not finish: " + string.Join(" | ", run.Log.TakeLast(5)));
			return run;
		}

		// ── Planner ──────────────────────────────────────────────────────────

		[Fact]
		public void PlanBuildsEachStackBottomUpInStackOrder()
		{
			var task = Task(1234);
			var plan = StackPlanner.Plan(task);

			Assert.Equal(task.Objects.Count, plan.Count);
			Assert.Equal(task.Objects.Count, plan.Select(p => p.Object.PoolTag).Distinct().Count());

			int lastStack = -1;
			foreach (var group in plan.GroupBy(p => p.StackIndex))
			{
				Assert.True(group.Key > lastStack, "stacks must be built one after another");
				lastStack = group.Key;
				var steps = group.ToList();
				Assert.Equal(Enumerable.Range(0, steps.Count), steps.Select(s => s.Order));
				Assert.Equal(steps.Select(s => (int)s.Object.Size).OrderByDescending(s => s), steps.Select(s => (int)s.Object.Size));
				Assert.All(steps, s => Assert.Equal(s.Stack.Color, s.Object.Color));
			}
		}

		[Fact]
		public void PlacementHeightComesFromTheLiveStack()
		{
			var stack = new StackTarget { Color = ObjectColor.Red, BasePosition = new Vector3(0.4f, 0.011f, 0.18f) };

			Vector3 first = StackPlacement.PlaceCentre(stack, Array.Empty<(Vector3, float)>(), 0.06f, 0.003f);
			Assert.Equal(new Vector3(0.4f, 0.011f + 0.03f + 0.003f, 0.18f), first);

			// The top object drifted 4 mm in X: the next one follows it, stacked on its real top.
			var placed = new List<(Vector3, float)> { (new Vector3(0.404f, 0.041f, 0.18f), 0.06f) };
			Vector3 second = StackPlacement.PlaceCentre(stack, placed, 0.05f, 0.003f);
			Assert.Equal(0.404f, second.X, 5);
			Assert.Equal(0.041f + 0.03f + 0.025f + 0.003f, second.Y, 5);
		}

		[Theory]
		[InlineData(0.0f, 0.0f, 0.0f)]
		[InlineData(1.2f, 0.0f, 1.2f - MathF.PI / 2f)]
		[InlineData(-0.9f, 0.0f, -0.9f + MathF.PI / 2f)]
		[InlineData(0.3f, 1.5f, 0.3f + MathF.PI / 2f)]
		public void CubeYawPicksTheClosestEquivalentFace(float objectYaw, float currentYaw, float expected)
		{
			float yaw = TaskStateMachine.NearestCubeYaw(objectYaw, currentYaw);
			Assert.Equal(expected, yaw, 4);
			Assert.InRange(yaw - currentYaw, -MathF.PI / 4f - 1e-4f, MathF.PI / 4f + 1e-4f);
		}

		// ── State machine, against the fake sim ──────────────────────────────

		public static IEnumerable<object[]> Seeds => Enumerable.Range(0, 60).Select(s => new object[] { s * 104729 + 3 });

		[Theory]
		[MemberData(nameof(Seeds))]
		public void BuildsEveryStackLargestAtTheBottom(int seed)
		{
			var run = Execute(Setup(Task(seed, colors: 4, maxPerColor: 4, tiers: 4)));

			Assert.Equal(TaskState.Success, run.Machine.State);
			Assert.True(run.Machine.Metrics.Succeeded);
			Assert.Equal(run.Task.Objects.Count, run.Machine.Metrics.ObjectsPlaced);
			Assert.Equal(run.Task.Objects.Count, run.Machine.Metrics.PickAttempts);
			Assert.Equal(0, run.Sim.Collisions);

			foreach (var stack in run.Task.Targets)
			{
				var byHeight = stack.ExpectedObjects.OrderBy(o => run.Sim.Bodies[o.PoolTag].Position.Y).ToList();
				Assert.Equal(stack.ExpectedObjects.Select(o => o.PoolTag), byHeight.Select(o => o.PoolTag));
				foreach (var o in stack.ExpectedObjects)
				{
					var body = run.Sim.Bodies[o.PoolTag];
					Assert.InRange(MathF.Abs(body.Position.X - stack.BasePosition.X), 0f, 0.005f);
					Assert.InRange(MathF.Abs(body.Position.Z - stack.BasePosition.Z), 0f, 0.005f);
					// Faces squared to the stack (yaw 0 modulo 90°).
					Assert.InRange(MathF.Abs(TaskStateMachine.WrapQuarter(body.Yaw)), 0f, 0.2f);
				}
			}
		}

		[Theory]
		[InlineData(-1f, 0.35f)]
		[InlineData(-1f, -0.6f)]
		[InlineData(1f, 0.5f)]
		public void YawConventionIsLearnedFromMeasurement(float sign, float offset)
		{
			foreach (int seed in new[] { 12345, 1234, 1111, 42, 9 })
			{
				var run = Execute(Setup(Task(seed), sim => { sim.YawSign = sign; sim.YawOffset = offset; }));
				Assert.Equal(TaskState.Success, run.Machine.State);
				Assert.Equal(0, run.Sim.Collisions);
				Assert.Equal(sign, run.Machine.LearnedYawSign);
			}
		}

		[Fact]
		public void WithoutClosedLoopYawAWrongSignCollidesAndIsDetected()
		{
			var run = Execute(Setup(Task(12345), sim => sim.YawSign = -1f, s => s.ClosedLoopYaw = false));
			Assert.True(run.Sim.Collisions > 0);
			Assert.Contains(run.Log, l => l.StartsWith("Collision:"));
		}

		// 11 mm of tracking error is inside MoveTolerance but more than a Large cube's ~10 mm finger clearance.
		private static readonly Vector3 k_TrackingBias = new Vector3(0.009f, 0f, -0.006f);

		[Fact]
		public void TrackingErrorWithoutCentringPushesTheLargeCube()
		{
			var run = Execute(Setup(Task(12345), sim => sim.TrackingBias = k_TrackingBias, s => s.MaxCenterIterations = 0));
			Assert.True(run.Sim.Collisions > 0);
			Assert.Contains(run.Log, l => l.StartsWith("Collision:"));
		}

		[Fact]
		public void TrackingErrorIsCentredOutBeforeDescent()
		{
			foreach (int seed in new[] { 12345, 1234, 42 })
			{
				var run = Execute(Setup(Task(seed), sim => sim.TrackingBias = k_TrackingBias));
				Assert.Equal(TaskState.Success, run.Machine.State);
				Assert.Equal(0, run.Sim.Collisions);
				Assert.Contains(run.Log, l => l.StartsWith("Centring:"));
			}
		}

		[Fact]
		public void PushedObjectIsReapproachedFromItsNewPosition()
		{
			var task = Task(12345);
			string tag = task.Targets[0].ExpectedObjects[0].PoolTag;
			var run = Execute(Setup(task, sim => sim.PushOnce[tag] = new Vector3(0.015f, 0f, 0f)));

			Assert.Equal(TaskState.Success, run.Machine.State);
			Assert.Equal(1, run.Machine.Metrics.GraspFailures);
			Assert.Contains(run.Log, l => l.StartsWith("Collision: " + tag));
		}

		[Fact]
		public void SlippedGraspIsRetried()
		{
			var task = Task(7);
			string tag = task.Targets[0].ExpectedObjects[0].PoolTag;
			var run = Execute(Setup(task, sim => sim.GraspMisses[tag] = 1));

			Assert.Equal(TaskState.Success, run.Machine.State);
			Assert.Equal(1, run.Machine.Metrics.GraspFailures);
			Assert.Equal(task.Objects.Count + 1, run.Machine.Metrics.PickAttempts);
			Assert.Contains(run.Log, l => l.StartsWith("Grasp failed: " + tag));
		}

		[Fact]
		public void RepeatedGraspFailureEndsTheEpisode()
		{
			var task = Task(7);
			string tag = task.Targets[0].ExpectedObjects[0].PoolTag;
			var run = Execute(Setup(task, sim => sim.GraspMisses[tag] = 99));

			Assert.Equal(TaskState.Failure, run.Machine.State);
			Assert.Equal(FailureReason.GraspFailed, run.Machine.Metrics.Failure);
			Assert.Equal(new MotionSettings().MaxPickAttempts, run.Machine.Metrics.PickAttempts);
		}

		[Fact]
		public void MisplacedObjectIsPickedUpAgain()
		{
			var task = Task(11);
			string tag = task.Targets[0].ExpectedObjects[0].PoolTag;
			var run = Execute(Setup(task, sim => sim.DropOffsetOnce[tag] = new Vector3(0.03f, 0f, 0f)));

			Assert.Equal(TaskState.Success, run.Machine.State);
			Assert.Equal(1, run.Machine.Metrics.PlacementFailures);
			Assert.Equal(task.Objects.Count + 1, run.Machine.Metrics.PlaceAttempts);
		}

		[Fact]
		public void KnockedOverStackIsReportedAsCollapse()
		{
			var task = Task(5);
			var stack = task.Targets.First(t => t.ExpectedObjects.Count >= 2);
			string bottom = stack.ExpectedObjects[0].PoolTag;
			string second = stack.ExpectedObjects[1].PoolTag;
			var run = Execute(Setup(task, sim => sim.OnDropped = tag =>
			{
				if (tag == second) sim.Bodies[bottom].Position += new Vector3(0.05f, 0f, 0f);   // simulate a knock
			}));

			Assert.Equal(TaskState.Failure, run.Machine.State);
			Assert.Equal(FailureReason.StackCollapsed, run.Machine.Metrics.Failure);
		}

		[Fact]
		public void UnreachableObjectFailsInsteadOfHanging()
		{
			var task = Task(3);
			string tag = task.Targets[0].ExpectedObjects[0].PoolTag;
			var run = Execute(Setup(task, sim =>
			{
				sim.MaxReach = 0.6f;
				sim.Bodies[tag].Position = new Vector3(0.75f, sim.Bodies[tag].Position.Y, 0f);
			}));

			Assert.Equal(TaskState.Failure, run.Machine.State);
			Assert.Equal(FailureReason.Unreachable, run.Machine.Metrics.Failure);
			Assert.Equal(new MotionSettings().MaxPickAttempts, run.Machine.Metrics.PickAttempts);   // retried, then gave up
		}

		[Fact]
		public void StatesFollowThePickPlaceCycle()
		{
			var task = Task(1234, colors: 1, maxPerColor: 2, tiers: 2);
			var run = Setup(task);
			var states = new List<TaskState>();
			run.Machine.StateChanged += (_, to) => states.Add(to);
			Execute(run);

			var cycle = new[]
			{
				TaskState.SelectTarget, TaskState.MoveAboveObject, TaskState.AlignGripper, TaskState.CenterOverObject, TaskState.MoveToObject, TaskState.Grasp,
				TaskState.Lift, TaskState.VerifyGrasp, TaskState.MoveToStack, TaskState.AlignGripper, TaskState.Place,
				TaskState.Release, TaskState.Retreat, TaskState.VerifyPlacement,
			};
			var expected = Enumerable.Repeat(cycle, task.Objects.Count).SelectMany(c => c)
				.Concat(new[] { TaskState.SelectTarget, TaskState.ReturnHome, TaskState.Success });
			Assert.Equal(expected, states);
		}
	}
}
