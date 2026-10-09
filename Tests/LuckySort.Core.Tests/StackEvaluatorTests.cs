using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using LuckySortStack.Core;
using LuckySortStack.Evaluation;
using LuckySortStack.Generation;
using LuckySortStack.Planning;
using LuckySortStack.Robot;
using LuckySortStack.StateMachine;
using Xunit;

namespace LuckySort.Core.Tests
{
	public class StackEvaluatorTests
	{
		private sealed class World : IWorldState
		{
			public readonly Dictionary<string, Vector3> Positions = new();
			public Vector3 GetObjectPosition(string tag) => Positions[tag];
			public float GetObjectYaw(string tag) => 0f;
		}

		private static TaskDefinition Task(int seed = 1234) =>
			TaskGenerator.Generate(new TaskConfig { Seed = seed, NumColors = 3, MaxObjectsPerColor = 3, NumSizeTiers = 3 }, WorkspaceSpec.Default);

		// Every stack built perfectly, straight from the definition.
		private static World Solved(TaskDefinition task)
		{
			var world = new World();
			foreach (StackTarget t in task.Targets)
			{
				float y = t.BasePosition.Y;
				foreach (SortObjectDefinition o in t.ExpectedObjects)
				{
					world.Positions[o.PoolTag] = new Vector3(t.BasePosition.X, y + o.Edge / 2f, t.BasePosition.Z);
					y += o.Edge;
				}
			}
			return world;
		}

		private static EvaluationReport Inspect(TaskDefinition task, World world) =>
			StackEvaluator.Inspect(task, world, new EvaluationSettings());

		private static (TaskDefinition Task, StackTarget Stack) TaskWithTallStack(int minHeight = 2)
		{
			for (int seed = 1; ; seed++)
			{
				var task = Task(seed);
				var stack = task.Targets.FirstOrDefault(t => t.ExpectedObjects.Count >= minHeight);
				if (stack != null) return (task, stack);
			}
		}

		[Fact]
		public void PerfectStacksPass()
		{
			var task = Task();
			var report = Inspect(task, Solved(task));
			Assert.True(report.Success, report.Describe());
			Assert.Equal(task.Objects.Count, report.CorrectlyStacked);
		}

		[Fact]
		public void UntouchedSceneFailsWithEverythingUnplaced()
		{
			var task = Task();
			var world = new World();
			foreach (var o in task.Objects) world.Positions[o.PoolTag] = o.InitialPosition;
			var report = Inspect(task, world);

			Assert.False(report.Success);
			Assert.False(report.AllObjectsPlaced);
			Assert.Equal(task.Objects.Count, report.Unplaced.Count);
			Assert.Equal(0, report.CorrectlyStacked);
		}

		[Fact]
		public void ObjectOnTheWrongStackFailsColour()
		{
			var task = Task();
			var world = Solved(task);
			StackTarget a = task.Targets[0], b = task.Targets[1];
			SortObjectDefinition top = a.ExpectedObjects.Last();
			Vector3 bTop = world.Positions[b.ExpectedObjects.Last().PoolTag];
			world.Positions[top.PoolTag] = bTop + new Vector3(0f, b.ExpectedObjects.Last().Edge / 2f + top.Edge / 2f, 0f);

			var report = Inspect(task, world);
			Assert.False(report.Success);
			Assert.False(report.Stacks[1].ColorCorrect);
			Assert.False(report.Stacks[0].AllObjectsPlaced);
			Assert.Contains(report.Stacks[1].Issues, i => i.Contains("wrong colour: " + top.PoolTag));
		}

		[Fact]
		public void SwappedSizesFailOrder()
		{
			var (task, stack) = TaskWithTallStack();
			var world = Solved(task);
			// Put the second object at the bottom and the bottom one on top of it; leave the rest unplaced.
			SortObjectDefinition lower = stack.ExpectedObjects[0], upper = stack.ExpectedObjects[1];
			Vector3 b = stack.BasePosition;
			world.Positions[upper.PoolTag] = new Vector3(b.X, b.Y + upper.Edge / 2f, b.Z);
			world.Positions[lower.PoolTag] = new Vector3(b.X, b.Y + upper.Edge + lower.Edge / 2f, b.Z);
			foreach (var o in stack.ExpectedObjects.Skip(2)) world.Positions[o.PoolTag] = o.InitialPosition;

			var report = Inspect(task, world);
			var s = report.Stacks[task.Targets.IndexOf(stack)];
			Assert.False(s.SizeOrderCorrect);
			Assert.False(report.Success);
		}

		[Fact]
		public void OffsetObjectFailsPosition()
		{
			var task = Task();
			var world = Solved(task);
			string tag = task.Targets[0].ExpectedObjects[0].PoolTag;
			world.Positions[tag] += new Vector3(0.02f, 0f, 0f);

			var report = Inspect(task, world);
			Assert.False(report.Stacks[0].PositionWithinTolerance);
			Assert.False(report.Success);
		}

		[Fact]
		public void FloatingObjectFailsSupport()
		{
			var (task, stack) = TaskWithTallStack();
			var world = Solved(task);
			string top = stack.ExpectedObjects.Last().PoolTag;
			world.Positions[top] += new Vector3(0f, 0.02f, 0f);

			var report = Inspect(task, world);
			var s = report.Stacks[task.Targets.IndexOf(stack)];
			Assert.False(s.SupportedByPreviousObject);
			Assert.Contains(s.Issues, i => i.Contains(top) && i.Contains("above"));
		}

		[Fact]
		public void MovingStackFailsStability()
		{
			var task = Task();
			var world = Solved(task);
			var evaluator = new StackEvaluator(task, world, new EvaluationSettings { StableWindow = 0.5f });
			string tag = task.Targets[0].ExpectedObjects.Last().PoolTag;

			for (int i = 0; i < 100 && !evaluator.IsComplete; i++)
			{
				evaluator.Update(0.01f);
				world.Positions[tag] += new Vector3(0.0001f, 0f, 0f);   // slow slide: 10 mm/s
			}

			Assert.True(evaluator.IsComplete);
			Assert.False(evaluator.Report!.Stacks[0].StackStable);
			Assert.False(evaluator.Report.Success);
		}

		[Fact]
		public void StillSceneIsJudgedAfterTheWindow()
		{
			var task = Task();
			var evaluator = new StackEvaluator(task, Solved(task), new EvaluationSettings { StableWindow = 0.5f });
			int steps = 0;
			while (!evaluator.IsComplete) { evaluator.Update(0.01f); steps++; }
			Assert.InRange(steps, 50, 52);
			Assert.True(evaluator.Report!.Success, evaluator.Report.Describe());
		}

		// End to end: the state machine's own verdict and the evaluator's agree on a clean run…
		[Theory]
		[InlineData(1234)]
		[InlineData(12345)]
		[InlineData(42)]
		public void FullRunPassesEvaluation(int seed)
		{
			var (machine, sim, task) = RunEpisode(seed);
			Assert.Equal(TaskState.Success, machine.State);
			var report = Evaluate(task, sim);
			Assert.True(report.Success, report.Describe());
		}

		// …but the evaluator doesn't take the state machine's word for it.
		[Fact]
		public void EvaluatorCatchesWhatTheStateMachineMissed()
		{
			var (machine, sim, task) = RunEpisode(1234);
			Assert.Equal(TaskState.Success, machine.State);

			// After the robot's last verification, the top cube of a stack slides off.
			string tag = task.Targets[0].ExpectedObjects.Last().PoolTag;
			sim.Bodies[tag].Position += new Vector3(0.05f, 0f, 0f);

			var report = Evaluate(task, sim);
			Assert.False(report.Success);
			Assert.True(report.CorrectlyStacked < task.Objects.Count);
		}

		private static (TaskStateMachine, FakeSim, TaskDefinition) RunEpisode(int seed)
		{
			var task = Task(seed);
			var settings = new MotionSettings();
			var sim = new FakeSim(task, settings.Home);
			var machine = new TaskStateMachine(task, StackPlanner.Plan(task), sim, sim, settings);
			machine.Start();
			for (int i = 0; i < 200_000 && !machine.IsFinished; i++)
			{
				sim.Step(0.01f);
				machine.Update(0.01f);
			}
			Assert.True(machine.IsFinished);
			return (machine, sim, task);
		}

		private static EvaluationReport Evaluate(TaskDefinition task, FakeSim sim)
		{
			var evaluator = new StackEvaluator(task, sim, new EvaluationSettings());
			while (!evaluator.IsComplete)
			{
				sim.Step(0.01f);
				evaluator.Update(0.01f);
			}
			return evaluator.Report!;
		}
	}
}
