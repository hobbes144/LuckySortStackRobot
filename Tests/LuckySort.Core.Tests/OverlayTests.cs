using System;
using System.Linq;
using LuckySortStack.Core;
using LuckySortStack.Evaluation;
using LuckySortStack.Generation;
using LuckySortStack.Planning;
using LuckySortStack.Presentation;
using LuckySortStack.StateMachine;
using Xunit;

namespace LuckySort.Core.Tests
{
	public class OverlayTests
	{
		private static (TaskDefinition Task, FakeSim Sim, TaskStateMachine Machine) Start(int seed, int colors = 2, int perColor = 3, int tiers = 3)
		{
			var task = TaskGenerator.Generate(new TaskConfig { Seed = seed, NumColors = colors, MaxObjectsPerColor = perColor, NumSizeTiers = tiers }, WorkspaceSpec.Default);
			var settings = new MotionSettings();
			var sim = new FakeSim(task, settings.Home);
			var machine = new TaskStateMachine(task, StackPlanner.Plan(task), sim, sim, settings);
			machine.Start();
			return (task, sim, machine);
		}

		private static void Step(FakeSim sim, TaskStateMachine machine, int steps)
		{
			for (int i = 0; i < steps && !machine.IsFinished; i++) { sim.Step(0.01f); machine.Update(0.01f); }
		}

		private static EvaluationReport Judge(TaskDefinition task, FakeSim sim)
		{
			var evaluator = new StackEvaluator(task, sim, new EvaluationSettings());
			for (int i = 0; i < 1000 && !evaluator.IsComplete; i++) { sim.Step(0.01f); evaluator.Update(0.01f); }
			return evaluator.Report!;
		}

		private static void AssertFits(OverlayText text)
		{
			foreach (string line in new[] { text.Title, text.Status }.Concat(text.Body.Split('\n')))
			{
				Assert.True(line.Length <= OverlayFormatter.MaxLineChars, $"line too long ({line.Length}): '{line}'");
				Assert.All(line, c => Assert.InRange(c, ' ', '~'));   // ASCII only: the engine font's coverage is unknown
			}
		}

		[Fact]
		public void BeforeTheRobotStartsThePhaseIsShown()
		{
			var text = OverlayFormatter.Build(new OverlayInput { EpisodeNumber = 1, EpisodeCount = 10, Phase = "Laying out cubes" });

			Assert.Equal("LUCKYSORT  ep 1/10", text.Title);
			Assert.Equal("Laying out cubes", text.Status);
			AssertFits(text);
		}

		[Fact]
		public void RunningEpisodeShowsSeedStateStepAndStackProgress()
		{
			var (task, sim, machine) = Start(1234);
			Step(sim, machine, 50);
			var text = OverlayFormatter.Build(new OverlayInput { EpisodeNumber = 3, EpisodeCount = 10, Task = task, Machine = machine,
				ElapsedSeconds = 0.5f, BenchmarkDone = 2, BenchmarkPassed = 1 });

			Assert.Equal("RUNNING", text.Status);
			Assert.Equal(OverlayTone.Running, text.Tone);
			Assert.Contains("seed 1234 " + task.Fingerprint().Substring(0, 8), text.Body);
			Assert.Contains("bench 1/2 passed", text.Body);
			Assert.Contains(machine.State.ToString(), text.Body);
			Assert.Contains($"step 1/{machine.Steps.Count}  try 1/3", text.Body);
			PlannedStep first = machine.Steps[0];
			Assert.Contains($"{first.Object.Size} {first.Object.Color} -> {first.Stack.Color.ToString().ToUpperInvariant()}", text.Body);
			Assert.Contains("* " + first.Stack.Color.ToString().ToUpperInvariant(), text.Body);
			Assert.Contains("[" + new string('-', first.Stack.ExpectedObjects.Count) + "]", text.Body);
			AssertFits(text);
		}

		[Fact]
		public void FinishedEpisodeShowsTheEvaluatorChecklist()
		{
			var (task, sim, machine) = Start(1234);
			Step(sim, machine, 200_000);
			Assert.Equal(TaskState.Success, machine.State);
			var text = OverlayFormatter.Build(new OverlayInput { EpisodeNumber = 1, Task = task, Machine = machine, Report = Judge(task, sim) });

			Assert.Equal("SUCCESS", text.Status);
			Assert.Equal(OverlayTone.Success, text.Tone);
			Assert.Contains("[x] colours", text.Body);
			Assert.Contains("[x] stable", text.Body);
			Assert.Contains($"{task.Objects.Count}/{task.Objects.Count} stacked", text.Body);
			Assert.DoesNotContain("* ", text.Body);   // no active stack once judged
			foreach (var target in task.Targets)
				Assert.Contains("[" + new string('#', target.ExpectedObjects.Count) + "]", text.Body);
			AssertFits(text);
		}

		[Fact]
		public void ReplayIsLabelledAndKeyHintsFit()
		{
			var (task, sim, machine) = Start(5604);
			Step(sim, machine, 100);
			var text = OverlayFormatter.Build(new OverlayInput { EpisodeNumber = 1, Task = task, Machine = machine, Replay = true, KeyHints = true });

			Assert.Equal("LUCKYSORT  replay", text.Title);
			Assert.EndsWith("C cam  R/F/P replay", text.Body);
			AssertFits(text);
		}

		[Fact]
		public void WorstCaseLinesStillFitWithoutWrapping()
		{
			// Longest inputs: 4 colours x 4 sizes, a 10-digit seed, 3-digit episode counts, every state along the way.
			var (task, sim, machine) = Start(int.MaxValue, colors: 4, perColor: 4, tiers: 4);
			var input = new OverlayInput { EpisodeNumber = 999, EpisodeCount = 999, Task = task, Machine = machine,
				ElapsedSeconds = 9999.9f, BenchmarkDone = 999, BenchmarkPassed = 999 };
			for (int i = 0; i < 2000 && !machine.IsFinished; i++)
			{
				Step(sim, machine, 50);
				AssertFits(OverlayFormatter.Build(input));
			}
			input.Evaluating = true;
			AssertFits(OverlayFormatter.Build(input));
			input.Report = Judge(task, sim);
			AssertFits(OverlayFormatter.Build(input));
			AssertFits(OverlayFormatter.Build(new OverlayInput { Phase = "LAYOUT FAILED (see log)" }));
		}
	}
}
