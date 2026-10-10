using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using LuckySortStack.Benchmark;
using LuckySortStack.Core;
using LuckySortStack.Evaluation;
using LuckySortStack.Generation;
using LuckySortStack.Planning;
using LuckySortStack.StateMachine;
using Xunit;

namespace LuckySort.Core.Tests
{
	public class BenchmarkTests
	{
		// ── Seeds ────────────────────────────────────────────────────────────

		[Fact]
		public void SeedsAreConsecutiveFromTheBase()
		{
			Assert.Equal(new[] { 10, 11, 12 }, BenchmarkSeeds.Make(10, 3));
		}

		[Fact]
		public void NegativeOrOverflowingSeedsAreRejected()
		{
			Assert.Throws<ArgumentOutOfRangeException>(() => BenchmarkSeeds.Make(-5, 3));
			Assert.Throws<ArgumentOutOfRangeException>(() => BenchmarkSeeds.Make(int.MaxValue, 2));
			Assert.Throws<ArgumentOutOfRangeException>(() => BenchmarkSeeds.Make(0, 0));
		}

		// ── Statistics ───────────────────────────────────────────────────────

		[Theory]
		[InlineData(0, 10, 0.0, 0.2775)]
		[InlineData(10, 10, 0.7225, 1.0)]
		[InlineData(5, 10, 0.2366, 0.7634)]
		[InlineData(95, 100, 0.8883, 0.9785)]
		public void WilsonIntervalMatchesReferenceValues(int successes, int n, double low, double high)
		{
			var (l, h) = BenchmarkSummary.Wilson(successes, n);
			Assert.Equal(low, l, 3);
			Assert.Equal(high, h, 3);
		}

		[Fact]
		public void DurationStatsUseSampleStdDevAndTrueMedian()
		{
			var s = DurationStats.Of(new[] { 4f, 1f, 3f, 2f });
			Assert.Equal(2.5f, s.Mean, 4);
			Assert.Equal(2.5f, s.Median, 4);
			Assert.Equal(1.2910f, s.StdDev, 3);
			Assert.Equal(1f, s.Min);
			Assert.Equal(4f, s.Max);
			Assert.Equal(0, DurationStats.Of(Array.Empty<float>()).Count);
		}

		// ── Observer labels ──────────────────────────────────────────────────

		[Fact]
		public void EveryPlannedStepHasAPreRegisteredLabel()
		{
			var labels = StackPlanner.AllStepLabels();
			Assert.Equal(40, labels.Count);                 // 4 colours × (1 + 2 + 3 + 4) levels
			Assert.Equal(labels.Count, labels.Distinct().Count());
			var set = labels.ToHashSet();
			for (int seed = 0; seed < 300; seed++)
			{
				var task = TaskGenerator.Generate(new TaskConfig { Seed = seed, NumColors = 4, MaxObjectsPerColor = 4, NumSizeTiers = 4 }, WorkspaceSpec.Default);
				Assert.All(StackPlanner.Plan(task), step => Assert.Contains(step.ToString(), set));
			}
		}

		// ── Offline benchmark ────────────────────────────────────────────────

		private enum Fault { None, GraspAlwaysSlips, DroppedSteps }

		// Deterministic fault per seed so the expected summary is known exactly.
		private static Fault FaultFor(int seed) => (seed % 10) switch { 3 => Fault.GraspAlwaysSlips, 7 => Fault.DroppedSteps, _ => Fault.None };

		private static EpisodeResult RunEpisode(int seed)
		{
			var task = TaskGenerator.Generate(new TaskConfig { Seed = seed, NumColors = 3, MaxObjectsPerColor = 3, NumSizeTiers = 3 }, WorkspaceSpec.Default);
			var settings = new MotionSettings();
			var sim = new FakeSim(task, settings.Home);
			var plan = StackPlanner.Plan(task);
			switch (FaultFor(seed))
			{
				case Fault.GraspAlwaysSlips: sim.GraspMisses[plan[0].Object.PoolTag] = 99; break;
				case Fault.DroppedSteps:     plan.RemoveAt(plan.Count - 1); break;
			}

			var machine = new TaskStateMachine(task, plan, sim, sim, settings);
			machine.Start();
			for (int i = 0; i < 200_000 && !machine.IsFinished; i++) { sim.Step(0.01f); machine.Update(0.01f); }

			var evaluator = new StackEvaluator(task, sim, new EvaluationSettings());
			while (!evaluator.IsComplete) { sim.Step(0.01f); evaluator.Update(0.01f); }
			return EpisodeResult.From(task, machine.Metrics, evaluator.Report!);
		}

		[Fact]
		public void OfflineBenchmarkSummaryMatchesInjectedFaults()
		{
			List<int> seeds = BenchmarkSeeds.Make(100, 40);
			var results = seeds.Select(RunEpisode).ToList();
			var summary = BenchmarkSummary.From(results);

			int slips = seeds.Count(s => FaultFor(s) == Fault.GraspAlwaysSlips);
			int dropped = seeds.Count(s => FaultFor(s) == Fault.DroppedSteps);
			int clean = seeds.Count - slips - dropped;

			Assert.Equal(40, summary.Episodes);
			Assert.Equal(clean, summary.Successes);
			Assert.Equal((float)clean / 40, summary.SuccessRate, 5);
			Assert.True(summary.SuccessRateLow95 < summary.SuccessRate && summary.SuccessRate < summary.SuccessRateHigh95);

			Assert.Equal(clean, summary.Categories["Success"]);
			Assert.Equal(slips, summary.Categories["GraspFailed"]);          // robot gave up: its own reason
			Assert.Equal(dropped, summary.Categories["Eval.Incomplete"]);    // robot "succeeded"; evaluator disagrees
			Assert.Equal(dropped, summary.Disagreements);
			Assert.Equal("Success", summary.Categories.Keys.First());        // most frequent first

			Assert.Equal(seeds.Where(s => FaultFor(s) != Fault.None).OrderBy(s => s), summary.FailedSeeds.OrderBy(s => s));
			Assert.Equal(clean, summary.SuccessDuration.Count);
			Assert.Equal(1f, summary.PicksPerObject);                       // clean runs never retry in FakeSim
			Assert.InRange(summary.StackedFraction, 0.5f, 1f);
			Assert.Contains("Failed seeds", summary.Describe());
		}

		[Fact]
		public void OfflineBenchmarkIsReproducible()
		{
			var a = BenchmarkSummary.From(BenchmarkSeeds.Make(500, 10).Select(RunEpisode).ToList());
			var b = BenchmarkSummary.From(BenchmarkSeeds.Make(500, 10).Select(RunEpisode).ToList());
			Assert.Equal(a.Describe(), b.Describe());
		}
	}
}
