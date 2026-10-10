using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Text;
using LuckySortStack.Evaluation;

namespace LuckySortStack.Benchmark
{
	// Which seeds a benchmark runs. Consecutive non-negative seeds from a base: reproducible from two
	// numbers, and no two episodes can share a task. (System.Random(seed) takes |seed|, so −n and n
	// would generate the same task under different names.)
	public static class BenchmarkSeeds
	{
		public static List<int> Make(int baseSeed, int count)
		{
			if (baseSeed < 0) throw new ArgumentOutOfRangeException(nameof(baseSeed), "benchmark seeds must be >= 0");
			if (count < 1) throw new ArgumentOutOfRangeException(nameof(count), "must run at least one episode");
			if ((long)baseSeed + count - 1 > int.MaxValue) throw new ArgumentOutOfRangeException(nameof(count), "seed range overflows int");
			return Enumerable.Range(baseSeed, count).ToList();
		}
	}

	public sealed class DurationStats
	{
		public int Count { get; set; }
		public float Mean { get; set; }
		public float StdDev { get; set; }
		public float Min { get; set; }
		public float Median { get; set; }
		public float Max { get; set; }

		public static DurationStats Of(IEnumerable<float> values)
		{
			var v = values.OrderBy(x => x).ToList();
			if (v.Count == 0) return new DurationStats();
			float mean = v.Average();
			float var = v.Count > 1 ? v.Sum(x => (x - mean) * (x - mean)) / (v.Count - 1) : 0f;   // sample variance
			float median = v.Count % 2 == 1 ? v[v.Count / 2] : 0.5f * (v[v.Count / 2 - 1] + v[v.Count / 2]);
			return new DurationStats { Count = v.Count, Mean = mean, StdDev = MathF.Sqrt(var), Min = v[0], Median = median, Max = v[^1] };
		}

		public override string ToString() => Count == 0 ? "n/a" : string.Format(CultureInfo.InvariantCulture,
			"mean {0:F1}s ± {1:F1}  (min {2:F1}, median {3:F1}, max {4:F1}, n={5})", Mean, StdDev, Min, Median, Max, Count);
	}

	// Aggregate of a benchmark run (plan §17 / Milestone 7): success rate with a confidence interval,
	// failure categories, durations, efficiency and partial credit. Pure function of the episode results.
	public sealed class BenchmarkSummary
	{
		public int Episodes { get; set; }
		public int Successes { get; set; }
		public float SuccessRate { get; set; }
		public float SuccessRateLow95 { get; set; }    // Wilson score interval
		public float SuccessRateHigh95 { get; set; }
		public int Disagreements { get; set; }         // robot and evaluator verdicts differed
		public Dictionary<string, int> Categories { get; set; } = new Dictionary<string, int>();   // most frequent first
		public DurationStats Duration { get; set; } = new DurationStats();            // all episodes
		public DurationStats SuccessDuration { get; set; } = new DurationStats();     // successful episodes only
		public float PicksPerObject { get; set; }      // pick attempts per object in successful episodes; 1.0 = never retried
		public float StackedFraction { get; set; }     // correctly stacked / expected, over all episodes
		public List<int> FailedSeeds { get; set; } = new List<int>();

		public static BenchmarkSummary From(IReadOnlyList<EpisodeResult> results)
		{
			var s = new BenchmarkSummary { Episodes = results.Count };
			if (results.Count == 0) return s;

			s.Successes = results.Count(r => r.Success);
			s.SuccessRate = (float)s.Successes / results.Count;
			(s.SuccessRateLow95, s.SuccessRateHigh95) = Wilson(s.Successes, results.Count);
			s.Disagreements = results.Count(r => r.Disagreement);
			s.Categories = results.GroupBy(r => r.Category)
				.OrderByDescending(g => g.Count()).ThenBy(g => g.Key, StringComparer.Ordinal)
				.ToDictionary(g => g.Key, g => g.Count());
			s.Duration = DurationStats.Of(results.Select(r => r.Robot.Duration));
			s.SuccessDuration = DurationStats.Of(results.Where(r => r.Success).Select(r => r.Robot.Duration));

			int objects = results.Sum(r => r.Evaluation.ExpectedCount);
			var won = results.Where(r => r.Success).ToList();
			int wonObjects = won.Sum(r => r.Evaluation.ExpectedCount);
			s.PicksPerObject = wonObjects > 0 ? (float)won.Sum(r => r.Robot.PickAttempts) / wonObjects : 0f;
			s.StackedFraction = objects > 0 ? (float)results.Sum(r => r.Evaluation.CorrectlyStacked) / objects : 0f;
			s.FailedSeeds = results.Where(r => !r.Success).Select(r => r.Seed).ToList();
			return s;
		}

		// 95% Wilson score interval: honest at small n and at 0% / 100%, unlike the normal approximation.
		public static (float Low, float High) Wilson(int successes, int n, double z = 1.959964)
		{
			if (n == 0) return (0f, 1f);
			double p = (double)successes / n;
			double z2 = z * z;
			double centre = (p + z2 / (2 * n)) / (1 + z2 / n);
			double half = z * Math.Sqrt(p * (1 - p) / n + z2 / (4.0 * n * n)) / (1 + z2 / n);
			return ((float)Math.Max(0, centre - half), (float)Math.Min(1, centre + half));
		}

		public string Describe()
		{
			var ci = CultureInfo.InvariantCulture;
			var sb = new StringBuilder();
			sb.Append(string.Format(ci, "Success rate {0:P0} ({1}/{2}), 95% CI {3:P0}–{4:P0}\n",
				SuccessRate, Successes, Episodes, SuccessRateLow95, SuccessRateHigh95));
			sb.Append("Categories: ").Append(string.Join(", ", Categories.Select(kv => $"{kv.Key} {kv.Value}"))).Append('\n');
			sb.Append("Duration (all):     ").Append(Duration).Append('\n');
			sb.Append("Duration (success): ").Append(SuccessDuration).Append('\n');
			sb.Append(string.Format(ci, "Picks per object (successful episodes) {0:F2}, objects correctly stacked {1:P1}, robot/evaluator disagreements {2}\n",
				PicksPerObject, StackedFraction, Disagreements));
			if (FailedSeeds.Count > 0)
				sb.Append("Failed seeds (replay with Seed = …): ").Append(string.Join(", ", FailedSeeds)).Append('\n');
			return sb.ToString();
		}
	}
}
