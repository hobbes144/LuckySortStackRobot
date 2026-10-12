using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text.Encodings.Web;
using System.Text.Json;
using LuckySortStack.Benchmark;
using LuckySortStack.Core;
using LuckySortStack.Evaluation;
using LuckySortStack.Planning;
using LuckySortStack.Robot;
using LuckySortStack.StateMachine;

namespace LuckySortStack.Recording
{
	// Collects one episode's timeline from the state machine's events and writes the record when
	// the evaluation is in. Engine-free: time comes from the state machine's sim-time metrics and
	// poses from IWorldState. Writes go under one session directory:
	//   episode_0000.json   full record per episode
	//   episodes.jsonl      one EpisodeResult line per episode, for benchmark aggregation
	public sealed class EpisodeRecorder
	{
		// Relaxed escaping keeps "->" and "°" readable in the files (they are data files, never embedded in HTML).
		private static readonly JsonSerializerOptions k_Json = new JsonSerializerOptions { WriteIndented = true, Encoder = JavaScriptEncoder.UnsafeRelaxedJsonEscaping };
		private static readonly JsonSerializerOptions k_JsonLine = new JsonSerializerOptions { Encoder = JavaScriptEncoder.UnsafeRelaxedJsonEscaping };

		public string SessionDirectory { get; }
		public string Session { get; }
		public int EpisodeIndex { get; private set; } = -1;

		private TaskDefinition? m_Task;
		private TaskStateMachine? m_Machine;
		private List<PlannedStep> m_Plan = new List<PlannedStep>();
		private readonly List<TimelineEvent> m_Timeline = new List<TimelineEvent>();

		public EpisodeRecorder(string sessionDirectory, string session)
		{
			SessionDirectory = sessionDirectory;
			Session = session;
		}

		// Start a new episode and subscribe to the machine's events. Call before machine.Start().
		public void Begin(TaskDefinition task, IReadOnlyList<PlannedStep> plan, TaskStateMachine machine)
		{
			EpisodeIndex++;
			m_Task = task;
			m_Plan = plan.ToList();
			m_Machine = machine;
			m_Timeline.Clear();
			machine.StateChanged += (from, to) => Add("state", $"{from} -> {to}");
			machine.Log += message => Add("event", message);
		}

		// Build the record from the finished episode and write it. Returns the episode file path.
		public string Finish(EpisodeResult result, IWorldState world)
		{
			EpisodeRecord record = Build(result, world);
			Directory.CreateDirectory(SessionDirectory);
			string path = Path.Combine(SessionDirectory, $"episode_{EpisodeIndex:D4}.json");
			File.WriteAllText(path, JsonSerializer.Serialize(record, k_Json));
			File.AppendAllText(Path.Combine(SessionDirectory, "episodes.jsonl"),
				JsonSerializer.Serialize(new { session = Session, episode = EpisodeIndex, summary = result.ToString(), record.Result }, k_JsonLine) + "\n");
			return path;
		}

		// Benchmark aggregate for the session: benchmark_summary.json (data) + benchmark_summary.txt (readable).
		public string WriteSummary(BenchmarkSummary summary, IReadOnlyList<int> seeds, bool partial)
		{
			Directory.CreateDirectory(SessionDirectory);
			string path = Path.Combine(SessionDirectory, "benchmark_summary.json");
			File.WriteAllText(path, JsonSerializer.Serialize(new
			{
				session   = Session,
				partial,
				baseSeed  = seeds.Count > 0 ? seeds[0] : 0,
				planned   = seeds.Count,
				summary,
			}, k_Json));
			File.WriteAllText(Path.Combine(SessionDirectory, "benchmark_summary.txt"),
				$"Session {Session}{(partial ? " (PARTIAL)" : "")}, seeds {(seeds.Count > 0 ? seeds[0] : 0)}..{(seeds.Count > 0 ? seeds[^1] : 0)}\n" +
				summary.Describe());
			return path;
		}

		public EpisodeRecord Build(EpisodeResult result, IWorldState world)
		{
			TaskDefinition task = m_Task ?? throw new InvalidOperationException("Begin() was not called");
			EvaluationReport eval = result.Evaluation;
			EpisodeMetrics robot = result.Robot;

			return new EpisodeRecord
			{
				Session     = Session,
				Episode     = EpisodeIndex,
				Seed        = task.Seed,
				Fingerprint = task.Fingerprint(),
				Config = new ConfigRecord
				{
					NumColors          = task.Config.NumColors,
					MinObjectsPerColor = task.Config.MinObjectsPerColor,
					MaxObjectsPerColor = task.Config.MaxObjectsPerColor,
					NumSizeTiers       = task.Config.NumSizeTiers,
					RandomizePositions = task.Config.RandomizePositions,
					RandomizeRotations = task.Config.RandomizeRotations,
				},
				Objects = task.Objects.Select(o => new ObjectRecord
				{
					Tag             = o.PoolTag,
					Color           = o.Color.ToString(),
					Size            = o.Size.ToString(),
					Edge            = o.Edge,
					TargetStack     = task.Targets[o.TargetStack].Color.ToString(),
					TargetOrder     = o.TargetOrder,
					InitialPosition = Arr(o.InitialPosition),
					InitialYawDeg   = Deg(o.InitialYaw),
					FinalPosition   = Arr(world.GetObjectPosition(o.PoolTag)),
					FinalYawDeg     = Deg(world.GetObjectYaw(o.PoolTag)),
					FinalTiltDeg    = Deg(world.GetObjectTilt(o.PoolTag)),
				}).ToList(),
				Plan     = m_Plan.Select(s => s.ToString()).ToList(),
				Timeline = m_Timeline.ToList(),
				Result = new ResultRecord
				{
					Success            = result.Success,
					Category           = result.Category,
					Disagreement       = result.Disagreement,
					RobotOutcome       = robot.Succeeded ? "Success" : robot.Failure.ToString(),
					RobotFailureDetail = robot.FailureDetail,
					EvalFailures       = eval.Failures.Select(f => f.ToString()).ToList(),
					CorrectlyStacked   = eval.CorrectlyStacked,
					ExpectedObjects    = eval.ExpectedCount,
					PickAttempts       = robot.PickAttempts,
					PlaceAttempts      = robot.PlaceAttempts,
					GraspFailures      = robot.GraspFailures,
					PlacementFailures  = robot.PlacementFailures,
					Deferrals          = robot.Deferrals,
					BufferMoves        = robot.BufferMoves,
					Duration           = robot.Duration,
				},
				Stacks = eval.Stacks.Select(s => new StackRecord
				{
					Color    = s.Color.ToString(),
					Success  = s.Success,
					Expected = s.Expected.Select(o => o.PoolTag).ToList(),
					Actual   = s.Actual.Select(o => o.PoolTag).ToList(),
					Checks = new Dictionary<string, bool>
					{
						["ColorCorrect"]              = s.ColorCorrect,
						["ObjectCountCorrect"]        = s.ObjectCountCorrect,
						["SizeOrderCorrect"]          = s.SizeOrderCorrect,
						["PositionWithinTolerance"]   = s.PositionWithinTolerance,
						["SupportedByPreviousObject"] = s.SupportedByPreviousObject,
						["StackStable"]               = s.StackStable,
						["AllObjectsPlaced"]          = s.AllObjectsPlaced,
					},
					Issues = s.Issues.ToList(),
				}).ToList(),
			};
		}

		private void Add(string kind, string text)
		{
			if (m_Machine == null) return;
			m_Timeline.Add(new TimelineEvent { Time = m_Machine.Metrics.Duration, Step = m_Machine.StepIndex, Kind = kind, Text = text });
		}

		private static float[] Arr(System.Numerics.Vector3 v) => new[] { v.X, v.Y, v.Z };
		private static float Deg(float radians) => radians * 180f / MathF.PI;
	}
}
