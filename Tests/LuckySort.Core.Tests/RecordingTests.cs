using System;
using System.IO;
using System.Linq;
using System.Text.Json;
using LuckySortStack.Core;
using LuckySortStack.Evaluation;
using LuckySortStack.Generation;
using LuckySortStack.Planning;
using LuckySortStack.Recording;
using LuckySortStack.StateMachine;
using Xunit;

namespace LuckySort.Core.Tests
{
	public sealed class RecordingTests : IDisposable
	{
		private readonly string m_Dir = Path.Combine(Path.GetTempPath(), "LuckySortRecTest_" + Guid.NewGuid().ToString("N"));

		public void Dispose()
		{
			if (Directory.Exists(m_Dir)) Directory.Delete(m_Dir, recursive: true);
		}

		private static TaskDefinition Task(int seed) =>
			TaskGenerator.Generate(new TaskConfig { Seed = seed, NumColors = 3, MaxObjectsPerColor = 3, NumSizeTiers = 3 }, WorkspaceSpec.Default);

		// Run one full episode (robot + evaluation) through the recorder, as EpisodeManager does.
		private static (EpisodeRecord Record, EpisodeResult Result, FakeSim Sim) RunRecorded(EpisodeRecorder recorder, TaskDefinition task,
			Action<FakeSim>? arrange = null, bool write = true)
		{
			var settings = new MotionSettings();
			var sim = new FakeSim(task, settings.Home);
			arrange?.Invoke(sim);
			var plan = StackPlanner.Plan(task);
			var machine = new TaskStateMachine(task, plan, sim, sim, settings);
			recorder.Begin(task, plan, machine);
			machine.Start();
			for (int i = 0; i < 200_000 && !machine.IsFinished; i++)
			{
				sim.Step(0.01f);
				machine.Update(0.01f);
			}

			var evaluator = new StackEvaluator(task, sim, new EvaluationSettings());
			while (!evaluator.IsComplete)
			{
				sim.Step(0.01f);
				evaluator.Update(0.01f);
			}
			var result = EpisodeResult.From(task, machine.Metrics, evaluator.Report!);
			if (write) recorder.Finish(result, sim);
			return (recorder.Build(result, sim), result, sim);
		}

		[Fact]
		public void SuccessfulEpisodeIsWrittenWithTaskTimelineAndVerdict()
		{
			var task = Task(1234);
			var recorder = new EpisodeRecorder(m_Dir, "s1");
			var (_, result, sim) = RunRecorded(recorder, task);

			string path = Path.Combine(m_Dir, "episode_0000.json");
			Assert.True(File.Exists(path));
			var record = JsonSerializer.Deserialize<EpisodeRecord>(File.ReadAllText(path))!;

			Assert.Equal(1234, record.Seed);
			Assert.Equal(task.Fingerprint(), record.Fingerprint);
			Assert.Equal(task.Objects.Count, record.Objects.Count);
			Assert.Equal(task.Objects.Count, record.Plan.Count);
			Assert.True(record.Result.Success);
			Assert.Equal("Success", record.Result.Category);
			Assert.All(record.Stacks, s => Assert.True(s.Checks.Values.All(v => v)));

			Assert.Equal("Idle -> SelectTarget", record.Timeline.First().Text);
			Assert.Equal("ReturnHome -> Success", record.Timeline.Last(e => e.Kind == "state").Text);
			Assert.True(record.Timeline.Zip(record.Timeline.Skip(1)).All(p => p.First.Time <= p.Second.Time), "timeline is in sim-time order");

			// Final poses are the physical ones at evaluation time.
			var obj = record.Objects[0];
			var body = sim.Bodies[obj.Tag];
			Assert.Equal(new[] { body.Position.X, body.Position.Y, body.Position.Z }, obj.FinalPosition);
		}

		[Fact]
		public void FailedEpisodeRecordsTheReasonAndTheEvents()
		{
			var task = Task(7);
			string tag = task.Targets[0].ExpectedObjects[0].PoolTag;
			var (record, _, _) = RunRecorded(new EpisodeRecorder(m_Dir, "s1"), task, sim => sim.GraspMisses[tag] = 99);

			Assert.False(record.Result.Success);
			Assert.Equal("GraspFailed", record.Result.Category);
			Assert.Equal("GraspFailed", record.Result.RobotOutcome);
			Assert.Contains("Incomplete", record.Result.EvalFailures);
			Assert.Contains(record.Timeline, e => e.Kind == "event" && e.Text.StartsWith("Grasp failed: " + tag));
		}

		[Fact]
		public void EpisodesInASessionAreNumberedAndIndexed()
		{
			var recorder = new EpisodeRecorder(m_Dir, "s1");
			RunRecorded(recorder, Task(1234));
			RunRecorded(recorder, Task(42));

			Assert.True(File.Exists(Path.Combine(m_Dir, "episode_0000.json")));
			Assert.True(File.Exists(Path.Combine(m_Dir, "episode_0001.json")));
			string[] lines = File.ReadAllLines(Path.Combine(m_Dir, "episodes.jsonl"));
			Assert.Equal(2, lines.Length);
			Assert.Contains("seed=1234", lines[0]);
			Assert.Contains("seed=42", lines[1]);
		}

		[Fact]
		public void SameSeedRecordsTheSameEpisode()
		{
			// Determinism end to end: same seed → same task, same timeline, same outcome (offline sim).
			var a = RunRecorded(new EpisodeRecorder(m_Dir, "a"), Task(1234), write: false).Record;
			var b = RunRecorded(new EpisodeRecorder(m_Dir, "b"), Task(1234), write: false).Record;
			a.Session = b.Session = "";
			Assert.Equal(JsonSerializer.Serialize(a), JsonSerializer.Serialize(b));
		}
	}
}
