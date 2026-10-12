using System.Globalization;
using System.Linq;
using System.Text;
using LuckySortStack.Core;
using LuckySortStack.Evaluation;
using LuckySortStack.Planning;
using LuckySortStack.StateMachine;

namespace LuckySortStack.Presentation
{
	public enum OverlayTone { Neutral, Running, Success, Failure }

	// Everything the overlay shows, gathered by the scene side. Engine-free so the text is unit-tested.
	public sealed class OverlayInput
	{
		public int EpisodeNumber;               // 1-based
		public int EpisodeCount = 1;
		public TaskDefinition? Task;
		public TaskStateMachine? Machine;
		public EvaluationReport? Report;        // set once the evaluator has a verdict
		public bool Evaluating;                 // robot finished, evaluator watching for stability
		public float ElapsedSeconds;            // sim time of this episode's task
		public int BenchmarkDone;               // finished episodes so far in this session
		public int BenchmarkPassed;
		public string Phase = "";               // set before the robot starts, e.g. "Laying out cubes"
	}

	public sealed class OverlayText
	{
		public string Title = "";
		public string Status = "";
		public OverlayTone Tone;
		public string Body = "";
	}

	// Compact top-left readout (plan §20). Two engine constraints shape it:
	//  - TextComponent wraps long lines at an undocumented width (~28 characters in the 2026-10-12 screenshot) and C#
	//    can't change it, so every line is kept to MaxLineChars. OverlayTests enforces this.
	//  - The font's glyph coverage is unknown, so it's ASCII only.
	public static class OverlayFormatter
	{
		public const int MaxLineChars = 24;
		private static readonly CultureInfo Ci = CultureInfo.InvariantCulture;

		public static OverlayText Build(OverlayInput input)
		{
			TaskDefinition? task = input.Task;
			TaskStateMachine? machine = input.Machine;
			var text = new OverlayText
			{
				Title = string.Format(Ci, "LUCKYSORT  ep {0}/{1}", input.EpisodeNumber, input.EpisodeCount),
			};

			var sb = new StringBuilder();
			if (task != null) sb.Append(string.Format(Ci, "seed {0} {1}\n", task.Seed, task.Fingerprint().Substring(0, 8)));
			if (input.BenchmarkDone > 0) sb.Append(string.Format(Ci, "bench {0}/{1} passed\n", input.BenchmarkPassed, input.BenchmarkDone));

			if (input.Report != null)
			{
				text.Status = input.Report.Success ? "SUCCESS" : "FAIL: " + input.Report.PrimaryFailure;
				text.Tone = input.Report.Success ? OverlayTone.Success : OverlayTone.Failure;
				sb.Append('\n');
				AppendVerdict(sb, input.Report);
			}
			else if (machine != null)
			{
				text.Status = input.Evaluating ? "EVALUATING" : "RUNNING";
				text.Tone = input.Evaluating ? OverlayTone.Neutral : OverlayTone.Running;
				sb.Append('\n');
				AppendRun(sb, machine, input);
			}
			else
			{
				text.Status = input.Phase.Length > 0 ? input.Phase : "WAITING";
				text.Tone = OverlayTone.Neutral;
			}

			if (task != null)
			{
				sb.Append('\n');
				AppendStacks(sb, task, machine, input.Report);
			}
			text.Body = sb.ToString().TrimEnd('\n');
			return text;
		}

		private static void AppendRun(StringBuilder sb, TaskStateMachine m, OverlayInput input)
		{
			if (input.Evaluating)
			{
				sb.Append("robot: ").Append(m.State == TaskState.Success ? "Success" : m.Metrics.Failure.ToString()).Append('\n');
				sb.Append(string.Format(Ci, "t {0:F1}s\n", input.ElapsedSeconds));
				return;
			}
			sb.Append(m.State).Append('\n');
			PlannedStep? step = m.CurrentStep;
			if (step != null)
			{
				sb.Append(string.Format(Ci, "step {0}/{1}  try {2}/{3}\n", m.StepIndex + 1, m.Steps.Count, m.CurrentPickAttempt, m.MaxPickAttempts));
				sb.Append(step.Object.Size).Append(' ').Append(step.Object.Color).Append(" -> ")
				  .Append(step.Stack.Color.ToString().ToUpperInvariant()).Append('\n');
			}
			sb.Append(string.Format(Ci, "t {0:F1}s\n", input.ElapsedSeconds));
		}

		private static void AppendVerdict(StringBuilder sb, EvaluationReport r)
		{
			Check(sb, r.ColorCorrect, "colours");
			Check(sb, r.ObjectCountCorrect, "count");
			Check(sb, r.SizeOrderCorrect, "size order");
			Check(sb, r.PositionWithinTolerance, "position");
			Check(sb, r.SupportedByPreviousObject, "support");
			Check(sb, r.StackStable, "stable");
			sb.Append(string.Format(Ci, "{0}/{1} stacked\n", r.CorrectlyStacked, r.ExpectedCount));
		}

		private static void Check(StringBuilder sb, bool ok, string label) =>
			sb.Append(ok ? "[x] " : "[ ] ").Append(label).Append('\n');

		// "* BLUE   [#-] L S": the stack being built is starred; the letters are the expected sizes, bottom first.
		private static void AppendStacks(StringBuilder sb, TaskDefinition task, TaskStateMachine? m, EvaluationReport? report)
		{
			for (int s = 0; s < task.Targets.Count; s++)
			{
				StackTarget target = task.Targets[s];
				int expected = target.ExpectedObjects.Count;
				int placed = report != null
					? report.Stacks.FirstOrDefault(r => r.Color == target.Color)?.Actual.Count ?? 0
					: m == null ? 0 : m.Steps.Take(m.StepIndex).Count(st => st.StackIndex == s);
				bool active = report == null && m != null && !m.IsFinished && m.CurrentStep != null && m.CurrentStep.StackIndex == s;

				sb.Append(active ? "* " : "  ");
				sb.Append(target.Color.ToString().ToUpperInvariant().PadRight(6)).Append(' ');
				sb.Append('[').Append(new string('#', System.Math.Min(placed, expected))).Append(new string('-', System.Math.Max(0, expected - placed))).Append("] ");
				sb.Append(string.Join(" ", target.ExpectedObjects.Select(o => o.Size.ToString().Substring(0, 1))));
				sb.Append('\n');
			}
		}
	}
}
