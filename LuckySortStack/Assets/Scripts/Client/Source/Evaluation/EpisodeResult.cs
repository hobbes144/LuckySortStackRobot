using System.Globalization;
using LuckySortStack.Core;

namespace LuckySortStack.Evaluation
{
	// One finished episode: what the robot believed (EpisodeMetrics, from the state machine) and
	// what the evaluator found. The evaluator decides Success; the robot's own failure reason, when
	// it gave up, is the more specific explanation, so it takes precedence for the category.
	// This is the record benchmark mode aggregates and recording stores.
	public sealed class EpisodeResult
	{
		public int Seed;
		public string Fingerprint = "";
		public EpisodeMetrics Robot = new EpisodeMetrics();
		public EvaluationReport Evaluation = new EvaluationReport();

		public bool Success => Evaluation.Success;

		// The state machine and the evaluator reached different verdicts.
		public bool Disagreement => Robot.Succeeded != Evaluation.Success;

		// Failure category for aggregation: "Success", the robot's FailureReason when it gave up,
		// or "Eval.<EvalFailure>" when the robot thought it succeeded but the scene says otherwise.
		public string Category =>
			Success ? "Success"
			: !Robot.Succeeded ? Robot.Failure.ToString()
			: "Eval." + Evaluation.PrimaryFailure;

		public static EpisodeResult From(TaskDefinition task, EpisodeMetrics robot, EvaluationReport evaluation) => new EpisodeResult
		{
			Seed        = task.Seed,
			Fingerprint = task.Fingerprint(),
			Robot       = robot,
			Evaluation  = evaluation,
		};

		// Single line, key=value, so logs can be grepped and parsed by the benchmark runner.
		public override string ToString() => string.Format(CultureInfo.InvariantCulture,
			"seed={0} task={1} result={2} category={3} stacked={4}/{5} robot={6} picks={7} places={8} duration={9:F1}s",
			Seed, Fingerprint, Success ? "PASS" : "FAIL", Category, Evaluation.CorrectlyStacked, Evaluation.ExpectedCount,
			Robot.Succeeded ? "Success" : Robot.Failure.ToString(), Robot.PickAttempts, Robot.PlaceAttempts, Robot.Duration);
	}
}
