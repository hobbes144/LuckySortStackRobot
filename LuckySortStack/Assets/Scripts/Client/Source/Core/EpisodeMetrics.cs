namespace LuckySortStack.Core
{
	public enum FailureReason
	{
		None,
		GraspFailed,       // object not in hand after lift, retries exhausted
		PlacementFailed,   // object not where it was placed, retries exhausted
		StackCollapsed,    // a previously verified object moved off its stack
		Unreachable,       // the arm never converged on a commanded pose
		Timeout,           // episode exceeded its time budget
	}

	// Per-episode counters; aggregated by benchmark mode later.
	public sealed class EpisodeMetrics
	{
		public int PickAttempts;
		public int PlaceAttempts;
		public int GraspFailures;
		public int PlacementFailures;
		public int ObjectsPlaced;
		public int Deferrals;             // picks moved to the end of the plan after running out of attempts
		public int BufferMoves;           // cubes taken off a pile and set on a buffer spot (cluttered starts)
		public float Duration;            // sim seconds
		public FailureReason Failure = FailureReason.None;
		public string FailureDetail = "";

		public bool Succeeded => Failure == FailureReason.None;

		public override string ToString() =>
			$"placed={ObjectsPlaced} picks={PickAttempts} places={PlaceAttempts} graspFail={GraspFailures} " +
			$"placeFail={PlacementFailures} duration={Duration:F1}s" + (Deferrals > 0 ? $" deferred={Deferrals}" : "") + (BufferMoves > 0 ? $" buffered={BufferMoves}" : "") + (Succeeded ? "" : $" failure={Failure} ({FailureDetail})");
	}
}
