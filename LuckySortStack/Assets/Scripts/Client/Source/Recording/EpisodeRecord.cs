using System.Collections.Generic;

namespace LuckySortStack.Recording
{
	// Serialised form of one episode's task metadata, timeline and outcome. Lucky's Observer records
	// the robot (joint state, actions, camera); this records everything the Observer has no field for,
	// so the Observer dataset and this file together make a complete, reproducible episode.
	// Plain properties and arrays only, so System.Text.Json needs no converters.
	public sealed class EpisodeRecord
	{
		public int SchemaVersion { get; set; } = 1;
		public string Session { get; set; } = "";     // groups episodes recorded in one run
		public int Episode { get; set; }              // index within the session

		public int Seed { get; set; }
		public string Fingerprint { get; set; } = "";
		public ConfigRecord Config { get; set; } = new ConfigRecord();

		public List<ObjectRecord> Objects { get; set; } = new List<ObjectRecord>();
		public List<string> Plan { get; set; } = new List<string>();
		public List<TimelineEvent> Timeline { get; set; } = new List<TimelineEvent>();

		public ResultRecord Result { get; set; } = new ResultRecord();
		public List<StackRecord> Stacks { get; set; } = new List<StackRecord>();
	}

	public sealed class ConfigRecord
	{
		public int NumColors { get; set; }
		public int MinObjectsPerColor { get; set; }
		public int MaxObjectsPerColor { get; set; }
		public int NumSizeTiers { get; set; }
		public bool RandomizePositions { get; set; }
		public bool RandomizeRotations { get; set; }
	}

	public sealed class ObjectRecord
	{
		public string Tag { get; set; } = "";
		public string Color { get; set; } = "";
		public string Size { get; set; } = "";
		public float Edge { get; set; }
		public string TargetStack { get; set; } = "";
		public int TargetOrder { get; set; }
		public float[] InitialPosition { get; set; } = new float[3];
		public float InitialYawDeg { get; set; }
		public float[] FinalPosition { get; set; } = new float[3];   // physical, at evaluation time
		public float FinalYawDeg { get; set; }
		public float FinalTiltDeg { get; set; }
	}

	// One state transition or state-machine event, stamped with sim time since the task started.
	public sealed class TimelineEvent
	{
		public float Time { get; set; }
		public int Step { get; set; }                 // plan step index (0-based) at the time
		public string Kind { get; set; } = "";        // "state" or "event"
		public string Text { get; set; } = "";        // "MoveAboveObject -> AlignGripper" or the event message
	}

	public sealed class ResultRecord
	{
		public bool Success { get; set; }             // evaluator's verdict
		public string Category { get; set; } = "";
		public bool Disagreement { get; set; }        // state machine and evaluator disagreed
		public string RobotOutcome { get; set; } = "";
		public string RobotFailureDetail { get; set; } = "";
		public List<string> EvalFailures { get; set; } = new List<string>();
		public int CorrectlyStacked { get; set; }
		public int ExpectedObjects { get; set; }
		public int PickAttempts { get; set; }
		public int PlaceAttempts { get; set; }
		public int GraspFailures { get; set; }
		public int PlacementFailures { get; set; }
		public float Duration { get; set; }           // sim seconds, robot task only
	}

	public sealed class StackRecord
	{
		public string Color { get; set; } = "";
		public bool Success { get; set; }
		public List<string> Expected { get; set; } = new List<string>();   // bottom → top
		public List<string> Actual { get; set; } = new List<string>();     // bottom → top, as found
		public Dictionary<string, bool> Checks { get; set; } = new Dictionary<string, bool>();
		public List<string> Issues { get; set; } = new List<string>();
	}
}
