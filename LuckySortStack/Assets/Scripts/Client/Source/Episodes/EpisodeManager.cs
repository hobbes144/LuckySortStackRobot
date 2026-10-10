using System;
using System.Collections.Generic;
using Hazel;
using LuckySortStack.Core;
using LuckySortStack.Evaluation;
using LuckySortStack.Generation;
using LuckySortStack.Planning;
using LuckySortStack.Recording;
using LuckySortStack.Robot;
using LuckySortStack.StateMachine;
using Num = System.Numerics;

namespace LuckySortStack.Episodes
{
	// Owns one episode: reads the workspace from the scene, generates a task from
	// the seed, lays out the object pool, plans the pick/place sequence, runs
	// the task state machine on the robot, then has the StackEvaluator judge the result
	// and records the episode (Observer dataset + JSON episode record).
	public class EpisodeManager : Entity
	{
		[Group("Task")] [Tooltip("Same seed + config = same task.")]
		public int Seed = -1399230583;

		[Group("Task")] [Tooltip("Ignore Seed and draw a fresh one each Play (it is logged so it can be replayed).")]
		public bool RandomizeSeedOnPlay = false;

		[Group("Task")] [ClampValue(1, 4)]
		public int NumColors = 4;

		[Group("Task")] [ClampValue(1, 4)]
		public int MinObjectsPerColor = 2;

		[Group("Task")] [ClampValue(1, 4)]
		public int MaxObjectsPerColor = 4;

		[Group("Task")] [ClampValue(1, 4)]
		[Tooltip("Number of size tiers in play, from Large downwards.")]
		public int NumSizeTiers = 4;

		[Group("Task")]
		public bool RandomizePositions = true;

		[Group("Task")]
		public bool RandomizeRotations = true;

		[Group("Scene")]
		[Tooltip("Move pool objects to the generated layout at Play. Off = log the task only.")]
		public bool ApplyLayoutToScene = true;

		[Group("Scene")] [Units("m")]
		[Tooltip("World X of the row where unused pool objects are parked (behind the robot).")]
		public float ParkingX = -0.35f;

		[Group("Scene")] [Units("s")] [Slider(0.1f, 3.0f)]
		[Tooltip("Sim time to wait before checking that objects ended up where the layout put them.")]
		public float PlacementCheckDelay = 0.5f;

		[Group("Scene")] [Units("m")]
		public float PlacementTolerance = 0.005f;

		[Group("Robot")] [Tooltip("Run the planned pick/place sequence once the layout is verified.")]
		public bool RunTask = true;

		[Group("Robot")]
		public string RobotTag = "Panda";

		[Group("Robot")]
		public string GraspSiteTag = "grasp_site";

		[Group("Robot")]
		public string GripperActuatorName = "actuator8";

		[Group("Robot")]
		[Tooltip("Measure the gripper's real yaw before each descent/placement and correct it. " +
			"Sign and offset of the yaw command are learned from these measurements.")]
		public bool ClosedLoopYaw = true;

		[Group("Robot")] [Tooltip("Initial guess for the yaw command direction (+1/-1). Learned at runtime when ClosedLoopYaw is on.")]
		public float YawSign = 1f;

		[Group("Motion")] [Units("m")] [Slider(0.15f, 0.45f)]
		[Tooltip("Grasp-site height for all lateral moves. Must clear the tallest stack plus the held cube.")]
		public float TravelHeight = 0.28f;

		[Group("Motion")] [Units("m/s")] [Slider(0.05f, 1.0f)]
		public float TravelSpeed = 0.40f;

		[Group("Motion")] [Units("m/s")] [Slider(0.02f, 0.5f)]
		public float DescendSpeed = 0.15f;

		[Group("Motion")] [Units("m/s")] [Slider(0.02f, 0.3f)]
		public float PlaceDescendSpeed = 0.08f;

		[Group("Motion")] [Units("m")] [Slider(-0.03f, 0.03f)]
		[Tooltip("Grasp-site height relative to the object centre when grasping/placing (0 verified in Stage 1).")]
		public float GraspOffsetY = 0.0f;

		[Group("Motion")] [Units("m")] [Slider(0.005f, 0.05f)]
		[Tooltip("How close the grasp site must get to a target for a move to count as done.")]
		public float MoveTolerance = 0.015f;

		[Group("Motion")] [ClampValue(1, 5)]
		[Tooltip("Pick attempts per object, including retries after a failed placement.")]
		public int MaxPickAttempts = 3;

		[Group("Motion")] [ClampValue(1, 5)]
		public int MaxPlaceAttempts = 2;

		[Group("Evaluation")] [Units("m")] [Slider(0.005f, 0.05f)]
		[Tooltip("Max horizontal offset of any stacked object from its stack centre.")]
		public float EvalPositionTolerance = 0.015f;

		[Group("Evaluation")] [Units("s")] [Slider(0.2f, 5.0f)]
		[Tooltip("Sim time the finished scene is watched for movement before the verdict.")]
		public float EvalStableWindow = 1.0f;

		[Group("Recording")]
		[Tooltip("Drive Lucky's Observer: one Observer episode per task, labelled per pick, success = evaluator verdict.")]
		public bool RecordWithObserver = true;

		[Group("Recording")]
		[Tooltip("Write the episode's task, plan, timeline and evaluation as JSON (what the Observer can't store).")]
		public bool WriteEpisodeRecord = true;

		[Group("Recording")]
		[Tooltip("Relative to the project folder unless absolute. Each Play gets a timestamped session subfolder.")]
		public string RecordingDirectory = "Recordings";

		[Group("Debug")] [ClampValue(0, 16)]
		[Tooltip("Drop this many steps from the end of the plan. The state machine then 'succeeds' at a shortened " +
			"plan and the evaluator should report the missing objects: a demo that the evaluator doesn't trust the planner.")]
		public int DropPlanSteps = 0;

		public TaskDefinition? CurrentTask { get; private set; }
		public TaskStateMachine? StateMachine { get; private set; }
		public StackEvaluator? Evaluator { get; private set; }
		public EpisodeResult? Result { get; private set; }

		// Several EpisodeManagers would fight over the pool and the robot; only the first one runs.
		[HideFromEditor] private static EpisodeManager? s_Active;

		[HideFromEditor] private readonly Dictionary<string, Entity> m_Pool = new Dictionary<string, Entity>();
		[HideFromEditor] private readonly Dictionary<Entity, Vector3> m_Intended = new Dictionary<Entity, Vector3>();
		[HideFromEditor] private float m_CheckTimer = -1f;
		[HideFromEditor] private EpisodeRecorder? m_Recorder;
		[HideFromEditor] private ObserverSession? m_Observer;

		protected override void OnCreate()
		{
			// A leftover instance from a previous Play session has the same entity ID (or no
			// longer exists) and is simply replaced; a different live entity is a real duplicate.
			if (s_Active != null && s_Active.ID != ID && FindEntityByID(s_Active.ID) != null)
			{
				Log.Error($"[Episode] More than one EpisodeManager in the scene ('{s_Active.Tag}' and '{Tag}'). " +
					$"'{Tag}' is disabled; delete one of them.");
				return;
			}
			s_Active = this;

			WorkspaceSpec? workspace = ReadWorkspace();
			if (workspace == null) return;

			TaskConfig config = new TaskConfig
			{
				Seed               = RandomizeSeedOnPlay ? Environment.TickCount & 0x7FFFFFFF : Seed,
				NumColors          = NumColors,
				MinObjectsPerColor = MinObjectsPerColor,
				MaxObjectsPerColor = MaxObjectsPerColor,
				NumSizeTiers       = NumSizeTiers,
				RandomizePositions = RandomizePositions,
				RandomizeRotations = RandomizeRotations,
			};

			try
			{
				CurrentTask = TaskGenerator.Generate(config, workspace);
			}
			catch (ArgumentException e)
			{
				Log.Error($"[Episode] Invalid task config: {e.Message}");
				return;
			}

			LogTask(CurrentTask);

			if (ApplyLayoutToScene)
			{
				ApplyLayout(CurrentTask);
				m_CheckTimer = PlacementCheckDelay;
			}
			else if (RunTask)
			{
				Log.Warn("[Episode] RunTask needs ApplyLayoutToScene; the pool isn't where the task says it is.");
			}
		}

		protected override void OnUpdate(float ts)
		{
			if (s_Active != this) return;

			if (m_CheckTimer >= 0f)
			{
				m_CheckTimer -= ts;
				if (m_CheckTimer < 0f && CheckPlacement() && RunTask)
					StartTask();
				return;
			}

			if (StateMachine == null) return;
			StateMachine.Update(ts);

			// Once the robot is done (either way), judge the scene independently of the state machine.
			if (StateMachine.IsFinished && Evaluator == null && CurrentTask != null)
				Evaluator = new StackEvaluator(CurrentTask, new SceneWorldState(m_Pool), new EvaluationSettings
				{
					PositionTolerance = EvalPositionTolerance,
					StableWindow      = EvalStableWindow,
				});

			if (Evaluator != null && !Evaluator.IsComplete)
			{
				Evaluator.Update(ts);
				if (Evaluator.IsComplete) LogEvaluation(Evaluator.Report!);
			}
		}

		private void LogEvaluation(EvaluationReport report)
		{
			foreach (string line in report.Describe().Split('\n'))
				if (line.Length > 0) Log.Info("[Eval] " + line);

			Result = EpisodeResult.From(CurrentTask!, StateMachine!.Metrics, report);
			if (Result.Disagreement)
				Log.Warn($"[Eval] State machine reported {StateMachine.State} but the evaluator says {(report.Success ? "PASS" : "FAIL")}. " +
					"The evaluator is the source of truth.");
			Log.Info("[Result] " + Result);
			FinishRecording();
		}

		protected override void OnDestroy()
		{
			// Play stopped before the verdict: close the Observer episode as unsuccessful rather than leaving it open.
			m_Observer?.EndEpisode(success: false, stopRecording: true);
			if (s_Active == this) s_Active = null;
		}

		private void StartTask()
		{
			if (CurrentTask == null) return;

			Entity? robot     = FindEntityByTag(RobotTag);
			Entity? graspSite = FindEntityByTag(GraspSiteTag);
			RobotControllerComponent? controller = robot?.GetComponent<RobotControllerComponent>();
			MujocoActuator? gripper = robot?.GetComponent<MujocoSceneComponent>()?.GetActuator(GripperActuatorName);
			if (controller == null || gripper == null || graspSite == null)
			{
				Log.Error($"[Episode] Robot wiring incomplete: robot '{RobotTag}' controller={controller != null} " +
					$"gripper '{GripperActuatorName}'={gripper != null} grasp site '{GraspSiteTag}'={graspSite != null}");
				return;
			}

			var arm = new PandaManipulator(controller, gripper, graspSite);
			var settings = new MotionSettings
			{
				TravelHeight      = TravelHeight,
				TravelSpeed       = TravelSpeed,
				DescendSpeed      = DescendSpeed,
				PlaceDescendSpeed = PlaceDescendSpeed,
				GraspOffsetY      = GraspOffsetY,
				ClosedLoopYaw     = ClosedLoopYaw,
				YawSign           = YawSign,
				MoveTolerance     = MoveTolerance,
				MaxPickAttempts   = MaxPickAttempts,
				MaxPlaceAttempts  = MaxPlaceAttempts,
			};

			List<PlannedStep> plan = StackPlanner.Plan(CurrentTask);
			if (DropPlanSteps > 0)
			{
				int drop = Math.Min(DropPlanSteps, plan.Count);
				plan.RemoveRange(plan.Count - drop, drop);
				Log.Warn($"[Plan] Debug: dropped the last {drop} step(s); the evaluator should report them missing.");
			}
			foreach (string line in StackPlanner.Describe(plan).Split('\n'))
				if (line.Length > 0) Log.Info("[Plan] " + line);

			StateMachine = new TaskStateMachine(CurrentTask, plan, arm, new SceneWorldState(m_Pool), settings);
			StateMachine.Log += message => Log.Info("[Task] " + message);
			StateMachine.StateChanged += (from, to) => Log.Info($"[Task] {from} -> {to}");
			StartRecording(plan);
			StateMachine.Start();
		}

		// ── Recording ───────────────────────────────────────────────────────

		private void StartRecording(List<PlannedStep> plan)
		{
			if (WriteEpisodeRecord)
			{
				string root = ResolveRecordingRoot();
				string session = DateTime.Now.ToString("yyyyMMdd_HHmmss");   // folder name only; never used for timing
				m_Recorder = new EpisodeRecorder(System.IO.Path.Combine(root, session), session);
				m_Recorder.Begin(CurrentTask!, plan, StateMachine!);
				Log.Info($"[Record] Episode records -> {m_Recorder.SessionDirectory}");
			}

			if (RecordWithObserver)
			{
				m_Observer = new ObserverSession();
				m_Observer.BeginEpisode();
				// Label frames per pick: entering MoveAboveObject is the start of each pick attempt.
				StateMachine!.StateChanged += (_, to) =>
				{
					if (to == TaskState.MoveAboveObject && StateMachine.CurrentStep != null)
						m_Observer.SetStep(StateMachine.CurrentStep);
				};
			}
		}

		private void FinishRecording()
		{
			// Observer episode success is the evaluator's verdict: the dataset must not inherit the robot's opinion.
			m_Observer?.EndEpisode(Result!.Success, stopRecording: true);

			if (m_Recorder == null) return;
			try
			{
				string path = m_Recorder.Finish(Result!, new SceneWorldState(m_Pool));
				Log.Info($"[Record] Wrote {path}");
			}
			catch (Exception e) when (e is System.IO.IOException || e is UnauthorizedAccessException)
			{
				Log.Error($"[Record] Could not write the episode record: {e.Message}");
			}
		}

		// RecordingDirectory if absolute; otherwise relative to the project folder (the one holding the .hproj),
		// found by walking up from the script assembly or the working directory.
		//
		// The engine exposes no project path; the script assembly is loaded from memory (empty Location)
		// and the editor's working directory is C:\Windows\System32. The reliable anchor is this source
		// file's compile-time path: the editor compiles the project's own scripts, so it's inside the project.
		private string ResolveRecordingRoot()
		{
			if (System.IO.Path.IsPathRooted(RecordingDirectory)) return RecordingDirectory;

			foreach (string start in new[] { SourceFilePath(), typeof(EpisodeManager).Assembly.Location })
			{
				if (string.IsNullOrEmpty(start)) continue;
				for (var dir = new System.IO.DirectoryInfo(start); dir != null; dir = dir.Parent)
					if (dir.Exists && dir.GetFiles("*.hproj").Length > 0)
						return System.IO.Path.Combine(dir.FullName, RecordingDirectory);
			}

			string fallback = System.IO.Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments), "LuckySort", RecordingDirectory);
			Log.Warn($"[Record] Project folder not found; writing to {fallback}. Set RecordingDirectory to an absolute path to choose.");
			return fallback;
		}

		private static string SourceFilePath([System.Runtime.CompilerServices.CallerFilePath] string path = "") => path;

		// Build the generator's view of the workspace from actual scene geometry.
		private WorkspaceSpec? ReadWorkspace()
		{
			WorkspaceSpec ws = WorkspaceSpec.Default;
			ws.EdgeLength.Clear();
			ws.StackBase.Clear();

			Entity? surface = FindEntityByTag(SceneTags.Workspace);
			if (surface == null)
			{
				Log.Error($"[Episode] '{SceneTags.Workspace}' not found. Run Tools/SceneBuilder.");
				return null;
			}
			ws.SurfaceY = surface.Transform.WorldTranslation.Y + surface.Transform.Scale.Y * 0.5f;

			foreach (ObjectColor color in Enum.GetValues(typeof(ObjectColor)))
			{
				Entity? marker = FindEntityByTag(SceneTags.StackTarget(color));
				if (marker == null)
				{
					Log.Error($"[Episode] Stack target '{SceneTags.StackTarget(color)}' not found.");
					return null;
				}
				Vector3 top = marker.Transform.WorldTranslation + new Vector3(0f, marker.Transform.Scale.Y * 0.5f, 0f);
				ws.StackBase[color] = ToNum(top);

				foreach (ObjectSize size in Enum.GetValues(typeof(ObjectSize)))
				{
					string tag = SceneTags.PoolObject(color, size);
					Entity? obj = FindEntityByTag(tag);
					if (obj == null)
					{
						Log.Error($"[Episode] Pool object '{tag}' not found.");
						return null;
					}
					m_Pool[tag] = obj;
					ws.EdgeLength[size] = obj.Transform.Scale.Y;   // physical size is the source of truth
				}
			}
			return ws;
		}

		private void ApplyLayout(TaskDefinition task)
		{
			var used = new HashSet<string>();
			foreach (SortObjectDefinition def in task.Objects)
			{
				Entity obj = m_Pool[def.PoolTag];
				Place(obj, ToHazel(def.InitialPosition), def.InitialYaw);
				used.Add(def.PoolTag);
			}

			// Park everything else in a row behind the robot, resting on the ground (y = 0).
			int slot = 0;
			foreach (var kv in m_Pool)
			{
				if (used.Contains(kv.Key)) continue;
				float edge = kv.Value.Transform.Scale.Y;
				Place(kv.Value, new Vector3(ParkingX, edge * 0.5f + 0.002f, -0.50f + slot * 0.07f), 0f);
				slot++;
			}
		}

		private void Place(Entity obj, Vector3 position, float yaw)
		{
			obj.Transform.WorldTranslation = position;
			obj.Transform.WorldRotation = new Vector3(0f, yaw, 0f);
			m_Intended[obj] = position;
		}

		// Placement by transform write before physics starts is an unverified engine
		// behaviour for MuJoCo bodies (see LUCKY_API_NOTES §8). This makes it obvious.
		private bool CheckPlacement()
		{
			int ok = 0;
			foreach (var kv in m_Intended)
			{
				Vector3 actual = kv.Key.Transform.WorldTranslation;
				float dx = actual.X - kv.Value.X, dz = actual.Z - kv.Value.Z;
				float horizontal = Mathf.Sqrt(dx * dx + dz * dz);
				if (horizontal <= PlacementTolerance) { ok++; continue; }
				Log.Warn($"[Episode] {kv.Key.Tag}: intended ({kv.Value.X:F3}, {kv.Value.Z:F3}) actual ({actual.X:F3}, {actual.Z:F3}) off by {horizontal * 1000f:F1} mm");
			}

			if (ok == m_Intended.Count)
			{
				Log.Info($"[Episode] Placement check passed: {ok}/{m_Intended.Count} objects within {PlacementTolerance * 1000f:F0} mm.");
				return true;
			}
			Log.Warn($"[Episode] Placement check FAILED: {ok}/{m_Intended.Count} objects within tolerance. Task not started.");
			return false;
		}

		private static void LogTask(TaskDefinition task)
		{
			Log.Info($"[Episode] Generated task {task.Fingerprint()}{(task.UsedFallbackLayout ? " (grid fallback layout)" : "")}");
			foreach (string line in task.Describe().Split('\n'))
				if (line.Length > 0) Log.Info("[Episode] " + line);
		}

		private static Num.Vector3 ToNum(Vector3 v) => new Num.Vector3(v.X, v.Y, v.Z);
		private static Vector3 ToHazel(Num.Vector3 v) => new Vector3(v.X, v.Y, v.Z);
	}
}
