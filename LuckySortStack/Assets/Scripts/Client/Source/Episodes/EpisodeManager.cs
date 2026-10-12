using System;
using System.Collections.Generic;
using Hazel;
using LuckySortStack.Benchmark;
using LuckySortStack.Core;
using LuckySortStack.Evaluation;
using LuckySortStack.Generation;
using LuckySortStack.Planning;
using LuckySortStack.Presentation;
using LuckySortStack.Recording;
using LuckySortStack.Robot;
using LuckySortStack.StateMachine;
using LuckySortStack.UI;
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
		public int Seed = 1234;

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

		[Group("Task")] [ClampValue(0, 4)]
		[Tooltip("Cluttered starts: piles of mixed cubes at the start (docs/CLUTTER_DESIGN.md). 0 = off, and every seed gives the task it always did.")]
		public int PileCount = 2;

		[Group("Task")] [ClampValue(2, 4)]
		[Tooltip("Cubes per pile, including the bottom one.")]
		public int MaxPileHeight = 3;

		[Group("Task")]
		[Tooltip("Allow a larger cube on a smaller one in a starting pile (harder; may topple at reset).")]
		public bool AllowInvertedPiles = false;

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

		[Group("Scene")] [Units("s")] [Slider(0.5f, 5.0f)]
		[Tooltip("After an in-Play reset (benchmark episodes 2+), time for the arm to return to its start pose " +
			"before the placement check and the next task.")]
		public float RehomeSeconds = 1.5f;

		[Group("Robot")] [Tooltip("Run the planned pick/place sequence once the layout is verified.")]
		public bool RunTask = true;

		[Group("Robot")]
		public string RobotTag = "Panda";

		[Group("Robot")]
		public string GraspSiteTag = "grasp_site";

		[Group("Robot")]
		public string GripperActuatorName = "actuator8";

		[Group("Robot")]
		public string LeftFingerTag = "left_finger";

		[Group("Robot")]
		public string RightFingerTag = "right_finger";

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

		[Group("Motion")]
		[Tooltip("Retries change the approach: grasp turned ±90° (a cube is symmetric), and a lower centring hover after a " +
			"collision. Off = every retry repeats the first approach exactly (the pre-2026-10-12 behaviour).")]
		public bool VaryRetries = true;

		[Group("Motion")]
		[Tooltip("Grip each cube across the pair of faces whose open fingers have the most room from neighbouring cubes, " +
			"checked against the measured finger axis. Off = whichever axis the yaw alignment lands on.")]
		public bool ChooseGraspAxis = true;

		[Group("Motion")]
		[Tooltip("When a cube runs out of pick attempts, build the other stacks first and come back to it once, " +
			"instead of ending the episode. Off = fail on the first exhausted pick.")]
		public bool DeferFailedPicks = true;

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

		[Group("Benchmark")] [ClampValue(1, 1000)]
		[Tooltip("Episodes to run in this Play session, on seeds Seed, Seed+1, … (must be >= 0 when more than 1). " +
			"Between episodes the scene is reset in place. 1 = a single episode, as before.")]
		public int BenchmarkEpisodes = 1;

		[Group("Debug")]
		[Tooltip("Reset spike: lay out a few consecutive seeds via the in-Play reset and check each placement, " +
			"without running the robot. Verifies SceneResetter in seconds before a long benchmark.")]
		public bool ResetSpike = false;

		[Group("Debug")] [ClampValue(0, 16)]
		[Tooltip("Drop this many steps from the end of the plan. The state machine then 'succeeds' at a shortened " +
			"plan and the evaluator should report the missing objects: a demo that the evaluator doesn't trust the planner.")]
		public int DropPlanSteps = 0;

		[Group("Overlay")] [Tooltip("On-screen presentation overlay: episode, seed, state, current step, stack progress, verdict.")]
		public bool ShowOverlay = true;

		[Group("Overlay")] [Tooltip("In-world markers: stack bases, current target, arm target, chosen finger axis.")]
		public bool ShowMarkers = true;

		[Group("Overlay")] [Tooltip("The overlay text is parented to this camera so it stays fixed on screen.")]
		public string CameraTag = "Camera";

		[Group("Overlay")] [Slider(0.01f, 0.06f)]
		[Tooltip("Height of one overlay text line as a fraction of the screen height. The panel is anchored top-left.")]
		public float OverlayLineHeight = 0.022f;

		[Group("Overlay")] [Slider(0f, 0.1f)]
		[Tooltip("Gap between the panel and the screen's top-left corner, as a fraction of the screen height.")]
		public float OverlayMargin = 0.02f;

		[Group("Overlay")] [Slider(1f, 4f)]
		[Tooltip("Viewport width/height, used only if it can't be measured through CameraComponent.ToScreenSpace (see the log).")]
		public float OverlayFallbackAspect = 1.7778f;

		[Group("Overlay")] [Units("s")] [Slider(0f, 10f)]
		[Tooltip("Benchmark mode: keep each episode's verdict on screen this long (sim time) before the next reset.")]
		public float ResultHoldSeconds = 3f;

		[Group("Presentation")]
		[Dropdown("Authored", "Overview", "Front", "Top-down", "Follow")]
		[Tooltip("Camera view at the start of Play. Cycle with C (viewport focused) or the 'Next camera view' button.")]
		public int InitialCameraView = 0;

		[Group("Presentation")]
		[Tooltip("Seed for 'Replay chosen seed' (P). R replays the current seed, F the last failed one.")]
		public int ReplaySeed = 0;

		[Group("Presentation")]
		[Tooltip("Keyboard shortcuts while the viewport has focus: C camera, R replay current, F replay last failed, P replay ReplaySeed.")]
		public bool KeyboardShortcuts = true;

		public TaskDefinition? CurrentTask { get; private set; }
		public TaskStateMachine? StateMachine { get; private set; }
		public StackEvaluator? Evaluator { get; private set; }
		public EpisodeResult? Result { get; private set; }

		// Several EpisodeManagers would fight over the pool and the robot; only the first one runs.
		[HideFromEditor] private static EpisodeManager? s_Active;

		[HideFromEditor] private readonly Dictionary<string, Entity> m_Pool = new Dictionary<string, Entity>();
		[HideFromEditor] private readonly Dictionary<Entity, (Vector3 Position, float Yaw)> m_Intended = new Dictionary<Entity, (Vector3, float)>();
		[HideFromEditor] private float m_CheckTimer = -1f;
		[HideFromEditor] private EpisodeRecorder? m_Recorder;
		[HideFromEditor] private ObserverSession? m_Observer;

		// Episode sequence (benchmark mode; a single episode is a sequence of one).
		[HideFromEditor] private WorkspaceSpec? m_Workspace;
		[HideFromEditor] private List<int> m_Seeds = new List<int>();
		[HideFromEditor] private int m_EpisodeIndex = -1;
		[HideFromEditor] private readonly List<EpisodeResult> m_Results = new List<EpisodeResult>();
		[HideFromEditor] private SceneResetter? m_Resetter;
		[HideFromEditor] private bool m_SequenceDone;
		[HideFromEditor] private bool m_PendingStart;
		[HideFromEditor] private bool m_Started;
		[HideFromEditor] private DebugOverlay? m_Overlay;
		[HideFromEditor] private PresentationCamera? m_PresentationCamera;
		[HideFromEditor] private bool m_FreeStepSeen;
		[HideFromEditor] private bool m_FreeStepHinted;
		[HideFromEditor] private float m_FreeStepWait;
		[HideFromEditor] private int? m_PendingReplay;
		[HideFromEditor] private bool m_Replaying;
		[HideFromEditor] private int? m_LastFailedSeed;
		[HideFromEditor] private readonly DebugOverlaySettings m_OverlaySettings = new DebugOverlaySettings();
		[HideFromEditor] private string m_Phase = "";
		[HideFromEditor] private float m_OverlayRefresh;
		[HideFromEditor] private float m_HoldTimer = -1f;
		[HideFromEditor] private Vector3? m_HomeGraspPosition;   // grasp_site at the start of Play, before anything moved

		private bool MultiEpisode => m_Seeds.Count > 1;

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

			m_Workspace = ReadWorkspace();
			if (m_Workspace == null) return;

			int baseSeed = RandomizeSeedOnPlay ? Environment.TickCount & 0x7FFFFFFF : Seed;
			int count = ResetSpike ? Math.Max(3, BenchmarkEpisodes) : BenchmarkEpisodes;
			if (count == 1)
			{
				m_Seeds = new List<int> { baseSeed };
			}
			else
			{
				if (baseSeed < 0)
				{
					Log.Warn($"[Benchmark] Seed {baseSeed} is negative; System.Random(seed) uses |seed|, so negative seeds alias " +
						$"positive ones. Using {baseSeed & 0x7FFFFFFF} as the base.");
					baseSeed &= 0x7FFFFFFF;
				}
				count = (int)Math.Min(count, (long)int.MaxValue - baseSeed + 1);
				m_Seeds = BenchmarkSeeds.Make(baseSeed, count);
				Log.Info($"[Benchmark] {(ResetSpike ? "Reset spike" : "Benchmark")}: {count} episodes, seeds {m_Seeds[0]}..{m_Seeds[^1]}");
			}

			if (!ApplyLayoutToScene)
			{
				if (RunTask) Log.Warn("[Episode] RunTask needs ApplyLayoutToScene; the pool isn't where the task says it is.");
				if (GenerateTask(m_Seeds[0]) != null) LogTask(CurrentTask!);
				return;
			}
			m_PendingStart = true;   // layout happens on the first update, once MuJoCo state can be read
		}

		protected override void OnUpdate(float ts)
		{
			if (s_Active != this) return;
			if (m_PendingReplay is int replaySeed && m_Started)
			{
				m_PendingReplay = null;
				StartReplay(replaySeed);
				return;
			}
			if (m_SequenceDone) return;

			if (m_PendingStart && !m_Started)
			{
				StartSequence();
				return;
			}

			if (m_CheckTimer >= 0f)
			{
				m_CheckTimer -= ts;
				if (m_CheckTimer < 0f) OnLayoutSettled();
				return;
			}

			// Verdict on screen for a moment before the next episode's reset wipes the scene.
			if (m_HoldTimer >= 0f)
			{
				m_HoldTimer -= ts;
				if (m_HoldTimer < 0f) NextEpisode();
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
			m_Results.Add(Result);
			if (!Result.Success) m_LastFailedSeed = CurrentTask!.Seed;
			bool more = m_EpisodeIndex + 1 < m_Seeds.Count;
			if (more && m_Overlay != null && ResultHoldSeconds > 0f) m_HoldTimer = ResultHoldSeconds;
			else NextEpisode();
		}

		// ── Overlay (Milestone 8) ───────────────────────────────────────────

		// Two clocks (docs: scripting/getting-started/entity-lifecycle):
		//  - OnUpdate/OnLateUpdate run once per fixed simulation step (50 Hz); a rendered frame can contain zero, one or
		//    several steps. DebugRenderer is immediate-mode, so markers drawn here vanish on frames with no step: they blink.
		//  - OnFreePostUpdate runs once per rendered frame (scene setting "Enable Free Step Update Scripts", FreeStepEnabled).
		// So: text (persistent entities) refreshes here at 10 Hz of sim time; markers, camera blending and key presses
		// (IsKeyPressed is true for one rendered frame only) go in PerFrame, driven by OnFreePostUpdate. If that callback
		// never runs, PerFrame falls back to this one and the markers blink, with a log line saying how to fix it.
		protected override void OnLateUpdate(float ts)
		{
			if (s_Active != this) return;
			if (!m_FreeStepSeen)
			{
				PerFrame(ts);
				m_FreeStepWait += ts;
				if (m_FreeStepWait > 2f && !m_FreeStepHinted)
				{
					m_FreeStepHinted = true;
					Log.Warn("[Overlay] OnFreePostUpdate isn't running, so markers are drawn per simulation step and will blink. " +
						"Turn on Settings → Scene → Scripts → 'Enable Free Step Update Scripts' (FreeStepEnabled in the scene).");
				}
			}
			if (m_Overlay == null) return;

			m_OverlayRefresh -= ts;
			if (m_OverlayRefresh <= 0f)
			{
				m_OverlayRefresh = 0.1f;
				m_OverlaySettings.LineHeight     = OverlayLineHeight;   // live: tweak the sliders during Play
				m_OverlaySettings.Margin         = OverlayMargin;
				m_OverlaySettings.FallbackAspect = OverlayFallbackAspect;
				m_OverlaySettings.DrawMarkers    = ShowMarkers;
				m_Overlay.Layout();
				int passed = 0;
				foreach (EpisodeResult r in m_Results) if (r.Success) passed++;
				m_Overlay.Show(OverlayFormatter.Build(new OverlayInput
				{
					EpisodeNumber  = Math.Max(1, m_EpisodeIndex + 1),
					EpisodeCount   = Math.Max(1, m_Seeds.Count),
					Task           = CurrentTask,
					Machine        = StateMachine,
					Report         = Evaluator?.Report,
					Evaluating     = StateMachine != null && StateMachine.IsFinished && (Evaluator == null || !Evaluator.IsComplete),
					BenchmarkDone  = MultiEpisode ? m_Results.Count : 0,
					BenchmarkPassed = passed,
					ElapsedSeconds = StateMachine?.Metrics.Duration ?? 0f,
					Phase          = m_Phase,
					Replay         = m_Replaying,
					KeyHints       = KeyboardShortcuts,
				}));
			}

		}

		protected override void OnFreePostUpdate(float ts)
		{
			if (s_Active != this) return;
			m_FreeStepSeen = true;
			PerFrame(ts);
		}

		// Once per rendered frame: shortcuts, camera, and the immediate-mode markers.
		private void PerFrame(float ts)
		{
			HandleShortcuts();
			UpdateCamera(ts);
			if (m_Overlay == null || !ShowMarkers || CurrentTask == null || StateMachine == null) return;
			var world = new SceneWorldState(m_Pool);
			Entity? site = FindEntityByTag(GraspSiteTag);
			Vector3 g = site?.Transform.WorldTranslation ?? new Vector3(0f, 0f, 0f);
			m_Overlay.DrawMarkers(CurrentTask, StateMachine, world.GetObjectPosition, world.GetObjectYaw, new Num.Vector3(g.X, g.Y, g.Z));
		}

		// ── Presentation camera + replay (Milestone 8) ─────────────────────

		private void HandleShortcuts()
		{
			if (!KeyboardShortcuts || !m_Started) return;
			if (Input.IsKeyPressed(KeyCode.C)) NextCameraView();
			if (Input.IsKeyPressed(KeyCode.R)) ReplayCurrentSeed();
			if (Input.IsKeyPressed(KeyCode.F)) ReplayLastFailedSeed();
			if (Input.IsKeyPressed(KeyCode.P)) ReplayChosenSeed();
		}

		private void UpdateCamera(float ts)
		{
			if (m_PresentationCamera == null) return;
			// Follow view tracks the cube being picked, otherwise the gripper.
			Num.Vector3 focus;
			PlannedStep? step = StateMachine != null && !StateMachine.IsFinished ? StateMachine.CurrentStep : null;
			if (step != null && m_Pool.TryGetValue(step.Object.PoolTag, out Entity? cube) && cube != null)
			{
				Vector3 c = cube.Transform.WorldTranslation;
				focus = new Num.Vector3(c.X, c.Y, c.Z);
			}
			else
			{
				Vector3 g = FindEntityByTag(GraspSiteTag)?.Transform.WorldTranslation ?? new Vector3(0.5f, 0.1f, 0f);
				focus = new Num.Vector3(g.X, g.Y, g.Z);
			}
			m_PresentationCamera.Update(ts, focus);
		}

		[Button("Next camera view", RuntimeOnly = true)]
		internal void NextCameraView() => m_PresentationCamera?.Next();

		[Button("Replay current seed", RuntimeOnly = true)]
		internal void ReplayCurrentSeed()
		{
			if (CurrentTask != null) RequestReplay(CurrentTask.Seed);
			else if (m_Seeds.Count > 0) RequestReplay(m_Seeds[Math.Max(0, m_EpisodeIndex)]);
		}

		[Button("Replay last failed seed", RuntimeOnly = true)]
		internal void ReplayLastFailedSeed()
		{
			if (m_LastFailedSeed is int seed) RequestReplay(seed);
			else Log.Info("[Replay] No failed episode yet this session.");
		}

		[Button("Replay chosen seed (ReplaySeed)", RuntimeOnly = true)]
		internal void ReplayChosenSeed() => RequestReplay(ReplaySeed);

		// Queued and handled at the top of the next OnUpdate, so a replay never starts halfway through a state-machine update.
		private void RequestReplay(int seed)
		{
			m_PendingReplay = seed;
			Log.Info($"[Replay] Requested seed {seed}");
		}

		// Abort whatever is running and run `seed` as a single episode through the same in-Play reset as benchmark mode.
		// A benchmark in progress is closed first with a PARTIAL summary, so its numbers aren't mixed with the replay.
		private void StartReplay(int seed)
		{
			if (m_Resetter == null)
			{
				Log.Warn("[Replay] Needs the in-Play reset, which isn't calibrated this session (see the [Reset] log). Restart Play instead.");
				return;
			}

			bool running = StateMachine != null && Result == null;
			if (running)
			{
				Log.Info($"[Replay] Aborting episode {m_EpisodeIndex + 1} (seed {CurrentTask?.Seed}) at {StateMachine!.State}.");
				m_Observer?.EndEpisode(success: false, stopRecording: false);
			}
			if (MultiEpisode && !m_SequenceDone && m_Results.Count > 0) FinishSequence(partial: true);

			m_Seeds = new List<int> { seed };
			m_Results.Clear();
			m_SequenceDone = false;
			m_Replaying = true;
			m_HoldTimer = -1f;
			m_CheckTimer = -1f;
			Log.Info($"[Replay] ── Seed {seed} ──");
			BeginEpisode(0, forceReset: true);
		}

		private void CreateOverlay()
		{
			if (!ShowOverlay || m_Overlay != null) return;
			m_Overlay = new DebugOverlay(FindEntityByTag(CameraTag), m_OverlaySettings);
		}

		protected override void OnDestroy()
		{
			// Play stopped before the verdict: close the Observer episode as unsuccessful rather than leaving it open.
			m_Observer?.EndEpisode(success: false, stopRecording: true);
			// A benchmark cut short still reports what it got through.
			if (MultiEpisode && !m_SequenceDone && m_Results.Count > 0) FinishSequence(partial: true);
			m_Overlay?.Destroy();
			m_Overlay = null;
			if (s_Active == this) s_Active = null;
		}

		// ── Episode sequence ────────────────────────────────────────────────

		private TaskDefinition? GenerateTask(int seed)
		{
			var config = new TaskConfig
			{
				Seed               = seed,
				NumColors          = NumColors,
				MinObjectsPerColor = MinObjectsPerColor,
				MaxObjectsPerColor = MaxObjectsPerColor,
				NumSizeTiers       = NumSizeTiers,
				RandomizePositions = RandomizePositions,
				RandomizeRotations = RandomizeRotations,
				PileCount          = PileCount,
				MaxPileHeight      = MaxPileHeight,
				AllowInvertedPiles = AllowInvertedPiles,
			};
			try
			{
				CurrentTask = TaskGenerator.Generate(config, m_Workspace!);
				return CurrentTask;
			}
			catch (ArgumentException e)
			{
				Log.Error($"[Episode] Invalid task config: {e.Message}");
				CurrentTask = null;
				return null;
			}
		}

		// Generate episode i's task and put the pool in its layout with one atomic qpos write (SceneResetter),
		// for the first episode as well as every later one.
		private void BeginEpisode(int index, bool forceReset = false)
		{
			m_EpisodeIndex = index;
			m_Phase = "Laying out cubes";
			StateMachine = null;
			Evaluator = null;
			Result = null;

			TaskDefinition? task = GenerateTask(m_Seeds[index]);
			if (task == null) { FinishSequence(partial: true); return; }
			if (MultiEpisode) Log.Info($"[Benchmark] ── Episode {index + 1}/{m_Seeds.Count}, seed {task.Seed} ──");
			LogTask(task);

			Dictionary<string, (Vector3 Position, float Yaw)> layout = ComputeLayout(task);
			m_Intended.Clear();
			foreach (var kv in layout) m_Intended[m_Pool[kv.Key]] = kv.Value;

			if (m_Resetter != null)
			{
				// Episode 1: MuJoCo is already at its initial state; ResetToInitialState here would only clear
				// the arm's actuator targets, and it slumps. Later episodes: full reset, then send the arm home.
				bool resetPhysics = index > 0 || forceReset;
				m_Resetter.Reset(layout, resetPhysics);
				if (resetPhysics) RehomeArm();
			}
			else
			{
				// Fallback only. Transform writes don't move the MuJoCo bodies all at once: a cube can arrive
				// while another still sits in its pre-Play spot, or pass through one, and get flung
				// (seed 748253065: LS_Obj_Red_Small ended up 430 mm away).
				foreach (var kv in layout)
				{
					m_Pool[kv.Key].Transform.WorldTranslation = kv.Value.Position;
					m_Pool[kv.Key].Transform.WorldRotation = new Vector3(0f, kv.Value.Yaw, 0f);
				}
			}
			m_CheckTimer = (index > 0 || forceReset) && m_Resetter != null ? Math.Max(PlacementCheckDelay, RehomeSeconds) : PlacementCheckDelay;
		}

		// After ResetToInitialState the motion graph still holds the previous episode's last IK target and the
		// gripper control is zeroed. Command the start-of-Play grasp-site pose and reopen the gripper so the
		// next episode starts from the same place as the first.
		private void RehomeArm()
		{
			Entity? robot     = FindEntityByTag(RobotTag);
			Entity? graspSite = FindEntityByTag(GraspSiteTag);
			RobotControllerComponent? controller = robot?.GetComponent<RobotControllerComponent>();
			MujocoActuator? gripper = robot?.GetComponent<MujocoSceneComponent>()?.GetActuator(GripperActuatorName);
			if (controller == null || gripper == null || graspSite == null || m_HomeGraspPosition == null)
			{
				Log.Warn("[Reset] Can't send the arm home (robot wiring or start pose missing); the next task starts from wherever it is.");
				return;
			}
			var arm = new PandaManipulator(controller, gripper, graspSite);
			Vector3 home = m_HomeGraspPosition.Value;
			arm.SetGripper(1f);
			arm.MoveTo(new Num.Vector3(home.X, home.Y, home.Z), 0f, RehomeSeconds);
			Log.Info($"[Reset] Arm sent home to ({home.X:F3}, {home.Y:F3}, {home.Z:F3}) over {RehomeSeconds:F1} s, gripper open.");
		}

		// First frame of Play: the pool is at its saved scene positions, which never overlap, so this is
		// the moment to calibrate the resetter. Every layout, including the first, then goes through it.
		private void StartSequence()
		{
			m_Started = true;
			CreateOverlay();
			Entity? camera = FindEntityByTag(CameraTag);
			if (camera != null)
				m_PresentationCamera = new PresentationCamera(camera, (CameraView)Math.Clamp(InitialCameraView, 0, CameraViews.Count - 1));
			else
				Log.Warn($"[Camera] No entity tagged '{CameraTag}'; camera views disabled.");
			Entity? site = FindEntityByTag(GraspSiteTag);
			if (site != null) m_HomeGraspPosition = site.Transform.WorldTranslation;
			MujocoSceneComponent? mujoco = FindEntityByTag(RobotTag)?.GetComponent<MujocoSceneComponent>();
			var resetter = mujoco != null ? new SceneResetter(mujoco) : null;
			if (resetter != null && resetter.Calibrate(m_Pool))
			{
				m_Resetter = resetter;
			}
			else
			{
				Log.Warn("[Reset] SceneResetter unavailable; laying out by transform writes (can fling overlapping cubes) " +
					"and running a single episode.");
				m_Seeds = new List<int> { m_Seeds[0] };
			}
			BeginEpisode(0);
		}

		// The layout has had PlacementCheckDelay of sim time to settle.
		private void OnLayoutSettled()
		{
			bool placed = CheckPlacement();

			if (!placed)
			{
				m_Phase = "LAYOUT FAILED (see log)";
				Log.Error($"[Reset] Layout for episode {m_EpisodeIndex + 1} (seed {m_Seeds[m_EpisodeIndex]}) didn't land; stopping. " +
					"See the per-object errors above.");
				FinishSequence(partial: true);
				return;
			}

			if (ResetSpike)
			{
				Log.Info($"[Reset] Spike {m_EpisodeIndex + 1}/{m_Seeds.Count}: layout for seed {m_Seeds[m_EpisodeIndex]} " +
					$"placed by {(m_Resetter != null ? "qpos reset" : "transform write")} — OK");
				if (m_EpisodeIndex + 1 < m_Seeds.Count) BeginEpisode(m_EpisodeIndex + 1);
				else { Log.Info("[Reset] Spike passed: every layout placed all objects within tolerance."); m_SequenceDone = true; }
				return;
			}

			if (RunTask) StartTask();
		}

		private void NextEpisode()
		{
			if (m_EpisodeIndex + 1 < m_Seeds.Count) BeginEpisode(m_EpisodeIndex + 1);
			else FinishSequence(partial: false);
		}

		private void FinishSequence(bool partial)
		{
			m_SequenceDone = true;
			if (!MultiEpisode || m_Results.Count == 0) return;

			BenchmarkSummary summary = BenchmarkSummary.From(m_Results);
			Log.Info($"[Benchmark] {(partial ? "PARTIAL " : "")}Summary over {m_Results.Count}/{m_Seeds.Count} episodes " +
				$"(seeds {m_Seeds[0]}..{m_Seeds[^1]}):");
			foreach (string line in summary.Describe().Split('\n'))
				if (line.Length > 0) Log.Info("[Benchmark] " + line);

			if (m_Recorder == null) return;
			try
			{
				string path = m_Recorder.WriteSummary(summary, m_Seeds, partial);
				Log.Info($"[Record] Wrote {path}");
			}
			catch (Exception e) when (e is System.IO.IOException || e is UnauthorizedAccessException)
			{
				Log.Error($"[Record] Could not write the benchmark summary: {e.Message}");
			}
		}

		private void StartTask()
		{
			if (CurrentTask == null) return;
            //When task starts, Found by using the tags we have set in the editor, we find the robot, grasp site and the controller and gripper of the robot. If any of these are missing, we log an error and return.
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

            //Create new pandascript manipulator with the controller, gripper, grasp site and the left and right finger tags.
            var arm = new PandaManipulator(controller, gripper, graspSite, FindEntityByTag(LeftFingerTag), FindEntityByTag(RightFingerTag));
			if (ChooseGraspAxis && arm.FingerAxis == null && FindEntityByTag(LeftFingerTag) == null)
				Log.Warn($"[Episode] Finger entities '{LeftFingerTag}'/'{RightFingerTag}' not found; grasp-axis check disabled.");
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
				VaryRetries       = VaryRetries,
				ChooseGraspAxis   = ChooseGraspAxis,
				DeferFailedPicks  = DeferFailedPicks,
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
				if (m_Recorder == null)
				{
					string root = ResolveRecordingRoot();
					string session = DateTime.Now.ToString("yyyyMMdd_HHmmss");   // folder name only; never used for timing
					m_Recorder = new EpisodeRecorder(System.IO.Path.Combine(root, session), session);
					Log.Info($"[Record] Episode records -> {m_Recorder.SessionDirectory}");
				}
				m_Recorder.Begin(CurrentTask!, plan, StateMachine!);
			}

			if (RecordWithObserver)
			{
				m_Observer ??= new ObserverSession();
				if (m_Observer.BeginEpisode())
				{
					// Label frames per pick: entering MoveAboveObject is the start of each pick attempt.
					TaskStateMachine machine = StateMachine!;
					machine.StateChanged += (_, to) =>
					{
						if (to == TaskState.MoveAboveObject && machine.CurrentStep != null)
							m_Observer.SetStep(machine.CurrentStep);
					};
				}
			}
		}

		private void FinishRecording()
		{
			// Observer episode success is the evaluator's verdict: the dataset must not inherit the robot's opinion.
			bool last = m_EpisodeIndex + 1 >= m_Seeds.Count;
			m_Observer?.EndEpisode(Result!.Success, stopRecording: last);

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
			Vector3 mat = surface.Transform.WorldTranslation, matSize = surface.Transform.Scale;
			ws.MatMinX = mat.X - matSize.X * 0.5f;
			ws.MatMaxX = mat.X + matSize.X * 0.5f;
			ws.MatMinZ = mat.Z - matSize.Z * 0.5f;
			ws.MatMaxZ = mat.Z + matSize.Z * 0.5f;

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
					if (obj.Parent != null)
					{
						Log.Error($"[Episode] Pool object '{tag}' is parented under '{obj.Parent.Tag}' (entity {obj.Parent.ID}); " +
							"a child entity compiles into its parent's MuJoCo body and loses its own free joint. " +
							"Drag it back to the scene root before running.");
						return null;
					}
					m_Pool[tag] = obj;
					ws.EdgeLength[size] = obj.Transform.Scale.Y;   // physical size is the source of truth
				}
			}
			return ws;
		}

		// Pose of every pool object for a task: task objects at their generated start pose, the rest
		// parked in a row behind the robot, resting on the ground (y = 0).
		private Dictionary<string, (Vector3 Position, float Yaw)> ComputeLayout(TaskDefinition task)
		{
			var layout = new Dictionary<string, (Vector3, float)>();
			foreach (SortObjectDefinition def in task.Objects)
				layout[def.PoolTag] = (ToHazel(def.InitialPosition), def.InitialYaw);

			int slot = 0;
			foreach (var kv in m_Pool)
			{
				if (layout.ContainsKey(kv.Key)) continue;
				float edge = kv.Value.Transform.Scale.Y;
				layout[kv.Key] = (new Vector3(ParkingX, edge * 0.5f + 0.002f, -0.50f + slot * 0.07f), 0f);
				slot++;
			}
			return layout;
		}

		// The pool must be where the layout says (position and yaw modulo 90°) before the robot starts.
		// Guards both placement paths: transform writes at the start of Play and the in-Play reset.
		private bool CheckPlacement()
		{
			const float yawTolerance = 3f * Mathf.PI / 180f;
			var world = new SceneWorldState(m_Pool);
			int ok = 0;
			float worst = 0f;
			foreach (var kv in m_Intended)
			{
				Vector3 actual = kv.Key.Transform.WorldTranslation;
				float dx = actual.X - kv.Value.Position.X, dz = actual.Z - kv.Value.Position.Z;
				float horizontal = Mathf.Sqrt(dx * dx + dz * dz);
				float yawErr = Math.Abs(TaskStateMachine.WrapQuarter(world.GetObjectYaw(kv.Key.Tag) - kv.Value.Yaw));
				worst = Math.Max(worst, horizontal);
				if (horizontal <= PlacementTolerance && yawErr <= yawTolerance) { ok++; continue; }
				string before = m_EpisodeIndex > 0 && m_Resetter != null && m_Resetter.LastPreResetPositions.TryGetValue(kv.Key.Tag, out Vector3 b)
					? $", before the reset it was at ({b.X:F3}, {b.Y:F3}, {b.Z:F3})" : "";
				float intendedDeg = kv.Value.Yaw * 180f / Mathf.PI;
				float actualDeg = world.GetObjectYaw(kv.Key.Tag) * 180f / Mathf.PI;
				string rawQpos = m_Resetter != null && m_Resetter.TryGetRawQposYawDegrees(kv.Key.Tag, out float rawDeg)
					? $", raw qpos yaw {rawDeg:F1}°" : "";
				Log.Warn($"[Episode] {kv.Key.Tag}: intended ({kv.Value.Position.X:F3}, {kv.Value.Position.Y:F3}, {kv.Value.Position.Z:F3}) " +
					$"actual ({actual.X:F3}, {actual.Y:F3}, {actual.Z:F3}) off by {horizontal * 1000f:F1} mm, " +
					$"yaw off by {yawErr * 180f / Mathf.PI:F1}° (intended yaw {intendedDeg:F1}°, actual yaw {actualDeg:F1}°{rawQpos}){before}");
			}

			if (ok == m_Intended.Count)
			{
				Log.Info($"[Episode] Placement check passed: {ok}/{m_Intended.Count} objects within {PlacementTolerance * 1000f:F0} mm " +
					$"(worst {worst * 1000f:F1} mm) and 3° of yaw.");
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
