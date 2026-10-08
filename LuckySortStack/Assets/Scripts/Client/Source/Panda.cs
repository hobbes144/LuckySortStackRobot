using System;
using System.Collections.Generic;
using Hazel;

namespace LuckySortStack
{
	// Stage 0 smoke test: drives the Panda through motion-graph IK between two
	// waypoints, cycles the gripper, and logs grasp_site tracking error so the
	// IK frame/orientation conventions can be calibrated before real work starts.
	// Will be replaced by the modular RobotManipulator / TaskStateMachine.
	public class Panda : Entity
	{
		[Group("Waypoints")] [Units("m")]
		[Tooltip("Idle pose. Default is the grasp_site position in the Panda's rest pose.")]
		public Vector3 Home = new Vector3(0.55f, 0.52f, 0.0f);

		[Group("Waypoints")] [Units("m")]
		public Vector3 WaypointA = new Vector3(0.50f, 0.20f, -0.20f);

		[Group("Waypoints")] [Units("m")]
		public Vector3 WaypointB = new Vector3(0.50f, 0.20f, 0.20f);

		[Group("Grasp Test")]
		[Tooltip("Run a single pick/place of GraspObjectTag onto PlaceTargetTag instead of the waypoint loop.")]
		public bool RunGraspTest = true;

		[Group("Grasp Test")]
		public string GraspObjectTag = "LS_Obj_Red_Large";

		[Group("Grasp Test")]
		public string PlaceTargetTag = "LS_StackTarget_Red";

		[Group("Grasp Test")] [Units("m")] [Slider(0.03f, 0.30f)]
		[Tooltip("Height above the object/target centre for approach and retreat.")]
		public float HoverHeight = 0.12f;

		[Group("Grasp Test")] [Units("m")] [Slider(-0.03f, 0.03f)]
		[Tooltip("grasp_site offset from the object centre when grasping (0 = fingertips at centre).")]
		public float GraspOffsetY = 0.0f;

		[Group("Orientation")]
		[Tooltip("Whether the IK also solves for end-effector orientation.")]
		public bool SolveOrientation = true;

		[Group("Orientation")] [Units("deg")]
		[Tooltip("Gripper orientation (Euler, degrees). Default matches grasp_site in the rest pose (pointing down).")]
		public Vector3 TopDownOrientationDeg = new Vector3(90.0f, 90.0f, 0.0f);

		[Group("Timing")] [Units("s")] [Slider(0.5f, 10.0f)]
		public float MoveDuration = 2.0f;

		[Group("Timing")] [Units("s")] [Slider(0.0f, 5.0f)]
		public float StartupDelay = 1.0f;

		[Group("Gripper")]
		public string GripperActuatorName = "actuator8";

		[Group("Gripper")] [ClampValue(0f, 255f)]
		[Tooltip("panda.xml actuator8 ctrlrange [0,255] maps to finger travel 0-0.04 m; 255 = open.")]
		public float GripperOpenValue = 255f;

		[Group("Gripper")] [ClampValue(0f, 255f)]
		public float GripperCloseValue = 0f;

		[Group("Gripper")] [Units("s")] [Slider(0.1f, 3.0f)]
		public float GripperSettle = 1.0f;

		private static readonly Identifier k_Pos         = new Identifier("Target_Position");
		private static readonly Identifier k_Rot         = new Identifier("Target_Orientation");
		private static readonly Identifier k_SolveRot    = new Identifier("Solve_Orientation");
		private static readonly Identifier k_Dur         = new Identifier("Duration");
		private static readonly Identifier k_Go          = new Identifier("Start");

		[HideFromEditor] private RobotControllerComponent? m_Robot;
		[HideFromEditor] private MujocoActuator? m_Gripper;
		[HideFromEditor] private Entity? m_GraspSite;

		private struct Step { public string Label; public Action? OnStart; public Action? OnEnd; public float Duration; }
		[HideFromEditor] private readonly List<Step> m_Cycle = new List<Step>();
		[HideFromEditor] private int m_StepIndex;
		[HideFromEditor] private float m_StepElapsed;
		[HideFromEditor] private bool m_StepStarted;
		[HideFromEditor] private float m_StartupTimer;
		[HideFromEditor] private Vector3 m_CurrentTarget;

		protected override void OnCreate()
		{
			m_Robot     = GetComponent<RobotControllerComponent>();
			m_GraspSite = FindEntityByTag("grasp_site");

			MujocoSceneComponent? mujoco = GetComponent<MujocoSceneComponent>();
			m_Gripper = mujoco?.GetActuator(GripperActuatorName);
			if (m_Gripper != null)
			{
				m_Gripper.IK    = false;
				m_Gripper.Value = GripperOpenValue;
			}

			if (m_Robot == null)     Log.Error("[Panda] No RobotControllerComponent on this entity.");
			if (m_Gripper == null)   Log.Error($"[Panda] Gripper actuator '{GripperActuatorName}' not found.");
			if (m_GraspSite == null) Log.Warn("[Panda] No 'grasp_site' entity found; tracking error will not be logged.");

			BuildCycle();
			m_StartupTimer = StartupDelay;
			Log.Info($"[Panda] Smoke test ready: {m_Cycle.Count} steps per cycle.");
		}

		protected override void OnUpdate(float ts)
		{
			if (m_StartupTimer > 0f) { m_StartupTimer -= ts; return; }
			if (m_Cycle.Count == 0) return;

			Step step = m_Cycle[m_StepIndex];
			if (!m_StepStarted)
			{
				step.OnStart?.Invoke();
				m_StepStarted = true;
				m_StepElapsed = 0f;
			}

			m_StepElapsed += ts;
			if (m_StepElapsed < step.Duration) return;

			step.OnEnd?.Invoke();
			m_StepStarted = false;
			m_StepIndex++;
			if (m_StepIndex < m_Cycle.Count) return;

			if (RunGraspTest)
			{
				Log.Info("[Panda] Grasp test finished.");
				m_Cycle.Clear();
				return;
			}
			m_StepIndex = 0;
		}

		private void BuildCycle()
		{
			if (RunGraspTest)
			{
				BuildGraspTest();
				return;
			}

			AddMove("Home", Home);
			AddMove("A", WaypointA);
			AddGripper("Close", GripperCloseValue);
			AddMove("B", WaypointB);
			AddGripper("Open", GripperOpenValue);
		}

		// Stage 1 check: one object, picked from the pool and set down on a stack marker.
		// Positions are read once at OnCreate, before physics settles; good enough
		// for a resting cube, and replaced by live state reads in the real controller.
		private void BuildGraspTest()
		{
			Entity? obj    = FindEntityByTag(GraspObjectTag);
			Entity? target = FindEntityByTag(PlaceTargetTag);
			if (obj == null || target == null)
			{
				Log.Error($"[Panda] Grasp test needs '{GraspObjectTag}' and '{PlaceTargetTag}' in the scene.");
				return;
			}

			Vector3 objPos    = obj.Transform.WorldTranslation;
			float   halfEdge  = obj.Transform.Scale.Y * 0.5f;
			Vector3 targetTop = target.Transform.WorldTranslation + new Vector3(0f, target.Transform.Scale.Y * 0.5f, 0f);
			Vector3 grasp     = new Vector3(objPos.X, objPos.Y + GraspOffsetY, objPos.Z);
			Vector3 place     = new Vector3(targetTop.X, targetTop.Y + halfEdge + 0.005f + GraspOffsetY, targetTop.Z);
			Vector3 lift      = new Vector3(0f, HoverHeight, 0f);

			AddGripper("Open", GripperOpenValue);
			AddMove("Home", Home);
			AddMove("AboveObject", grasp + lift);
			AddMove("Grasp", grasp);
			AddGripper("Close", GripperCloseValue);
			AddMove("Lift", grasp + lift);
			AddMove("AboveTarget", place + lift);
			AddMove("Place", place);
			AddGripper("Release", GripperOpenValue);
			AddMove("Retreat", place + lift);
			AddMove("Home", Home);
			m_Cycle.Add(new Step { Label = "Report", OnStart = () => ReportObject(obj, targetTop), Duration = 0f });
		}

		private void ReportObject(Entity obj, Vector3 targetTop)
		{
			Vector3 p = obj.Transform.WorldTranslation;
			float horizontalMm = Mathf.Sqrt((p.X - targetTop.X) * (p.X - targetTop.X) + (p.Z - targetTop.Z) * (p.Z - targetTop.Z)) * 1000f;
			Log.Info($"[Panda] {obj.Tag} final={p} offset from target centre={horizontalMm:F1} mm, bottom height={(p.Y - obj.Transform.Scale.Y * 0.5f) * 1000f:F1} mm");
		}

		// One SetInputTrigger per step: each trigger cancels any in-flight motion.
		private void AddMove(string label, Vector3 target)
		{
			m_Cycle.Add(new Step
			{
				Label    = label,
				OnStart  = () => SendTarget(target),
				OnEnd    = () => LogTracking(label),
				Duration = MoveDuration + 0.25f,
			});
		}

		private void AddGripper(string label, float value)
		{
			m_Cycle.Add(new Step
			{
				Label    = label,
				OnStart  = () => { if (m_Gripper != null) m_Gripper.Value = value; },
				Duration = GripperSettle,
			});
		}

		private void SendTarget(Vector3 target)
		{
			if (m_Robot == null) return;

			m_CurrentTarget = target;
			m_Robot.SetInputBool   (k_SolveRot, SolveOrientation);
			m_Robot.SetInputVector3(k_Pos, target);
			m_Robot.SetInputVector3(k_Rot, TopDownOrientationDeg);
			m_Robot.SetInputFloat  (k_Dur, MoveDuration);
			m_Robot.SetInputTrigger(k_Go);
		}

		private void LogTracking(string label)
		{
			if (m_GraspSite == null) return;

			Vector3 actual = m_GraspSite.Transform.WorldTranslation;
			Vector3 rotDeg = m_GraspSite.Transform.WorldRotation * Mathf.Rad2Deg;
			float errorMm  = Vector3.Distance(actual, m_CurrentTarget) * 1000f;
			Log.Info($"[Panda] {label}: target={m_CurrentTarget} actual={actual} error={errorMm:F1} mm rot(deg)={rotDeg}");
		}
	}
}
