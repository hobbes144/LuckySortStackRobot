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
			m_StepIndex   = (m_StepIndex + 1) % m_Cycle.Count;
			m_StepStarted = false;
		}

		private void BuildCycle()
		{
			AddMove("Home", Home);
			AddMove("A", WaypointA);
			AddGripper("Close", GripperCloseValue);
			AddMove("B", WaypointB);
			AddGripper("Open", GripperOpenValue);
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
