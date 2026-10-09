using System;
using System.Collections.Generic;
using System.Numerics;
using LuckySortStack.Core;
using LuckySortStack.Planning;
using LuckySortStack.Robot;

namespace LuckySortStack.StateMachine
{
	public enum TaskState
	{
		Idle,
		SelectTarget,
		MoveAboveObject,
		MoveToObject,
		Grasp,
		Lift,
		VerifyGrasp,       // after Lift: the physical evidence of a grasp is the object rising with the hand
		MoveToStack,
		Place,
		Release,
		Retreat,
		VerifyPlacement,
		Recovery,
		ReturnHome,
		Success,
		Failure,
	}

	// Tunables, in metres / seconds / radians. Defaults are a starting point for the Panda.
	public sealed class MotionSettings
	{
		public Vector3 Home = new Vector3(0.55f, 0.52f, 0.0f);
		public float TravelHeight = 0.28f;          // grasp-site height for all lateral moves
		public float TravelSpeed = 0.40f;
		public float DescendSpeed = 0.15f;
		public float PlaceDescendSpeed = 0.08f;
		public float MaxYawSpeed = 1.5f;
		public float MinMoveDuration = 0.3f;
		public float MoveTolerance = 0.015f;        // grasp site must get this close to finish a move
		public float MoveTimeoutSlack = 2.0f;       // extra time past the planned duration before "unreachable"

		public float GraspOffsetY = 0.0f;           // grasp-site height relative to object centre
		public float PlaceGap = 0.003f;             // release this far above resting contact
		public float GripperCloseTime = 0.8f;
		public float ReleaseRampTime = 0.15f;       // open gradually to avoid flicking the object
		public float GripperReleaseTime = 0.6f;

		public float SettleSpeed = 0.02f;           // m/s below which an object counts as still
		public float SettleWindow = 0.3f;           // must stay still this long
		public float SettleTimeout = 3.0f;
		public float PlaceToleranceXZ = 0.015f;
		public float PlaceToleranceY = 0.010f;

		public int MaxPickAttempts = 3;             // per object
		public int MaxPlaceAttempts = 2;            // per object
		public float EpisodeTimeout = 600f;
	}

	// Explicit, debuggable pick/place loop. Every state has an entry action, an
	// update with a completion condition, a timeout and a failure path. Engine-free:
	// the arm and the world come in through IManipulator / IWorldState, and time
	// only advances through Update(dt).
	public sealed class TaskStateMachine
	{
		public TaskState State { get; private set; } = TaskState.Idle;
		public EpisodeMetrics Metrics { get; } = new EpisodeMetrics();
		public PlannedStep? CurrentStep => m_StepIndex < m_Steps.Count ? m_Steps[m_StepIndex] : null;
		public int StepIndex => m_StepIndex;
		public IReadOnlyList<PlannedStep> Steps => m_Steps;
		public bool IsFinished => State == TaskState.Success || State == TaskState.Failure;

		public event Action<string>? Log;
		public event Action<TaskState, TaskState>? StateChanged;

		private readonly IManipulator m_Arm;
		private readonly IWorldState m_World;
		private readonly MotionSettings m_Cfg;
		private readonly List<PlannedStep> m_Steps;

		// Verified placements per stack (bottom → top) with the position recorded at verification.
		private readonly Dictionary<int, List<(SortObjectDefinition Obj, Vector3 Position)>> m_Placed = new();

		private int m_StepIndex;
		private float m_StateTime;
		private int m_PickAttempts;
		private int m_PlaceAttempts;
		private FailureReason m_RecoveryReason;

		private Vector3 m_MoveTarget;
		private float m_MoveDuration;
		private float m_CommandedYaw;
		private float m_GraspYaw;
		private Vector3 m_PlaceCentre;

		private Vector3 m_SettleLastPos;
		private float m_SettleStillTime;

		public TaskStateMachine(TaskDefinition task, IReadOnlyList<PlannedStep> steps, IManipulator arm, IWorldState world, MotionSettings settings)
		{
			m_Arm   = arm;
			m_World = world;
			m_Cfg   = settings;
			m_Steps = new List<PlannedStep>(steps);
			for (int i = 0; i < task.Targets.Count; i++) m_Placed[i] = new List<(SortObjectDefinition, Vector3)>();
		}

		public void Start()
		{
			if (State != TaskState.Idle) return;
			TransitionTo(TaskState.SelectTarget);
		}

		public void Update(float dt)
		{
			if (State == TaskState.Idle || IsFinished) return;

			Metrics.Duration += dt;
			m_StateTime += dt;
			if (Metrics.Duration > m_Cfg.EpisodeTimeout)
			{
				Fail(FailureReason.Timeout, $"episode exceeded {m_Cfg.EpisodeTimeout:F0}s in {State}");
				return;
			}

			switch (State)
			{
				case TaskState.SelectTarget:    UpdateSelectTarget(); break;
				case TaskState.MoveAboveObject: OnMoveDone(TaskState.MoveToObject, FailureReason.Unreachable); break;
				case TaskState.MoveToObject:    OnMoveDone(TaskState.Grasp, FailureReason.Unreachable); break;
				case TaskState.Grasp:           if (m_StateTime >= m_Cfg.GripperCloseTime) TransitionTo(TaskState.Lift); break;
				case TaskState.Lift:            OnMoveDone(TaskState.VerifyGrasp, FailureReason.Unreachable); break;
				case TaskState.VerifyGrasp:     UpdateVerifyGrasp(); break;
				case TaskState.MoveToStack:     OnMoveDone(TaskState.Place, FailureReason.Unreachable); break;
				case TaskState.Place:           UpdatePlace(); break;
				case TaskState.Release:         UpdateRelease(); break;
				case TaskState.Retreat:         OnMoveDone(TaskState.VerifyPlacement, FailureReason.Unreachable); break;
				case TaskState.VerifyPlacement: UpdateVerifyPlacement(dt); break;
				case TaskState.Recovery:        UpdateRecovery(); break;
				case TaskState.ReturnHome:      UpdateReturnHome(); break;
			}
		}

		// ── Transitions ─────────────────────────────────────────────────────

		private void TransitionTo(TaskState next)
		{
			TaskState prev = State;
			State = next;
			m_StateTime = 0f;
			StateChanged?.Invoke(prev, next);
			Enter(next);
		}

		private void Enter(TaskState state)
		{
			PlannedStep? step = CurrentStep;
			switch (state)
			{
				case TaskState.MoveAboveObject:
				{
					m_Arm.SetGripper(1f);
					Vector3 p = ObjectPos(step!);
					m_GraspYaw = NearestCubeYaw(m_World.GetObjectYaw(step!.Object.PoolTag), m_CommandedYaw);
					StartMove(new Vector3(p.X, m_Cfg.TravelHeight, p.Z), m_GraspYaw, m_Cfg.TravelSpeed);
					break;
				}
				case TaskState.MoveToObject:
				{
					m_PickAttempts++;
					Metrics.PickAttempts++;
					Vector3 p = ObjectPos(step!);   // re-read: the object may have shifted
					StartMove(new Vector3(p.X, p.Y + m_Cfg.GraspOffsetY, p.Z), m_GraspYaw, m_Cfg.DescendSpeed);
					break;
				}
				case TaskState.Grasp:
					m_Arm.SetGripper(0f);
					break;
				case TaskState.Lift:
					StartMove(new Vector3(m_MoveTarget.X, m_Cfg.TravelHeight, m_MoveTarget.Z), m_GraspYaw, m_Cfg.DescendSpeed);
					break;
				case TaskState.MoveToStack:
				{
					m_PlaceCentre = StackPlacement.PlaceCentre(step!.Stack, LivePlaced(step.StackIndex), step.Object.Edge, m_Cfg.PlaceGap);
					// Stack yaw is 0: rotate the held cube square to the stack on the way.
					StartMove(new Vector3(m_PlaceCentre.X, m_Cfg.TravelHeight, m_PlaceCentre.Z), 0f, m_Cfg.TravelSpeed);
					break;
				}
				case TaskState.Place:
					m_PlaceAttempts++;
					Metrics.PlaceAttempts++;
					StartMove(m_PlaceCentre + new Vector3(0f, m_Cfg.GraspOffsetY, 0f), 0f, m_Cfg.PlaceDescendSpeed);
					break;
				case TaskState.Retreat:
					StartMove(new Vector3(m_PlaceCentre.X, m_Cfg.TravelHeight, m_PlaceCentre.Z), 0f, m_Cfg.DescendSpeed);
					break;
				case TaskState.VerifyPlacement:
					m_SettleLastPos = ObjectPos(step!);
					m_SettleStillTime = 0f;
					break;
				case TaskState.Recovery:
				{
					m_Arm.SetGripper(1f);
					Vector3 g = m_Arm.GripperPosition;
					StartMove(new Vector3(g.X, Math.Max(g.Y, m_Cfg.TravelHeight), g.Z), m_CommandedYaw, m_Cfg.DescendSpeed);
					break;
				}
				case TaskState.ReturnHome:
					m_Arm.SetGripper(1f);
					StartMove(m_Cfg.Home, 0f, m_Cfg.TravelSpeed);
					break;
			}
		}

		// ── State updates ───────────────────────────────────────────────────

		private void UpdateSelectTarget()
		{
			if (m_StepIndex >= m_Steps.Count)
			{
				TransitionTo(TaskState.ReturnHome);
				return;
			}
			m_PickAttempts = 0;
			m_PlaceAttempts = 0;
			Emit($"Step {m_StepIndex + 1}/{m_Steps.Count}: {CurrentStep}");
			TransitionTo(TaskState.MoveAboveObject);
		}

		private void UpdateVerifyGrasp()
		{
			PlannedStep step = CurrentStep!;
			float dist = Vector3.Distance(ObjectPos(step), m_Arm.GripperPosition);
			float limit = 0.6f * step.Object.Edge + 0.01f;
			if (dist <= limit)
			{
				TransitionTo(TaskState.MoveToStack);
				return;
			}
			Metrics.GraspFailures++;
			Emit($"Grasp failed: {step.Object.PoolTag} is {dist * 1000f:F0} mm from the gripper (limit {limit * 1000f:F0} mm)");
			BeginRecovery(FailureReason.GraspFailed);
		}

		private void UpdatePlace()
		{
			MoveStatus status = CheckMove();
			if (status == MoveStatus.Running) return;
			// A timeout here usually means the held object is resting on the stack before the
			// grasp site converged. Releasing is the right thing either way; VerifyPlacement judges.
			if (status == MoveStatus.TimedOut)
				Emit($"Place descent stopped {Vector3.Distance(m_Arm.GripperPosition, m_MoveTarget) * 1000f:F0} mm short; releasing");
			TransitionTo(TaskState.Release);
		}

		private void UpdateRelease()
		{
			m_Arm.SetGripper(Math.Min(1f, m_StateTime / m_Cfg.ReleaseRampTime));
			if (m_StateTime >= m_Cfg.GripperReleaseTime) TransitionTo(TaskState.Retreat);
		}

		private void UpdateVerifyPlacement(float dt)
		{
			PlannedStep step = CurrentStep!;
			Vector3 p = ObjectPos(step);
			float speed = dt > 0f ? Vector3.Distance(p, m_SettleLastPos) / dt : 0f;
			m_SettleLastPos = p;
			m_SettleStillTime = speed < m_Cfg.SettleSpeed ? m_SettleStillTime + dt : 0f;

			bool settled = m_SettleStillTime >= m_Cfg.SettleWindow;
			if (!settled && m_StateTime < m_Cfg.SettleTimeout) return;

			// Objects already verified on this stack must still be there.
			foreach (var (obj, recorded) in m_Placed[step.StackIndex])
			{
				float drift = HorizontalDistance(m_World.GetObjectPosition(obj.PoolTag), recorded);
				if (drift > m_Cfg.PlaceToleranceXZ * 2f)
				{
					Fail(FailureReason.StackCollapsed, $"{obj.PoolTag} moved {drift * 1000f:F0} mm off the {step.Stack.Color} stack");
					return;
				}
			}

			float restY = m_PlaceCentre.Y - m_Cfg.PlaceGap;
			float errXZ = HorizontalDistance(p, m_PlaceCentre);
			float errY  = Math.Abs(p.Y - restY);
			if (settled && errXZ <= m_Cfg.PlaceToleranceXZ && errY <= m_Cfg.PlaceToleranceY)
			{
				m_Placed[step.StackIndex].Add((step.Object, p));
				Metrics.ObjectsPlaced++;
				m_StepIndex++;
				TransitionTo(TaskState.SelectTarget);
				return;
			}

			Metrics.PlacementFailures++;
			Emit($"Placement failed: {step.Object.PoolTag} off by {errXZ * 1000f:F0} mm horizontally, {errY * 1000f:F0} mm vertically" +
				(settled ? "" : ", still moving"));
			BeginRecovery(FailureReason.PlacementFailed);
		}

		private void BeginRecovery(FailureReason reason)
		{
			m_RecoveryReason = reason;
			TransitionTo(TaskState.Recovery);
		}

		// Open, back off upward, then retry from the object's live position if attempts remain.
		private void UpdateRecovery()
		{
			MoveStatus status = CheckMove();
			if (status == MoveStatus.Running) return;
			if (status == MoveStatus.TimedOut)
			{
				Fail(FailureReason.Unreachable, "could not back off during recovery");
				return;
			}

			string tag = CurrentStep!.Object.PoolTag;
			if (m_RecoveryReason == FailureReason.GraspFailed && m_PickAttempts >= m_Cfg.MaxPickAttempts)
			{
				Fail(FailureReason.GraspFailed, $"{tag} not grasped after {m_PickAttempts} attempts");
				return;
			}
			if (m_RecoveryReason == FailureReason.PlacementFailed && m_PlaceAttempts >= m_Cfg.MaxPlaceAttempts)
			{
				Fail(FailureReason.PlacementFailed, $"{tag} not placed after {m_PlaceAttempts} attempts");
				return;
			}
			Emit($"Retrying {tag}");
			TransitionTo(TaskState.MoveAboveObject);
		}

		private void UpdateReturnHome()
		{
			// Going home is cosmetic: the stacks were already verified, so don't fail the episode on it.
			if (CheckMove() == MoveStatus.Running) return;
			Emit($"All {m_Steps.Count} steps complete. {Metrics}");
			TransitionTo(TaskState.Success);
		}

		private void OnMoveDone(TaskState next, FailureReason onTimeout)
		{
			MoveStatus status = CheckMove();
			if (status == MoveStatus.Done) TransitionTo(next);
			else if (status == MoveStatus.TimedOut)
				Fail(onTimeout, $"{State}: grasp site {Vector3.Distance(m_Arm.GripperPosition, m_MoveTarget) * 1000f:F0} mm from target after {m_StateTime:F1}s");
		}

		private void Fail(FailureReason reason, string detail)
		{
			Metrics.Failure = reason;
			Metrics.FailureDetail = detail;
			Emit($"FAILED ({reason}): {detail}. {Metrics}");
			TransitionTo(TaskState.Failure);
		}

		// ── Motion helpers ──────────────────────────────────────────────────

		private enum MoveStatus { Running, Done, TimedOut }

		private void StartMove(Vector3 target, float yaw, float speed)
		{
			float dist = Vector3.Distance(m_Arm.GripperPosition, target);
			float turn = Math.Abs(WrapPi(yaw - m_CommandedYaw));
			m_MoveDuration = Math.Max(m_Cfg.MinMoveDuration, Math.Max(dist / Math.Max(speed, 1e-3f), turn / Math.Max(m_Cfg.MaxYawSpeed, 1e-3f)));
			m_MoveTarget = target;
			m_CommandedYaw = yaw;
			m_Arm.MoveTo(target, yaw, m_MoveDuration);
		}

		private MoveStatus CheckMove()
		{
			if (m_StateTime < m_MoveDuration) return MoveStatus.Running;
			if (Vector3.Distance(m_Arm.GripperPosition, m_MoveTarget) <= m_Cfg.MoveTolerance) return MoveStatus.Done;
			return m_StateTime > m_MoveDuration + m_Cfg.MoveTimeoutSlack ? MoveStatus.TimedOut : MoveStatus.Running;
		}

		private List<(Vector3, float)> LivePlaced(int stackIndex)
		{
			var live = new List<(Vector3, float)>();
			foreach (var (obj, _) in m_Placed[stackIndex])
				live.Add((m_World.GetObjectPosition(obj.PoolTag), obj.Edge));
			return live;
		}

		private Vector3 ObjectPos(PlannedStep step) => m_World.GetObjectPosition(step.Object.PoolTag);

		private void Emit(string message) => Log?.Invoke(message);

		// A cube looks the same every 90°, so pick the equivalent yaw closest to the current gripper yaw.
		public static float NearestCubeYaw(float objectYaw, float currentYaw)
		{
			const float quarter = MathF.PI / 2f;
			float rel = objectYaw - currentYaw;
			return currentYaw + rel - quarter * MathF.Round(rel / quarter);
		}

		private static float WrapPi(float a)
		{
			while (a > MathF.PI) a -= 2f * MathF.PI;
			while (a < -MathF.PI) a += 2f * MathF.PI;
			return a;
		}

		private static float HorizontalDistance(Vector3 a, Vector3 b) =>
			MathF.Sqrt((a.X - b.X) * (a.X - b.X) + (a.Z - b.Z) * (a.Z - b.Z));
	}
}
