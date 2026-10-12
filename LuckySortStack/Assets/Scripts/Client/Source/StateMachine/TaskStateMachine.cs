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
		AlignGripper,      // measure the real gripper yaw and correct it before descending
		CenterOverObject,  // drop to just above the object and cancel the horizontal tracking error
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
		public float MoveTimeoutSlack = 2.0f;       // extra time past the planned duration before giving up

		// Yaw calibration. Commanded yaw maps to physical yaw as  physical = sign * command + offset  (mod 90°).
		// YawSign is only the initial guess; sign and offset are learned from measurements.
		public bool ClosedLoopYaw = true;
		public float YawSign = 1f;
		public float YawTolerance = 6f * MathF.PI / 180f;
		public float AlignDwell = 0.25f;            // let the wrist settle before measuring
		public int MaxAlignIterations = 4;

		// Position calibration before the final descent. The arm settles with a few mm of tracking error
		// that varies with its configuration, and a Large cube leaves only ~10 mm per finger.
		public float PreGraspClearance = 0.04f;     // grasp-site height above the object's top while centring
		public float CenterTolerance = 0.003f;      // horizontal grasp-site error allowed before descending
		public int MaxCenterIterations = 3;

		// Retries change the approach instead of repeating it. In the 50-seed benchmark every failed retry logged the
		// same numbers as the attempt before it (same 50 mm hover error, same 8 mm push): the failures depended on the
		// arm configuration, and an identical retry reproduces the identical configuration. A cube is symmetric under
		// 90° turns, so the grasp can be turned ±90° for free, which puts the wrist (and the IK solution, and its
		// tracking error) somewhere else. After a push, the centring hover is also lowered so less drift accumulates
		// between the last measurement and the fingers reaching the cube.
		public bool VaryRetries = true;
		public float RetryPreGraspClearance = 0.02f;   // centring hover height on a retry after a collision

		// Grasp-axis choice. A cube can be gripped across either pair of faces. The open fingers reach ~50 mm from the
		// cube's centre, further than the generator's spacing guarantees, so a neighbour can sit where a finger comes
		// down (50-seed benchmark: all 3 failures were a finger landing on a Large neighbour ~91 mm away). Before each
		// pick, score both axes by the clearance between the finger tips and every other cube, and grip across the
		// clearer one. The real finger axis is measured after alignment and corrected by 90° if it's the wrong one.
		public bool ChooseGraspAxis = true;
		public float FingerReach = 0.05f;            // finger centre distance from the grasp centre when open
		public float FingerHalfWidth = 0.012f;       // finger footprint radius
		public float AxisClearEnough = 0.03f;        // both axes at least this clear → don't constrain (no extra wrist turn)

		public float GraspOffsetY = 0.0f;           // grasp-site height relative to object centre
		public float PushTolerance = 0.008f;        // object moved this far during descent = collision
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
		public float LearnedYawSign => m_YawSign;
		public float LearnedYawOffset => m_YawOffset;

		// Read-only views for the presentation layer (overlay, markers). Nothing here changes behaviour.
		public Vector3 MoveTarget => m_MoveTarget;
		public Vector3? WantedFingerAxis => m_WantedFingerAxis;
		public int CurrentPickAttempt => m_PickAttempts;
		public int MaxPickAttempts => m_Cfg.MaxPickAttempts;
		public float FingerReach => m_Cfg.FingerReach;

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
		private float m_MoveSpeed;
		private float m_CommandedYaw;
		private Vector3 m_PlaceCentre;
		private Vector3 m_ObjectAtDescent;
		private Vector3 m_PosCorrection;            // added to pick targets to cancel the measured tracking error
		private int m_CenterIteration;
		private float m_RetryYaw;                   // extra grasp yaw for this pick attempt: 0, +90°, −90°, …
		private float m_HoverClearance;             // centring hover height for this pick attempt
		private Vector3? m_WantedFingerAxis;        // horizontal axis the fingers should close along for this pick
		private bool m_AxisCorrected;               // one 90° axis correction per approach
		private Vector3? m_LastFingerAxis;          // measured finger axis of the previous attempt on this object
		private readonly List<SortObjectDefinition> m_Objects;

		private float m_YawSign;
		private float m_YawOffset;
		private bool m_YawCalibrated;
		private float m_AlignDesired;
		private TaskState m_AlignNext;
		private int m_AlignIteration;
		private float m_AlignPrevError;

		private Vector3 m_SettleLastPos;
		private float m_SettleStillTime;

		public TaskStateMachine(TaskDefinition task, IReadOnlyList<PlannedStep> steps, IManipulator arm, IWorldState world, MotionSettings settings)
		{
			m_Arm   = arm;
			m_World = world;
			m_Cfg   = settings;
			m_Steps = new List<PlannedStep>(steps);
			m_YawSign = settings.YawSign >= 0f ? 1f : -1f;
			m_HoverClearance = settings.PreGraspClearance;
			m_Objects = new List<SortObjectDefinition>(task.Objects);
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
				case TaskState.MoveAboveObject: UpdateMoveAboveObject(); break;
				case TaskState.AlignGripper:    UpdateAlignGripper(); break;
				case TaskState.CenterOverObject: UpdateCenterOverObject(); break;
				case TaskState.MoveToObject:    UpdateMoveToObject(); break;
				case TaskState.Grasp:           if (m_StateTime >= m_Cfg.GripperCloseTime) TransitionTo(TaskState.Lift); break;
				case TaskState.Lift:            OnMoveDone(TaskState.VerifyGrasp); break;
				case TaskState.VerifyGrasp:     UpdateVerifyGrasp(); break;
				case TaskState.MoveToStack:     UpdateMoveToStack(); break;
				case TaskState.Place:           UpdatePlace(); break;
				case TaskState.Release:         UpdateRelease(); break;
				case TaskState.Retreat:         OnMoveDone(TaskState.VerifyPlacement); break;
				case TaskState.VerifyPlacement: UpdateVerifyPlacement(dt); break;
				case TaskState.Recovery:        UpdateRecovery(); break;
				case TaskState.ReturnHome:      UpdateReturnHome(); break;
			}
		}

        // ── Transitions ─────────────────────────────────────────────────────
        #region Transitions
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
					// Every approach is a pick attempt, so a target that can never be reached still runs out of retries.
					m_PickAttempts++;
					Metrics.PickAttempts++;
					m_Arm.SetGripper(1f);
					m_PosCorrection = Vector3.Zero;
					m_ObjectAtDescent = ObjectPos(step!);
					ChooseApproach(m_PickAttempts - 1);
					ChooseGraspAxis(step!, m_PickAttempts - 1);
					StartMove(new Vector3(m_ObjectAtDescent.X, m_Cfg.TravelHeight, m_ObjectAtDescent.Z), CommandFor(ObjectYaw(step!)) + m_RetryYaw, m_Cfg.TravelSpeed);
					break;
				}
				case TaskState.CenterOverObject:
					m_CenterIteration = 0;
					StartMove(PreGraspPoint(step!) + m_PosCorrection, m_CommandedYaw, m_Cfg.DescendSpeed);
					break;
				case TaskState.MoveToObject:
				{
					m_ObjectAtDescent = ObjectPos(step!);   // re-read: the object may have shifted
					StartMove(m_ObjectAtDescent + new Vector3(0f, m_Cfg.GraspOffsetY, 0f) + m_PosCorrection, m_CommandedYaw, m_Cfg.DescendSpeed);
					break;
				}
				case TaskState.Grasp:
					m_Arm.SetGripper(0f);
					break;
				case TaskState.Lift:
					StartMove(new Vector3(m_MoveTarget.X, m_Cfg.TravelHeight, m_MoveTarget.Z), m_CommandedYaw, m_Cfg.DescendSpeed);
					break;
				case TaskState.MoveToStack:
					m_PlaceCentre = StackPlacement.PlaceCentre(step!.Stack, LivePlaced(step.StackIndex), step.Object.Edge, m_Cfg.PlaceGap);
					StartMove(new Vector3(m_PlaceCentre.X, m_Cfg.TravelHeight, m_PlaceCentre.Z), CommandFor(StackYaw(step)), m_Cfg.TravelSpeed);
					break;
				case TaskState.Place:
					m_PlaceAttempts++;
					Metrics.PlaceAttempts++;
					StartMove(m_PlaceCentre + new Vector3(0f, m_Cfg.GraspOffsetY, 0f), m_CommandedYaw, m_Cfg.PlaceDescendSpeed);
					break;
				case TaskState.Retreat:
					StartMove(new Vector3(m_PlaceCentre.X, m_Cfg.TravelHeight, m_PlaceCentre.Z), m_CommandedYaw, m_Cfg.DescendSpeed);
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
					StartMove(m_Cfg.Home, CommandFor(0f), m_Cfg.TravelSpeed);
					break;
			}
		}
        #endregion

        // ── State updates ───────────────────────────────────────────────────
        #region State Updates
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

		private void UpdateMoveAboveObject()
		{
			MoveStatus status = CheckMove();
			if (status == MoveStatus.Running) return;
			if (status == MoveStatus.TimedOut) { MoveFailed("approach"); return; }
			BeginAlign(ObjectYaw(CurrentStep!), TaskState.CenterOverObject);
		}

		private void UpdateMoveToStack()
		{
			MoveStatus status = CheckMove();
			if (status == MoveStatus.Running) return;
			if (status == MoveStatus.TimedOut) { MoveFailed("move to stack"); return; }
			BeginAlign(StackYaw(CurrentStep!), TaskState.Place);
		}

		// Physical yaw must match the target modulo 90° (cube symmetry). Each correction also
		// refines the learned command→physical mapping; if a correction made things worse,
		// the sign guess was wrong and is flipped.
		private void BeginAlign(float desiredPhysicalYaw, TaskState next)
		{
			m_AlignDesired = desiredPhysicalYaw;
			m_AlignNext = next;
			m_AlignIteration = 0;
			m_MoveDuration = 0f;
			TransitionTo(TaskState.AlignGripper);
		}

		private void UpdateAlignGripper()
		{
			if (!m_Cfg.ClosedLoopYaw)
			{
				TransitionTo(m_AlignNext);
				return;
			}

			MoveStatus status = CheckMove();
			if (status == MoveStatus.TimedOut) { MoveFailed("gripper alignment"); return; }
			if (status == MoveStatus.Running || m_StateTime < m_MoveDuration + m_Cfg.AlignDwell) return;

			float measured = m_Arm.GripperYaw;
			float error = WrapQuarter(measured - m_AlignDesired);
			if (!m_YawCalibrated)
			{
				Emit($"Yaw check: commanded {Deg(m_CommandedYaw)}°, measured {Deg(measured)}°, wanted {Deg(m_AlignDesired)}° (mod 90), error {Deg(error)}°");
			}

			if (MathF.Abs(error) <= m_Cfg.YawTolerance)
			{
				m_YawCalibrated = true;
				if (m_AlignNext == TaskState.CenterOverObject)
				{
					if (CorrectFingerAxis()) return;
					m_LastFingerAxis = m_Arm.FingerAxis;
				}
				TransitionTo(m_AlignNext);
				return;
			}

			if (m_AlignIteration >= m_Cfg.MaxAlignIterations)
			{
				Emit($"Gripper still {Deg(error)}° off after {m_AlignIteration} corrections; continuing");
				TransitionTo(m_AlignNext);
				return;
			}

			if (m_AlignIteration > 0 && MathF.Abs(error) > MathF.Abs(m_AlignPrevError) - 0.5f * m_Cfg.YawTolerance)
			{
				m_YawSign = -m_YawSign;
				Emit($"Yaw correction didn't help ({Deg(m_AlignPrevError)}° → {Deg(error)}°); flipping yaw sign to {m_YawSign:+0;-0}");
			}
			m_YawOffset = WrapQuarter(measured - m_YawSign * m_CommandedYaw);
			m_AlignPrevError = error;
			m_AlignIteration++;

			float retryYaw = m_AlignNext == TaskState.CenterOverObject ? m_RetryYaw : 0f;   // keep the turned grasp while correcting
			StartMove(m_MoveTarget, CommandFor(m_AlignDesired) + retryYaw, m_MoveSpeed);
			m_StateTime = 0f;
		}

		// Pick the finger axis for this approach: on the first try (and every even retry) the axis with more clearance to
		// the neighbours, on odd retries the other one (a varied retry). Scoring uses live positions of every task cube.
		private void ChooseGraspAxis(PlannedStep step, int retry)
		{
			m_WantedFingerAxis = null;
			m_AxisCorrected = false;
			if (retry == 0) m_LastFingerAxis = null;
			if (!m_Cfg.ChooseGraspAxis) return;

			Vector3 p = ObjectPos(step);
			float yaw = ObjectYaw(step);
			var a = new Vector3(MathF.Sin(yaw), 0f, MathF.Cos(yaw));    // the cube's two horizontal face normals
			var b = new Vector3(MathF.Cos(yaw), 0f, -MathF.Sin(yaw));
			float ca = FingerClearance(step, p, a), cb = FingerClearance(step, p, b);

			if (retry % 2 == 1 && m_Cfg.VaryRetries)
			{
				// Varied retry: the axis the previous attempt did NOT use. Without a measured axis, fall back to a plain 90° turn.
				if (m_LastFingerAxis == null) { m_RetryYaw = MathF.PI / 2f; return; }
				bool lastWasA = MathF.Abs(Vector3.Dot(m_LastFingerAxis.Value, a)) >= 0.7071f;
				m_WantedFingerAxis = lastWasA ? b : a;
				Emit($"Grasp axis: retry across the other faces, fingers along {AxisDeg(m_WantedFingerAxis.Value):F0}° " +
					$"(clearance {(lastWasA ? cb : ca) * 1000f:F0} mm)");
				return;
			}

			if (MathF.Min(ca, cb) >= m_Cfg.AxisClearEnough) return;   // room either way: take whichever the alignment gives
			bool preferA = ca >= cb;
			m_WantedFingerAxis = preferA ? a : b;
			Emit($"Grasp axis: fingers along {AxisDeg(m_WantedFingerAxis.Value):F0}° (clearance {(preferA ? ca : cb) * 1000f:F0} mm; " +
				$"other axis {AxisDeg(preferA ? b : a):F0}° has {(preferA ? cb : ca) * 1000f:F0} mm)");
		}

		// Smallest gap between either open finger and any other cube's bounding circle, for fingers along `axis`.
		private float FingerClearance(PlannedStep step, Vector3 centre, Vector3 axis)
		{
			float best = float.MaxValue;
			foreach (SortObjectDefinition other in m_Objects)
			{
				if (other.PoolTag == step.Object.PoolTag) continue;
				Vector3 o = m_World.GetObjectPosition(other.PoolTag);
				float r = other.Edge * 0.5f * MathF.Sqrt(2f);
				foreach (float side in new[] { 1f, -1f })
				{
					Vector3 finger = centre + side * m_Cfg.FingerReach * axis;
					best = MathF.Min(best, HorizontalDistance(finger, o) - r - m_Cfg.FingerHalfWidth);
				}
			}
			return best;
		}

		// After the yaw is aligned (mod 90°), check the measured finger axis. If the fingers are across the other pair of
		// faces, turn the grasp 90° (towards a smaller command, to keep the wrist away from its limits) and align again.
		private bool CorrectFingerAxis()
		{
			if (m_WantedFingerAxis == null || m_AxisCorrected) return false;
			Vector3? measured = m_Arm.FingerAxis;
			if (measured == null) return false;
			float along = MathF.Abs(Vector3.Dot(measured.Value, m_WantedFingerAxis.Value));
			if (along >= 0.7071f) return false;

			m_AxisCorrected = true;
			m_RetryYaw += m_CommandedYaw > 0f ? -MathF.PI / 2f : MathF.PI / 2f;
			Emit($"Finger axis is {AxisDeg(measured.Value):F0}°, wanted {AxisDeg(m_WantedFingerAxis.Value):F0}°: turning the grasp 90°");
			m_AlignIteration = 0;
			StartMove(m_MoveTarget, CommandFor(m_AlignDesired) + m_RetryYaw, m_MoveSpeed);
			m_StateTime = 0f;
			return true;
		}

		private static float AxisDeg(Vector3 axis)
		{
			float d = MathF.Atan2(axis.X, axis.Z) * 180f / MathF.PI;
			return d < 0f ? d + 180f : (d >= 180f ? d - 180f : d);
		}

		// Approach for pick attempt `retry` (0 = first try): the grasp alternates +90°, −90° on retries (−90° second so a
		// wrist limit on one side still leaves the other), and after a collision the centring hover is lowered.
		private void ChooseApproach(int retry)
		{
			m_RetryYaw = 0f;
			m_HoverClearance = m_Cfg.PreGraspClearance;
			if (retry == 0 || !m_Cfg.VaryRetries) return;

			// With axis choice on, the retry's axis flip comes from ChooseGraspAxis + CorrectFingerAxis instead.
			if (!m_Cfg.ChooseGraspAxis) m_RetryYaw = (retry % 2 == 1 ? 1f : -1f) * MathF.PI / 2f;
			if (m_RecoveryReason == FailureReason.GraspFailed)
				m_HoverClearance = Math.Min(m_Cfg.PreGraspClearance, m_Cfg.RetryPreGraspClearance);
			Emit(m_Cfg.ChooseGraspAxis
				? $"Retry {retry}: other grasp axis, centring hover {m_HoverClearance * 1000f:F0} mm above the cube (after {m_RecoveryReason})"
				: $"Retry {retry}: grasp turned {Deg(m_RetryYaw)}°, centring hover {m_HoverClearance * 1000f:F0} mm above the cube " +
				  $"(after {m_RecoveryReason})");
		}

		// Hover just above the object and measure where the grasp site really is. The miss is
		// accumulated into m_PosCorrection, which the final descent also applies.
		private void UpdateCenterOverObject()
		{
			MoveStatus status = CheckMove();
			if (status == MoveStatus.TimedOut) { MoveFailed("pre-grasp hover"); return; }
			if (status == MoveStatus.Running || m_StateTime < m_MoveDuration + m_Cfg.AlignDwell) return;

			Vector3 desired = PreGraspPoint(CurrentStep!);
			Vector3 g = m_Arm.GripperPosition;
			Vector3 error = new Vector3(desired.X - g.X, 0f, desired.Z - g.Z);
			if (error.Length() <= m_Cfg.CenterTolerance)
			{
				TransitionTo(TaskState.MoveToObject);
				return;
			}

			if (m_CenterIteration >= m_Cfg.MaxCenterIterations)
			{
				Emit($"Grasp site still {error.Length() * 1000f:F1} mm off-centre after {m_CenterIteration} corrections; descending anyway");
				TransitionTo(TaskState.MoveToObject);
				return;
			}

			m_PosCorrection += error;
			m_CenterIteration++;
			Emit($"Centring: grasp site off by ({-error.X * 1000f:F1}, {-error.Z * 1000f:F1}) mm (x, z); " +
				$"correction now ({m_PosCorrection.X * 1000f:F1}, {m_PosCorrection.Z * 1000f:F1}) mm");
			StartMove(desired + m_PosCorrection, m_CommandedYaw, m_Cfg.DescendSpeed);
			m_StateTime = 0f;
		}

		private void UpdateMoveToObject()
		{
			PlannedStep step = CurrentStep!;
			Vector3 obj = ObjectPos(step);
			float pushed = HorizontalDistance(obj, m_ObjectAtDescent);
			if (pushed > m_Cfg.PushTolerance)
			{
				Metrics.GraspFailures++;
				Vector3 g = m_Arm.GripperPosition;
				Emit($"Collision: {step.Object.PoolTag} pushed {pushed * 1000f:F0} mm during descent " +
					$"(gripper yaw {Deg(m_Arm.GripperYaw)}°, object yaw {Deg(ObjectYaw(step))}°, " +
					$"grasp site ({(g.X - m_ObjectAtDescent.X) * 1000f:F1}, {(g.Z - m_ObjectAtDescent.Z) * 1000f:F1}) mm (x, z) " +
					$"from where the object was, {(g.Y - obj.Y) * 1000f:F0} mm above its centre)");
				BeginRecovery(FailureReason.GraspFailed);
				return;
			}

			MoveStatus status = CheckMove();
			if (status == MoveStatus.Done) TransitionTo(TaskState.Grasp);
			else if (status == MoveStatus.TimedOut) MoveFailed("descent to object");
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

		// A move that never converged. Picking moves are retried like a failed grasp (the
		// object may have been nudged); anything else is a hard failure.
		private void MoveFailed(string what)
		{
			PlannedStep? step = CurrentStep;
			Vector3 g = m_Arm.GripperPosition;
			string detail = $"{what}: grasp site {Vector3.Distance(g, m_MoveTarget) * 1000f:F0} mm from target after {m_StateTime:F1}s. " +
				$"target ({m_MoveTarget.X:F3}, {m_MoveTarget.Y:F3}, {m_MoveTarget.Z:F3}) r={MathF.Sqrt(m_MoveTarget.X * m_MoveTarget.X + m_MoveTarget.Z * m_MoveTarget.Z):F3}, " +
				$"gripper ({g.X:F3}, {g.Y:F3}, {g.Z:F3}), yaw cmd {Deg(m_CommandedYaw)}° measured {Deg(m_Arm.GripperYaw)}°";
			if (step != null && (State == TaskState.MoveAboveObject || State == TaskState.CenterOverObject || State == TaskState.MoveToObject))
			{
				detail += $", object moved {HorizontalDistance(ObjectPos(step), m_ObjectAtDescent) * 1000f:F0} mm";
				Emit("Move failed: " + detail);
				BeginRecovery(FailureReason.Unreachable);
				return;
			}
			Fail(FailureReason.Unreachable, detail);
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
			if (m_RecoveryReason == FailureReason.PlacementFailed)
			{
				if (m_PlaceAttempts >= m_Cfg.MaxPlaceAttempts)
				{
					Fail(FailureReason.PlacementFailed, $"{tag} not placed after {m_PlaceAttempts} attempts");
					return;
				}
			}
			else if (m_PickAttempts >= m_Cfg.MaxPickAttempts)
			{
				Fail(m_RecoveryReason, $"{tag} not picked after {m_PickAttempts} attempts");
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

		private void OnMoveDone(TaskState next)
		{
			MoveStatus status = CheckMove();
			if (status == MoveStatus.Done) TransitionTo(next);
			else if (status == MoveStatus.TimedOut) MoveFailed(State.ToString());
		}

		private void Fail(FailureReason reason, string detail)
		{
			Metrics.Failure = reason;
			Metrics.FailureDetail = detail;
			Emit($"FAILED ({reason}): {detail}. {Metrics}");
			TransitionTo(TaskState.Failure);
		}

        #endregion

        // ── Motion helpers ──────────────────────────────────────────────────
        #region Motion helpers

        private enum MoveStatus { Running, Done, TimedOut }

		private void StartMove(Vector3 target, float yaw, float speed)
		{
			float dist = Vector3.Distance(m_Arm.GripperPosition, target);
			float turn = Math.Abs(WrapPi(yaw - m_CommandedYaw));
			m_MoveDuration = Math.Max(m_Cfg.MinMoveDuration, Math.Max(dist / Math.Max(speed, 1e-3f), turn / Math.Max(m_Cfg.MaxYawSpeed, 1e-3f)));
			m_MoveTarget = target;
			m_MoveSpeed = speed;
			m_CommandedYaw = yaw;
			m_Arm.MoveTo(target, yaw, m_MoveDuration);
		}

		private MoveStatus CheckMove()
		{
			if (m_StateTime < m_MoveDuration) return MoveStatus.Running;
			if (Vector3.Distance(m_Arm.GripperPosition, m_MoveTarget) <= m_Cfg.MoveTolerance) return MoveStatus.Done;
			return m_StateTime > m_MoveDuration + m_Cfg.MoveTimeoutSlack ? MoveStatus.TimedOut : MoveStatus.Running;
		}

		// Command that should produce the given physical yaw (mod 90°), kept within ±45° to
		// avoid winding up the wrist.
		private float CommandFor(float physicalYaw) => NearestCubeYaw(m_YawSign * (physicalYaw - m_YawOffset), 0f);

		// Stack yaw: square to the cube below, or 0 for an empty stack.
		private float StackYaw(PlannedStep step)
		{
			var placed = m_Placed[step.StackIndex];
			return placed.Count == 0 ? 0f : m_World.GetObjectYaw(placed[placed.Count - 1].Obj.PoolTag);
		}

		private List<(Vector3, float)> LivePlaced(int stackIndex)
		{
			var live = new List<(Vector3, float)>();
			foreach (var (obj, _) in m_Placed[stackIndex])
				live.Add((m_World.GetObjectPosition(obj.PoolTag), obj.Edge));
			return live;
		}

		private Vector3 PreGraspPoint(PlannedStep step)
		{
			Vector3 obj = ObjectPos(step);
			return new Vector3(obj.X, obj.Y + step.Object.Edge / 2f + m_HoverClearance, obj.Z);
		}

		private Vector3 ObjectPos(PlannedStep step) => m_World.GetObjectPosition(step.Object.PoolTag);
		private float ObjectYaw(PlannedStep step) => m_World.GetObjectYaw(step.Object.PoolTag);

		private void Emit(string message) => Log?.Invoke(message);

		// A cube looks the same every 90°, so pick the equivalent yaw closest to a reference.
		public static float NearestCubeYaw(float objectYaw, float currentYaw) => currentYaw + WrapQuarter(objectYaw - currentYaw);

		// Wrap to [-45°, 45°].
		public static float WrapQuarter(float a)
		{
			const float quarter = MathF.PI / 2f;
			return a - quarter * MathF.Round(a / quarter);
		}

		private static float WrapPi(float a)
		{
			while (a > MathF.PI) a -= 2f * MathF.PI;
			while (a < -MathF.PI) a += 2f * MathF.PI;
			return a;
		}

		private static string Deg(float radians) => (radians * 180f / MathF.PI).ToString("F1");

		private static float HorizontalDistance(Vector3 a, Vector3 b) =>
			MathF.Sqrt((a.X - b.X) * (a.X - b.X) + (a.Z - b.Z) * (a.Z - b.Z));

        #endregion
    }
}
