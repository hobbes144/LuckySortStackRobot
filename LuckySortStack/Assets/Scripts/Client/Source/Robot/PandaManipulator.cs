using Hazel;
using Num = System.Numerics;

namespace LuckySortStack.Robot
{
	// IManipulator on top of Lucky's RobotControllerComponent (MG_Panda motion graph,
	// LimbIK on grasp_site) and the Panda's gripper actuator. One SetInputTrigger
	// per MoveTo: each trigger cancels the move in flight.
	public sealed class PandaManipulator : IManipulator
	{
		private static readonly Identifier k_Pos      = new Identifier("Target_Position");
		private static readonly Identifier k_Rot      = new Identifier("Target_Orientation");
		private static readonly Identifier k_SolveRot = new Identifier("Solve_Orientation");
		private static readonly Identifier k_Dur      = new Identifier("Duration");
		private static readonly Identifier k_Go       = new Identifier("Start");

		// grasp_site pointing straight down, yaw 0 (measured from the rest pose).
		public Vector3 TopDownOrientationDeg = new Vector3(90f, 90f, 0f);
		// Flip to -1 if the gripper turns the wrong way relative to an object's yaw.
		public float YawSign = 1f;
		public float GripperOpenValue = 255f;    // panda.xml actuator8 ctrlrange [0, 255]
		public float GripperCloseValue = 0f;

		private readonly RobotControllerComponent m_Robot;
		private readonly MujocoActuator m_Gripper;
		private readonly Entity m_GraspSite;

		public PandaManipulator(RobotControllerComponent robot, MujocoActuator gripper, Entity graspSite)
		{
			m_Robot     = robot;
			m_Gripper   = gripper;
			m_GraspSite = graspSite;
			m_Gripper.IK = false;   // script owns the gripper; otherwise the graph overwrites it every substep
		}

		public Num.Vector3 GripperPosition
		{
			get
			{
				Vector3 p = m_GraspSite.Transform.WorldTranslation;
				return new Num.Vector3(p.X, p.Y, p.Z);
			}
		}

		public void MoveTo(Num.Vector3 position, float yaw, float durationSeconds)
		{
			Vector3 rot = TopDownOrientationDeg + new Vector3(0f, YawSign * yaw * Mathf.Rad2Deg, 0f);
			m_Robot.SetInputBool   (k_SolveRot, true);
			m_Robot.SetInputVector3(k_Pos, new Vector3(position.X, position.Y, position.Z));
			m_Robot.SetInputVector3(k_Rot, rot);
			m_Robot.SetInputFloat  (k_Dur, durationSeconds);
			m_Robot.SetInputTrigger(k_Go);
		}

		public void SetGripper(float openFraction)
		{
			m_Gripper.Value = GripperCloseValue + (GripperOpenValue - GripperCloseValue) * openFraction;
		}
	}
}
