using System.Numerics;

namespace LuckySortStack.Robot
{
	// What the task state machine needs from a robot arm. The engine implementation
	// (PandaManipulator) drives Lucky's motion-graph IK; tests use a fake.
	// All positions are world space, metres, Y-up.
	public interface IManipulator
	{
		// Live position of the grasp point (between the fingertips).
		Vector3 GripperPosition { get; }

		// Live yaw (radians about world Y) of one of the gripper's horizontal axes. Only meaningful
		// modulo 90°, which is all a cube grasp needs.
		float GripperYaw { get; }

		// Issue one Cartesian move. yaw = rotation of the gripper about world Y (radians),
		// gripper pointing straight down. A new call cancels any move in flight.
		void MoveTo(Vector3 position, float yaw, float durationSeconds);

		// 1 = fully open, 0 = fully closed.
		void SetGripper(float openFraction);
	}

	// Read-only physical state of the task objects, by scene tag.
	public interface IWorldState
	{
		Vector3 GetObjectPosition(string tag);
		float GetObjectYaw(string tag);   // radians about world Y
	}
}
