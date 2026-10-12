using System;
using System.Numerics;

namespace LuckySortStack.Presentation
{
	public enum CameraView
	{
		Authored,   // the camera as saved in the scene
		Overview,   // high three-quarter view: robot, pick area and stacks
		Front,      // low view across the mat towards the robot
		TopDown,    // straight down on the workspace, robot at the bottom of the screen
		Follow,     // trails the cube being picked
	}

	// Presentation camera poses and the look-at maths. Engine-free so the Euler convention is unit-tested.
	//
	// Convention (Hazel/glm): Transform.Rotation is Euler XYZ in radians composed as R = Rz·Ry·Rx (X applied first), and a
	// camera looks down its local −Z. Checked against two known poses: AgentDocs' "(0, 2, 5) looking at the origin ≈
	// (−0.38, 0, 0)" and this scene's camera (0, 90°, 25°) at (1.24, 0.53, 0), which looks towards −X tilted down.
	public static class CameraViews
	{
		public const int Count = 5;

		// Workspace-relative poses (world metres, Y-up, robot base at the origin, +X forward; mat x 0.25–0.75, z ±0.45).
		public static (Vector3 Eye, Vector3 Target) Pose(CameraView view, Vector3 focus) => view switch
		{
			CameraView.Overview => (new Vector3(1.30f, 0.95f, 0.85f), new Vector3(0.45f, 0.05f, -0.02f)),
			CameraView.Front    => (new Vector3(1.20f, 0.50f, 0.00f), new Vector3(0.45f, 0.06f, -0.02f)),
			CameraView.TopDown  => (new Vector3(0.42f, 1.30f, 0.00f), new Vector3(0.50f, 0.00f, 0.00f)),
			CameraView.Follow   => (focus + new Vector3(0.40f, 0.30f, 0.30f), focus),
			_ => (focus, focus),   // Authored: the caller keeps the saved pose
		};

		// Euler (pitch, yaw, 0) that points the camera's −Z from eye at target with no roll.
		public static Vector3 LookAtEuler(Vector3 eye, Vector3 target)
		{
			Vector3 f = target - eye;
			float len = f.Length();
			if (len < 1e-6f) return Vector3.Zero;
			f /= len;
			float pitch = MathF.Asin(Math.Clamp(f.Y, -1f, 1f));
			float yaw = MathF.Atan2(-f.X, -f.Z);
			return new Vector3(pitch, yaw, 0f);
		}

		// Direction the camera's −Z points for an Euler XYZ rotation (R = Rz·Ry·Rx).
		public static Vector3 Forward(Vector3 euler)
		{
			float a = euler.X, b = euler.Y, c = euler.Z;
			// Rx: (0, 0, −1) → (0, sin a, −cos a)
			float x = 0f, y = MathF.Sin(a), z = -MathF.Cos(a);
			// Ry
			float x1 = x * MathF.Cos(b) + z * MathF.Sin(b);
			float z1 = -x * MathF.Sin(b) + z * MathF.Cos(b);
			// Rz
			float x2 = x1 * MathF.Cos(c) - y * MathF.Sin(c);
			float y2 = x1 * MathF.Sin(c) + y * MathF.Cos(c);
			return new Vector3(x2, y2, z1);
		}

		// The camera's local +Y in world space (for checking there's no roll).
		public static Vector3 Up(Vector3 euler)
		{
			float a = euler.X, b = euler.Y, c = euler.Z;
			float x = 0f, y = MathF.Cos(a), z = MathF.Sin(a);
			float x1 = x * MathF.Cos(b) + z * MathF.Sin(b);
			float z1 = -x * MathF.Sin(b) + z * MathF.Cos(b);
			float x2 = x1 * MathF.Cos(c) - y * MathF.Sin(c);
			float y2 = x1 * MathF.Sin(c) + y * MathF.Cos(c);
			return new Vector3(x2, y2, z1);
		}

		// Frame-rate-independent exponential approach: fraction of the remaining distance to cover this frame.
		public static float Blend(float dt, float sharpness) => 1f - MathF.Exp(-Math.Max(0f, dt) * sharpness);
	}
}
