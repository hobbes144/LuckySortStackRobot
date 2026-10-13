using System;
using System.Numerics;

namespace LuckySortStack.Perception
{
	// Ideal perspective camera, the model the vision pipeline uses to go between table coordinates and pixels.
	//
	// Conventions (match Hazel's camera entities, see UI/DebugOverlay): camera-local space looks down −Z with +Y up and
	// +X right; `Rotation` takes camera-local directions to world. Pixels: u to the right, v DOWN, origin at the top-left
	// corner of the image, pixel (i, j) covering [i, i+1) × [j, j+1) so its centre is (i + 0.5, j + 0.5). Square pixels;
	// the horizontal field of view follows from the aspect Width / Height.
	//
	// The engine side builds one from the vision camera entity (world transform + CameraComponent.VerticalFOV) and the
	// capture size, so there's no intrinsics calibration: the scene authors the camera. The V0 spike checks that the
	// engine renders captures with this same model (projected markers drawn over the captured frame).
	public sealed class PinholeCamera
	{
		public Vector3 Position;
		public Quaternion Rotation = Quaternion.Identity;
		public float VerticalFovDegrees = 30f;
		public int Width = 512;
		public int Height = 900;

		// Focal length in pixels (same on both axes).
		public float FocalPixels => 0.5f * Height / MathF.Tan(0.5f * VerticalFovDegrees * MathF.PI / 180f);

		public Vector3 Forward => Vector3.Transform(-Vector3.UnitZ, Rotation);

		// Camera at `position` looking straight down (−Y). Image right = world +X, image down = world +Z: the rotation is
		// −90° about X, which is what an entity rotation of [−90°, 0, 0] gives.
		public static PinholeCamera LookingDown(Vector3 position, float verticalFovDegrees, int width, int height) => new PinholeCamera
		{
			Position = position,
			Rotation = Quaternion.CreateFromAxisAngle(Vector3.UnitX, -MathF.PI / 2f),
			VerticalFovDegrees = verticalFovDegrees,
			Width = width,
			Height = height,
		};

		// World point → pixel. False when the point is behind the camera.
		public bool Project(Vector3 world, out Vector2 pixel, out float depth)
		{
			Vector3 local = Vector3.Transform(world - Position, Quaternion.Conjugate(Rotation));
			depth = -local.Z;
			if (depth <= 1e-6f)
			{
				pixel = default;
				return false;
			}
			float f = FocalPixels;
			pixel = new Vector2(0.5f * Width + f * local.X / depth, 0.5f * Height - f * local.Y / depth);
			return true;
		}

		public Vector2 Project(Vector3 world) => Project(world, out Vector2 p, out _) ? p : new Vector2(float.NaN, float.NaN);

		// Unit world-space direction of the ray through pixel coordinates (u, v) (not necessarily a pixel centre).
		public Vector3 RayDirection(float u, float v)
		{
			float f = FocalPixels;
			var local = new Vector3((u - 0.5f * Width) / f, -(v - 0.5f * Height) / f, -1f);
			return Vector3.Normalize(Vector3.Transform(local, Rotation));
		}

		// Where the ray through (u, v) meets the horizontal plane at height y, or null if it never does (in front).
		public Vector3? IntersectHorizontal(float u, float v, float y)
		{
			Vector3 d = RayDirection(u, v);
			if (MathF.Abs(d.Y) < 1e-6f) return null;
			float t = (y - Position.Y) / d.Y;
			if (t <= 0f) return null;
			return Position + t * d;
		}

		// Size of one pixel on the horizontal plane at height y, under the image centre (metres).
		public float MetresPerPixelAt(float y) => MathF.Abs(Position.Y - y) / FocalPixels;

		public PinholeCamera WithSize(int width, int height) => new PinholeCamera
		{
			Position = Position, Rotation = Rotation, VerticalFovDegrees = VerticalFovDegrees, Width = width, Height = height,
		};

		public override string ToString() =>
			$"pos ({Position.X:F3}, {Position.Y:F3}, {Position.Z:F3}), fwd ({Forward.X:F2}, {Forward.Y:F2}, {Forward.Z:F2}), " +
			$"vfov {VerticalFovDegrees:F1}°, {Width}x{Height}, f {FocalPixels:F1} px";
	}
}
