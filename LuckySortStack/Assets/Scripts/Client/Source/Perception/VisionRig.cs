using System;
using System.Numerics;

namespace LuckySortStack.Perception
{
	// Where the vision camera is. Tools/SceneBuilder authors the LS_Camera_Vision entity with these values (keep in sync);
	// at runtime the camera model is built from the entity itself, so these are only the tests' and the builder's copy.
	//
	// Perspective, straight down over the mat centre, 2 m up: the 0.9 m long axis of the mat (world Z) runs down the
	// image's 900 px, the 0.5 m short axis (world X) across its 512 px, ~1.07 mm per pixel on the mat. High up so the cube
	// sides it sees off-axis stay small (≤ ~15 mm at the corners); the detector models them anyway.
	public static class VisionRig
	{
		public const string CameraTag = "LS_Camera_Vision";
		public static readonly Vector3 Position = new Vector3(0.50f, 2.0f, 0.0f);
		public const float VerticalFovDegrees = 27.5f;
		public const int Width = 512;
		public const int Height = 900;

		public static PinholeCamera Camera() => PinholeCamera.LookingDown(Position, VerticalFovDegrees, Width, Height);
	}
}
