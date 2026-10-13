using System;
using System.Collections.Generic;
using System.Numerics;
using LuckySortStack.Core;

namespace LuckySortStack.Perception
{
	// What a pixel shows, as far as the vision pipeline cares. Cube colours are ObjectColor + 1, marker colours
	// ObjectColor + 5; everything else (mat, floor, robot, shadows) is Background.
	public static class PixelClass
	{
		public const byte Background = 0;
		public const byte FirstCube = 1;
		public const byte FirstMarker = 5;
		public const int Count = 9;

		public static byte Cube(ObjectColor c) => (byte)(FirstCube + (int)c);
		public static byte Marker(ObjectColor c) => (byte)(FirstMarker + (int)c);
		public static bool IsCube(byte label) => label >= FirstCube && label < FirstMarker;
		public static bool IsMarker(byte label) => label >= FirstMarker && label < Count;
		public static ObjectColor ColorOf(byte label) => (ObjectColor)((IsMarker(label) ? label - FirstMarker : label - FirstCube));

		public static string Name(byte label) =>
			label == Background ? "Background" : IsCube(label) ? ColorOf(label).ToString() : ColorOf(label) + "Marker";
	}

	// Material colours of the scene (linear albedo), as Tools/SceneBuilder writes them. Used for the classifier's default
	// references and the synthetic renderer. The editor run calibrates the classifier from a real capture instead.
	public static class VisionPalette
	{
		public static readonly Dictionary<ObjectColor, Vector3> Cube = new Dictionary<ObjectColor, Vector3>
		{
			{ ObjectColor.Red,    new Vector3(0.85f, 0.12f, 0.10f) },
			{ ObjectColor.Blue,   new Vector3(0.10f, 0.30f, 0.85f) },
			{ ObjectColor.Green,  new Vector3(0.12f, 0.65f, 0.20f) },
			{ ObjectColor.Yellow, new Vector3(0.95f, 0.80f, 0.10f) },
		};

		// SceneBuilder: colour * 0.45 + grey 0.55 * 0.25.
		public static Vector3 Marker(ObjectColor c) => Cube[c] * 0.45f + new Vector3(0.1375f);

		public static readonly Vector3 Mat = new Vector3(0.18f, 0.19f, 0.21f);
		public static readonly Vector3 Floor = new Vector3(0.45f, 0.45f, 0.45f);

		// Plain greys the robot and shadows can show (background references for the classifier).
		public static readonly Vector3[] Neutral =
		{
			new Vector3(0.02f), new Vector3(0.08f), new Vector3(0.25f), new Vector3(0.5f), new Vector3(0.8f), new Vector3(1f),
		};

		// Linear [0,1] → 8-bit sRGB, what a renderer writes to a display buffer.
		public static byte ToSrgb8(float linear)
		{
			float c = Math.Clamp(linear, 0f, 1f);
			float s = c <= 0.0031308f ? 12.92f * c : 1.055f * MathF.Pow(c, 1f / 2.4f) - 0.055f;
			return (byte)Math.Clamp((int)MathF.Round(s * 255f), 0, 255);
		}

		public static (byte R, byte G, byte B) ToSrgb8(Vector3 linear) => (ToSrgb8(linear.X), ToSrgb8(linear.Y), ToSrgb8(linear.Z));
	}
}
