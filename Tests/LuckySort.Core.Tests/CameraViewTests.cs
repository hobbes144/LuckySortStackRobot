using System;
using System.Numerics;
using LuckySortStack.Presentation;
using Xunit;

namespace LuckySort.Core.Tests
{
	public class CameraViewTests
	{
		private static void Near(Vector3 expected, Vector3 actual, float tol = 1e-3f) =>
			Assert.True(Vector3.Distance(expected, actual) < tol, $"expected {expected}, got {actual}");

		[Fact]
		public void MatchesTheEnginesDocumentedExample()
		{
			// AgentDocs components/Camera.md: looking at the origin from (0, 2, 5) -> rotation ~= (-0.38, 0, 0).
			Vector3 e = CameraViews.LookAtEuler(new Vector3(0, 2, 5), Vector3.Zero);
			Assert.InRange(e.X, -0.39f, -0.37f);
			Assert.Equal(0f, e.Y, 3);
		}

		[Fact]
		public void MatchesTheScenesAuthoredCamera()
		{
			// Main.hscene camera: rotation (0, 1.5708, 0.436332) looks towards -X (the robot), 25 degrees down.
			Vector3 f = CameraViews.Forward(new Vector3(0f, 1.5708f, 0.436332f));
			Near(new Vector3(-MathF.Cos(0.436332f), -MathF.Sin(0.436332f), 0f), f);
		}

		[Theory]
		[InlineData(CameraView.Overview)]
		[InlineData(CameraView.Front)]
		[InlineData(CameraView.TopDown)]
		[InlineData(CameraView.Follow)]
		public void EveryViewLooksAtItsTargetWithoutRoll(CameraView view)
		{
			var (eye, target) = CameraViews.Pose(view, new Vector3(0.5f, 0.03f, -0.2f));
			Vector3 euler = CameraViews.LookAtEuler(eye, target);
			Near(Vector3.Normalize(target - eye), CameraViews.Forward(euler));
			Vector3 up = CameraViews.Up(euler);
			Assert.True(up.Y > 0f, "camera is upside down");
			// No roll: the camera's right axis stays horizontal.
			Vector3 right = Vector3.Cross(CameraViews.Forward(euler), up);
			Assert.True(MathF.Abs(right.Y) < 1e-3f, $"rolled: right = {right}");
		}

		[Fact]
		public void TopDownPutsTheRobotAtTheBottomOfTheScreen()
		{
			var (eye, target) = CameraViews.Pose(CameraView.TopDown, Vector3.Zero);
			Vector3 up = CameraViews.Up(CameraViews.LookAtEuler(eye, target));
			Assert.True(up.X > 0.9f, $"screen-up should point away from the robot (+X), got {up}");
		}

		[Fact]
		public void BlendIsFrameRateIndependent()
		{
			float one = CameraViews.Blend(0.02f, 6f);
			float twoHalves = 1f - (1f - CameraViews.Blend(0.01f, 6f)) * (1f - CameraViews.Blend(0.01f, 6f));
			Assert.Equal(one, twoHalves, 5);
		}
	}
}
