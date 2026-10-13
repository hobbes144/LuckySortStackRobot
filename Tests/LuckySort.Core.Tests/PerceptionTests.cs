using System;
using System.Collections.Generic;
using System.IO;
using System.IO.Compression;
using System.Linq;
using System.Numerics;
using LuckySortStack.Core;
using LuckySortStack.Generation;
using LuckySortStack.Perception;
using Xunit;

namespace LuckySort.Core.Tests
{
	// Milestone 9, V1: the detector recovers cube poses from synthetic renders through the vision camera.
	// Gate (VISION_PLAN §3): every cube found, position ≤ 3 mm, yaw ≤ 3° (mod 90).
	public class PerceptionTests
	{
		private const float MaxPositionError = 0.003f;
		private const float MaxYawErrorDeg = 3f;

		private static readonly WorkspaceSpec Ws = WorkspaceSpec.Default;

		// Anti-aliased, noisy render: closer to a real frame than clean pixel centres.
		private static readonly RenderSettings Realistic = new RenderSettings { Supersample = 2, Noise = 2f };

		private static TaskDefinition Task(int seed, int piles = 0) => TaskGenerator.Generate(new TaskConfig
		{
			Seed = seed, NumColors = 4, NumSizeTiers = 4, MinObjectsPerColor = 2, MaxObjectsPerColor = 4, PileCount = piles,
		}, Ws);

		// Poses as they are once the cubes have settled (the generator adds a small drop height).
		private static List<(ObjectColor, float, Vector3, float)> Poses(TaskDefinition task) =>
			task.Objects.Select(o => (o.Color, o.Edge, Settled(task, o), o.InitialYaw)).ToList();

		private static Vector3 Settled(TaskDefinition task, SortObjectDefinition o)
		{
			float y = Ws.SurfaceY + o.Edge / 2f;
			for (string? below = o.InitialSupport; below != null; )
			{
				SortObjectDefinition b = task.Objects.First(x => x.PoolTag == below);
				y += b.Edge;
				below = b.InitialSupport;
			}
			return new Vector3(o.InitialPosition.X, y, o.InitialPosition.Z);
		}

		private static DetectionResult See(IReadOnlyList<(ObjectColor Color, float Edge, Vector3 Centre, float Yaw)> cubes,
			IEnumerable<(ObjectColor, ObjectSize)> expected, RenderSettings? render = null, List<RenderBox>? extra = null,
			ColorClassifier? classifier = null)
		{
			PinholeCamera cam = VisionRig.Camera();
			var boxes = SyntheticRenderer.WorkspaceScene(Ws, cubes);
			if (extra != null) boxes.AddRange(extra);
			RenderOutput frame = SyntheticRenderer.Render(cam, boxes, render ?? Realistic);
			byte[] labels = (classifier ?? ColorClassifier.Default()).Classify(frame.Image);
			return CubeDetector.Detect(labels, cam, Ws.SurfaceY, Ws.EdgeLength, expected);
		}

		private static float YawErrorDeg(float a, float b) => MathF.Abs(CubeDetector.NormalizeYaw(a - b)) * 180f / MathF.PI;

		private static float Horizontal(Vector3 a, Vector3 b) => MathF.Sqrt((a.X - b.X) * (a.X - b.X) + (a.Z - b.Z) * (a.Z - b.Z));

		// ── Camera model ────────────────────────────────────────────────────

		[Fact]
		public void ProjectionAndBackProjectionAgree()
		{
			PinholeCamera cam = VisionRig.Camera();
			foreach (var p in new[] { new Vector3(0.3f, 0.01f, -0.4f), new Vector3(0.7f, 0.07f, 0.4f), new Vector3(0.5f, 0.04f, 0f) })
			{
				Assert.True(cam.Project(p, out Vector2 px, out _));
				Vector3 back = cam.IntersectHorizontal(px.X, px.Y, p.Y)!.Value;
				Assert.True(Vector3.Distance(p, back) < 1e-5f, $"{p} → {px} → {back}");
			}
		}

		[Fact]
		public void TopDownCameraPutsPlusXRightAndPlusZDown()
		{
			PinholeCamera cam = VisionRig.Camera();
			Vector2 centre = cam.Project(new Vector3(0.5f, 0.01f, 0f));
			Assert.Equal(cam.Width / 2f, centre.X, 2);
			Assert.Equal(cam.Height / 2f, centre.Y, 2);
			Assert.True(cam.Project(new Vector3(0.6f, 0.01f, 0f)).X > centre.X);
			Assert.True(cam.Project(new Vector3(0.5f, 0.01f, 0.1f)).Y > centre.Y);
		}

		[Fact]
		public void WholeMatIsInViewAtAboutOneMillimetrePerPixel()
		{
			PinholeCamera cam = VisionRig.Camera();
			foreach (float x in new[] { Ws.MatMinX, Ws.MatMaxX })
				foreach (float z in new[] { Ws.MatMinZ, Ws.MatMaxZ })
				{
					Vector2 p = cam.Project(new Vector3(x, Ws.SurfaceY, z));
					Assert.InRange(p.X, 0f, cam.Width);
					Assert.InRange(p.Y, 0f, cam.Height);
				}
			Assert.InRange(cam.MetresPerPixelAt(Ws.SurfaceY), 0.0009f, 0.0012f);
		}

		// ── Images ──────────────────────────────────────────────────────────

		[Fact]
		public void PngHasSignatureHeaderAndAllRows()
		{
			var img = new RgbaImage(7, 5);
			img.Set(3, 2, 200, 100, 50);
			byte[] png = img.ToPng();
			Assert.Equal(new byte[] { 137, 80, 78, 71, 13, 10, 26, 10 }, png.Take(8).ToArray());
			Assert.Equal("IHDR", System.Text.Encoding.ASCII.GetString(png, 12, 4));
			int idat = IndexOf(png, "IDAT");
			int len = (png[idat - 4] << 24) | (png[idat - 3] << 16) | (png[idat - 2] << 8) | png[idat - 1];
			using var z = new ZLibStream(new MemoryStream(png, idat + 4, len), CompressionMode.Decompress);
			var raw = new MemoryStream();
			z.CopyTo(raw);
			Assert.Equal(5 * (1 + 7 * 4), raw.Length);
			byte[] r = raw.ToArray();
			int at = 2 * (1 + 7 * 4) + 1 + 3 * 4;
			Assert.Equal(new byte[] { 200, 100, 50, 255 }, r.Skip(at).Take(4).ToArray());
		}

		private static int IndexOf(byte[] data, string tag)
		{
			byte[] t = System.Text.Encoding.ASCII.GetBytes(tag);
			for (int i = 0; i < data.Length - 4; i++)
				if (data[i] == t[0] && data[i + 1] == t[1] && data[i + 2] == t[2] && data[i + 3] == t[3]) return i;
			return -1;
		}

		[Fact]
		public void BottomUpCaptureIsFlipped()
		{
			var rgba = new byte[2 * 3 * 4];
			rgba[0] = 255;   // first row of the buffer, red
			var img = RgbaImage.FromCapture(rgba, 2, 3, bottomUp: true);
			Assert.Equal((byte)255, img.Get(0, 2).R);
			Assert.Equal((byte)0, img.Get(0, 0).R);
		}

		// ── Classifier ──────────────────────────────────────────────────────

		[Fact]
		public void DefaultClassifierLabelsRenderedPixels()
		{
			Assert.True(LabelAccuracy(new RenderSettings(), ColorClassifier.Default()) > 0.995f);
		}

		[Fact]
		public void CalibrationAdaptsToDifferentLighting()
		{
			// Dimmer, bluer-lit scene than the defaults assume: calibrate from one frame of a known layout, then label another.
			var light = new RenderSettings { Ambient = 0.25f, Diffuse = 0.6f, LightDirection = Vector3.Normalize(new Vector3(-0.5f, 1f, 0.4f)), Noise = 2f };
			PinholeCamera cam = VisionRig.Camera();
			RenderOutput calib = SyntheticRenderer.Render(cam, SyntheticRenderer.WorkspaceScene(Ws, Poses(Task(11))), light);
			ColorClassifier calibrated = ColorClassifier.Calibrate(calib.Image, calib);
			float before = LabelAccuracy(light, ColorClassifier.Default(), seed: 12);
			float after = LabelAccuracy(light, calibrated, seed: 12);
			Assert.True(after > 0.995f, $"calibrated accuracy {after:P2} (default was {before:P2})");
		}

		[Fact]
		public void ClassifierRoundTripsThroughText()
		{
			ColorClassifier a = ColorClassifier.Default();
			ColorClassifier b = ColorClassifier.Deserialize(a.Serialize())!;
			Assert.Equal(a.References.Count, b.References.Count);
			for (int r = 0; r < 256; r += 17)
				for (int g = 0; g < 256; g += 17)
					for (int bl = 0; bl < 256; bl += 17)
						Assert.Equal(a.Classify((byte)r, (byte)g, (byte)bl), b.Classify((byte)r, (byte)g, (byte)bl));
		}

		// Share of pixel centres (away from edges) whose label matches what the renderer drew.
		private static float LabelAccuracy(RenderSettings light, ColorClassifier classifier, int seed = 7)
		{
			PinholeCamera cam = VisionRig.Camera();
			RenderOutput frame = SyntheticRenderer.Render(cam, SyntheticRenderer.WorkspaceScene(Ws, Poses(Task(seed))), light);
			byte[] labels = classifier.Classify(frame.Image);
			int w = cam.Width, ok = 0, n = 0;
			for (int y = 1; y < cam.Height - 1; y++)
				for (int x = 1; x < w - 1; x++)
				{
					int i = y * w + x;
					if (frame.Ids[i - 1] != frame.Ids[i] || frame.Ids[i + 1] != frame.Ids[i] || frame.Labels[i - w] != frame.Labels[i] || frame.Labels[i + w] != frame.Labels[i]) continue;
					n++;
					if (labels[i] == frame.Labels[i]) ok++;
				}
			return ok / (float)n;
		}

		// ── Detector: loose cubes ───────────────────────────────────────────

		public static IEnumerable<object[]> DetectorSeeds => new[] { 1, 42, 1234, 5555, 5594, 8000, 8017, 31337 }.Select(s => new object[] { s });

		[Theory]
		[MemberData(nameof(DetectorSeeds))]
		public void FindsEveryCubeOfAGeneratedTaskWithinTolerance(int seed)
		{
			TaskDefinition task = Task(seed);
			DetectionResult seen = See(Poses(task), task.Objects.Select(o => (o.Color, o.Size)));

			Assert.Empty(seen.Unexplained);
			Assert.Equal(task.Objects.Count, seen.Cubes.Count);
			foreach (SortObjectDefinition o in task.Objects)
			{
				CubeDetection? d = seen.Find(o.Color, o.Size);
				Assert.True(d != null, $"{o.PoolTag} not found; saw {string.Join("; ", seen.Cubes)}");
				float pos = Horizontal(d!.Centre, o.InitialPosition), yaw = YawErrorDeg(d.Yaw, o.InitialYaw);
				Assert.True(pos <= MaxPositionError, $"{o.PoolTag}: {pos * 1000f:F2} mm off");
				Assert.True(yaw <= MaxYawErrorDeg, $"{o.PoolTag}: {yaw:F2}° off");
				Assert.Equal(Settled(task, o).Y, d.Centre.Y, 4);
				Assert.Null(d.RestsOn);
			}
		}

		[Fact]
		public void CubesAtTheMatCornersWhereSidesShowMost()
		{
			// The worst case for perspective: far from the image centre, Large cubes show a lot of side face.
			var cubes = new List<(ObjectColor, float, Vector3, float)>
			{
				(ObjectColor.Red,    0.06f, new Vector3(0.29f, 0.04f, -0.41f), 0.3f),
				(ObjectColor.Blue,   0.06f, new Vector3(0.71f, 0.04f, -0.41f), -0.6f),
				(ObjectColor.Green,  0.06f, new Vector3(0.29f, 0.04f,  0.41f), 0.75f),
				(ObjectColor.Yellow, 0.06f, new Vector3(0.71f, 0.04f,  0.41f), 0.05f),
			};
			DetectionResult seen = See(cubes, cubes.Select(c => (c.Item1, ObjectSize.Large)));
			Assert.Equal(4, seen.Cubes.Count);
			foreach (var (color, _, centre, yaw) in cubes)
			{
				CubeDetection d = seen.Find(color, ObjectSize.Large)!;
				Assert.True(Horizontal(d.Centre, centre) <= MaxPositionError, $"{color}: {Horizontal(d.Centre, centre) * 1000f:F2} mm");
				Assert.True(YawErrorDeg(d.Yaw, yaw) <= MaxYawErrorDeg, $"{color}: {YawErrorDeg(d.Yaw, yaw):F2}°");
			}
		}

		[Fact]
		public void TellsSizesApartWhenOnlyOneCubeOfAColourIsThere()
		{
			foreach (ObjectSize size in Enum.GetValues(typeof(ObjectSize)))
			{
				float e = Ws.EdgeLength[size];
				var cubes = new List<(ObjectColor, float, Vector3, float)> { (ObjectColor.Green, e, new Vector3(0.45f, Ws.SurfaceY + e / 2f, -0.2f), 0.2f) };
				var expected = Enum.GetValues(typeof(ObjectSize)).Cast<ObjectSize>().Select(s => (ObjectColor.Green, s));
				DetectionResult seen = See(cubes, expected);
				Assert.Single(seen.Cubes);
				Assert.Equal(size, seen.Cubes[0].Size);
			}
		}

		// ── Detector: piles ─────────────────────────────────────────────────

		[Fact]
		public void TwoCubePileWithTheLowerOneSeenAsARing()
		{
			float s = Ws.SurfaceY;
			var cubes = new List<(ObjectColor, float, Vector3, float)>
			{
				(ObjectColor.Red,  0.06f, new Vector3(0.40f, s + 0.03f, -0.20f), 0.2f),
				(ObjectColor.Blue, 0.04f, new Vector3(0.404f, s + 0.06f + 0.02f, -0.197f), -0.35f),
			};
			DetectionResult seen = See(cubes, new[] { (ObjectColor.Red, ObjectSize.Large), (ObjectColor.Blue, ObjectSize.Small), (ObjectColor.Blue, ObjectSize.Tiny) });
			CubeDetection red = seen.Find(ObjectColor.Red, ObjectSize.Large)!, blue = seen.Find(ObjectColor.Blue, ObjectSize.Small)!;
			Assert.NotNull(red);
			Assert.NotNull(blue);
			Assert.Equal(ObjectColor.Red, blue.RestsOn);
			Assert.Null(red.RestsOn);
			Assert.Equal(cubes[1].Item3.Y, blue.Centre.Y, 4);
			foreach (var (d, truth) in new[] { (red, cubes[0]), (blue, cubes[1]) })
			{
				Assert.True(Horizontal(d.Centre, truth.Item3) <= MaxPositionError, $"{d}: {Horizontal(d.Centre, truth.Item3) * 1000f:F2} mm");
				Assert.True(YawErrorDeg(d.Yaw, truth.Item4) <= MaxYawErrorDeg, $"{d}: {YawErrorDeg(d.Yaw, truth.Item4):F2}°");
			}
		}

		[Fact]
		public void ThreeCubePile()
		{
			float s = Ws.SurfaceY;
			var cubes = new List<(ObjectColor, float, Vector3, float)>
			{
				(ObjectColor.Green,  0.06f, new Vector3(0.62f, s + 0.03f, -0.30f), 0.1f),
				(ObjectColor.Yellow, 0.05f, new Vector3(0.617f, s + 0.06f + 0.025f, -0.302f), 0.5f),
				(ObjectColor.Red,    0.03f, new Vector3(0.620f, s + 0.11f + 0.015f, -0.298f), -0.2f),
			};
			var expected = new[] { (ObjectColor.Green, ObjectSize.Large), (ObjectColor.Yellow, ObjectSize.Medium), (ObjectColor.Yellow, ObjectSize.Large),
				(ObjectColor.Red, ObjectSize.Tiny), (ObjectColor.Red, ObjectSize.Small) };
			DetectionResult seen = See(cubes, expected);
			CubeDetection g = seen.Find(ObjectColor.Green, ObjectSize.Large)!, y = seen.Find(ObjectColor.Yellow, ObjectSize.Medium)!, r = seen.Find(ObjectColor.Red, ObjectSize.Tiny)!;
			Assert.True(g != null && y != null && r != null, string.Join("; ", seen.Cubes) + " | " + string.Join("; ", seen.Unexplained));
			Assert.Equal(ObjectColor.Green, y.RestsOn);
			Assert.Equal(ObjectColor.Yellow, r.RestsOn);
			foreach (var (d, truth) in new[] { (g, cubes[0]), (y, cubes[1]), (r, cubes[2]) })
				Assert.True(Horizontal(d.Centre, truth.Item3) <= MaxPositionError, $"{d}: {Horizontal(d.Centre, truth.Item3) * 1000f:F2} mm");
		}

		// ── Detector: what it must not do ───────────────────────────────────

		[Fact]
		public void HalfHiddenCubeIsNotReportedWithABadPose()
		{
			// A grey box (the arm) over half of a cube: either the fit is rejected or it's still accurate. Never a wrong pose.
			var cube = (ObjectColor.Blue, 0.05f, new Vector3(0.5f, Ws.SurfaceY + 0.025f, -0.1f), 0.3f);
			var arm = new RenderBox { Centre = new Vector3(0.5f, 0.3f, -0.1f + 0.03f), HalfExtents = new Vector3(0.05f, 0.02f, 0.04f), Albedo = new Vector3(0.8f) };
			DetectionResult seen = See(new[] { cube }, new[] { (ObjectColor.Blue, ObjectSize.Medium) }, extra: new List<RenderBox> { arm });
			CubeDetection? d = seen.Find(ObjectColor.Blue, ObjectSize.Medium);
			if (d != null) Assert.True(Horizontal(d.Centre, cube.Item3) <= MaxPositionError, $"accepted a bad pose: {d}");
		}

		[Fact]
		public void SameColourStackIsUnexplainedNotAWrongCube()
		{
			// A finished target stack is one blob of one colour; it can't be split into cubes from above.
			float s = Ws.SurfaceY;
			var cubes = new List<(ObjectColor, float, Vector3, float)>
			{
				(ObjectColor.Red, 0.06f, new Vector3(0.40f, s + 0.001f + 0.03f, 0.18f), 0f),
				(ObjectColor.Red, 0.05f, new Vector3(0.40f, s + 0.061f + 0.025f, 0.18f), 0.4f),
				(ObjectColor.Red, 0.04f, new Vector3(0.40f, s + 0.111f + 0.02f, 0.18f), -0.3f),
			};
			DetectionResult seen = See(cubes, new[] { (ObjectColor.Red, ObjectSize.Large), (ObjectColor.Red, ObjectSize.Medium), (ObjectColor.Red, ObjectSize.Small) });
			Assert.All(seen.Cubes, d => Assert.True(Horizontal(d.Centre, new Vector3(0.40f, 0f, 0.18f)) < 0.01f, $"{d}"));
			Assert.True(seen.Cubes.Count + seen.Unexplained.Count >= 1);
		}

		[Fact]
		public void DetectionIsFastEnoughToRunPerDecision()
		{
			TaskDefinition task = Task(1234);
			PinholeCamera cam = VisionRig.Camera();
			RenderOutput frame = SyntheticRenderer.Render(cam, SyntheticRenderer.WorkspaceScene(Ws, Poses(task)), Realistic);
			ColorClassifier classifier = ColorClassifier.Default();
			var sw = System.Diagnostics.Stopwatch.StartNew();
			byte[] labels = classifier.Classify(frame.Image);
			CubeDetector.Detect(labels, cam, Ws.SurfaceY, Ws.EdgeLength, task.Objects.Select(o => (o.Color, o.Size)));
			sw.Stop();
			Assert.True(sw.ElapsedMilliseconds < 500, $"{sw.ElapsedMilliseconds} ms");
		}

		// ── V0 capture check (what the editor spike runs on a real frame) ───

		private static (byte[] Raw, List<TruthCube> Truth) FakeCapture(int seed, PinholeCamera renderCam, bool bottomUp, RenderSettings light)
		{
			TaskDefinition task = Task(seed);
			var truth = task.Objects.Select(o => new TruthCube { Color = o.Color, Size = o.Size, Edge = o.Edge, Centre = Settled(task, o), Yaw = o.InitialYaw }).ToList();
			RenderOutput frame = SyntheticRenderer.Render(renderCam, SyntheticRenderer.WorkspaceScene(Ws, Poses(task)), light);
			byte[] raw = bottomUp ? RgbaImage.FromCapture(frame.Image.Data, frame.Image.Width, frame.Image.Height, bottomUp: true).Data : frame.Image.Data;
			return (raw, truth);
		}

		[Fact]
		public void CaptureCheckFindsRowOrderCalibratesAndDetects()
		{
			var light = new RenderSettings { Ambient = 0.3f, Diffuse = 0.8f, Supersample = 2, Noise = 2f };
			PinholeCamera cam = VisionRig.Camera();
			var (raw, truth) = FakeCapture(5555, cam, bottomUp: true, light);
			CaptureCheckReport r = CaptureCheck.Run(raw, cam.Width, cam.Height, cam, Ws, truth);
			Assert.False(r.Blank);
			Assert.True(r.BottomUp, r.Describe());
			Assert.True(r.ModelLooksRight, r.Describe());
			Assert.All(r.Errors, e => Assert.True(e.Found && e.PositionMm <= 3f && e.YawDeg <= 3f, r.Describe()));
			Assert.Contains("BOTTOM-UP", r.Describe());
		}

		[Fact]
		public void CaptureCheckFlagsACameraModelMismatch()
		{
			// The engine renders with a wider FOV than the model assumes (e.g. FOV read as horizontal, or another aspect rule).
			PinholeCamera cam = VisionRig.Camera();
			PinholeCamera engine = VisionRig.Camera();
			engine.VerticalFovDegrees = 31f;
			var (raw, truth) = FakeCapture(42, engine, bottomUp: false, new RenderSettings());
			CaptureCheckReport r = CaptureCheck.Run(raw, cam.Width, cam.Height, cam, Ws, truth);
			Assert.False(r.ModelLooksRight, r.Describe());
			Assert.True(r.CentroidScale.Y < 0.97f, r.Describe());
			Assert.Contains("MISMATCH", r.Describe());
		}

		[Fact]
		public void CaptureCheckReportsABlankFrame()
		{
			PinholeCamera cam = VisionRig.Camera();
			var raw = new byte[cam.Width * cam.Height * 4];
			CaptureCheckReport r = CaptureCheck.Run(raw, cam.Width, cam.Height, cam, Ws, new List<TruthCube>());
			Assert.True(r.Blank);
			Assert.Contains("BLANK", r.Describe());
		}
	}
}
