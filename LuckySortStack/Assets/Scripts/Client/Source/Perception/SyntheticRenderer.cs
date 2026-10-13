using System;
using System.Collections.Generic;
using System.Numerics;
using LuckySortStack.Core;

namespace LuckySortStack.Perception
{
	// A box in the synthetic scene: yaw about world Y only (cubes resting on a face, the mat, markers, a stand-in arm).
	public sealed class RenderBox
	{
		public Vector3 Centre;
		public Vector3 HalfExtents;
		public float Yaw;                 // radians, same convention as IWorldState.GetObjectYaw
		public Vector3 Albedo;            // linear
		public byte Label;                // PixelClass of its pixels
		public int Id = -1;               // reported in the id image (e.g. index into a cube list), −1 = none
	}

	public sealed class RenderSettings
	{
		public Vector3 LightDirection = Vector3.Normalize(new Vector3(0.3f, 1f, 0.2f));   // towards the light
		public float Ambient = 0.45f;
		public float Diffuse = 0.55f;
		public Vector3 FloorAlbedo = VisionPalette.Floor;
		public float FloorY = 0f;
		public int Supersample = 1;       // n×n rays per pixel, averaged: anti-aliased edges like a real render
		public float Noise = 0f;          // std-dev of per-channel noise in 8-bit units
		public int NoiseSeed = 1;
	}

	public sealed class RenderOutput
	{
		public RgbaImage Image = null!;
		public byte[] Labels = null!;     // PixelClass of the box seen at each pixel centre
		public int[] Ids = null!;         // RenderBox.Id seen at each pixel centre (−1 = floor or a box without id)
		public bool[] TopFace = null!;    // the pixel centre sees an upward-facing face
	}

	// Ray-casts boxes through a PinholeCamera: the test double for the engine's camera. The tests render generated tasks
	// and check that the detector recovers the poses; the editor spike renders the ground truth and compares it with the
	// real capture (row order, camera model check, classifier calibration).
	public static class SyntheticRenderer
	{
		public static RenderOutput Render(PinholeCamera cam, IReadOnlyList<RenderBox> boxes, RenderSettings? settings = null)
		{
			settings ??= new RenderSettings();
			int w = cam.Width, h = cam.Height, n = Math.Max(1, settings.Supersample);
			var output = new RenderOutput
			{
				Image = new RgbaImage(w, h),
				Labels = new byte[w * h],
				Ids = new int[w * h],
				TopFace = new bool[w * h],
			};
			var accum = new Vector3[w * h];
			var depth = new float[w * h];
			var rng = new Random(settings.NoiseSeed);

			// Pixel rectangle each box can cover (projected corners), so a ray only tests the boxes that can be under it.
			var rects = new (int X0, int Y0, int X1, int Y1)[boxes.Count];
			for (int b = 0; b < boxes.Count; b++) rects[b] = PixelRect(cam, boxes[b]);

			for (int y = 0; y < h; y++)
				for (int x = 0; x < w; x++)
				{
					int idx = y * w + x;
					Vector3 sum = Vector3.Zero;
					for (int sy = 0; sy < n; sy++)
						for (int sx = 0; sx < n; sx++)
						{
							float u = x + (sx + 0.5f) / n, v = y + (sy + 0.5f) / n;
							bool centre = n == 1 || (sx == n / 2 && sy == n / 2);
							sum += Shade(cam, boxes, rects, settings, x, y, u, v, out byte label, out int id, out bool top);
							if (centre)
							{
								output.Labels[idx] = label;
								output.Ids[idx] = id;
								output.TopFace[idx] = top;
							}
						}
					accum[idx] = sum / (n * n);
				}

			for (int i = 0; i < w * h; i++)
			{
				var c = VisionPalette.ToSrgb8(accum[i]);
				if (settings.Noise > 0f)
					c = (Jitter(c.R, settings.Noise, rng), Jitter(c.G, settings.Noise, rng), Jitter(c.B, settings.Noise, rng));
				output.Image.Set(i % w, i / w, c);
			}
			return output;
		}

		private static Vector3 Shade(PinholeCamera cam, IReadOnlyList<RenderBox> boxes, (int X0, int Y0, int X1, int Y1)[] rects,
			RenderSettings s, int px, int py, float u, float v, out byte label, out int id, out bool top)
		{
			Vector3 dir = cam.RayDirection(u, v);
			float best = float.MaxValue;
			Vector3 normal = Vector3.UnitY;
			Vector3 albedo = s.FloorAlbedo;
			label = PixelClass.Background;
			id = -1;

			if (dir.Y < -1e-6f) best = (s.FloorY - cam.Position.Y) / dir.Y;

			for (int b = 0; b < boxes.Count; b++)
			{
				var r = rects[b];
				if (px < r.X0 || px > r.X1 || py < r.Y0 || py > r.Y1) continue;
				if (Intersect(cam.Position, dir, boxes[b], out float t, out Vector3 nrm) && t < best)
				{
					best = t;
					normal = nrm;
					albedo = boxes[b].Albedo;
					label = boxes[b].Label;
					id = boxes[b].Id;
				}
			}
			top = normal.Y > 0.9f;
			float lit = s.Ambient + s.Diffuse * MathF.Max(0f, Vector3.Dot(normal, s.LightDirection));
			return albedo * lit;
		}

		// Slab test in the box's frame. Returns the entry distance and the world normal of the face hit.
		public static bool Intersect(Vector3 origin, Vector3 dir, RenderBox box, out float t, out Vector3 normal)
		{
			float c = MathF.Cos(box.Yaw), s = MathF.Sin(box.Yaw);
			Vector3 o = ToLocal(origin - box.Centre, c, s);
			Vector3 d = ToLocal(dir, c, s);
			float tMin = float.NegativeInfinity, tMax = float.PositiveInfinity;
			int axis = -1;
			float sign = 1f;
			for (int a = 0; a < 3; a++)
			{
				float oa = a == 0 ? o.X : a == 1 ? o.Y : o.Z;
				float da = a == 0 ? d.X : a == 1 ? d.Y : d.Z;
				float ha = a == 0 ? box.HalfExtents.X : a == 1 ? box.HalfExtents.Y : box.HalfExtents.Z;
				if (MathF.Abs(da) < 1e-12f)
				{
					if (oa < -ha || oa > ha) { t = 0; normal = default; return false; }
					continue;
				}
				float t1 = (-ha - oa) / da, t2 = (ha - oa) / da;
				float near = MathF.Min(t1, t2), far = MathF.Max(t1, t2);
				if (near > tMin) { tMin = near; axis = a; sign = da > 0 ? -1f : 1f; }
				tMax = MathF.Min(tMax, far);
				if (tMin > tMax) { t = 0; normal = default; return false; }
			}
			if (tMin <= 0f || axis < 0) { t = 0; normal = default; return false; }
			t = tMin;
			Vector3 ln = axis == 0 ? new Vector3(sign, 0, 0) : axis == 1 ? new Vector3(0, sign, 0) : new Vector3(0, 0, sign);
			normal = FromLocal(ln, c, s);
			return true;
		}

		// Yaw convention: R_y(yaw) maps local +Z to (sin yaw, 0, cos yaw).
		private static Vector3 FromLocal(Vector3 v, float c, float s) => new Vector3(c * v.X + s * v.Z, v.Y, -s * v.X + c * v.Z);
		private static Vector3 ToLocal(Vector3 v, float c, float s) => new Vector3(c * v.X - s * v.Z, v.Y, s * v.X + c * v.Z);

		public static Vector3[] Corners(RenderBox box)
		{
			float c = MathF.Cos(box.Yaw), s = MathF.Sin(box.Yaw);
			var list = new Vector3[8];
			int i = 0;
			foreach (float sx in new[] { -1f, 1f })
				foreach (float sy in new[] { -1f, 1f })
					foreach (float sz in new[] { -1f, 1f })
						list[i++] = box.Centre + FromLocal(new Vector3(sx * box.HalfExtents.X, sy * box.HalfExtents.Y, sz * box.HalfExtents.Z), c, s);
			return list;
		}

		private static (int, int, int, int) PixelRect(PinholeCamera cam, RenderBox box)
		{
			float x0 = float.MaxValue, y0 = float.MaxValue, x1 = float.MinValue, y1 = float.MinValue;
			foreach (Vector3 p in Corners(box))
			{
				if (!cam.Project(p, out Vector2 q, out _)) return (0, 0, cam.Width - 1, cam.Height - 1);
				x0 = MathF.Min(x0, q.X); y0 = MathF.Min(y0, q.Y); x1 = MathF.Max(x1, q.X); y1 = MathF.Max(y1, q.Y);
			}
			return ((int)MathF.Floor(x0) - 1, (int)MathF.Floor(y0) - 1, (int)MathF.Ceiling(x1) + 1, (int)MathF.Ceiling(y1) + 1);
		}

		private static byte Jitter(byte v, float sigma, Random rng)
		{
			double u1 = 1.0 - rng.NextDouble(), u2 = rng.NextDouble();
			double g = Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Cos(2.0 * Math.PI * u2);
			return (byte)Math.Clamp((int)Math.Round(v + g * sigma), 0, 255);
		}

		// ── Scene from the workspace ────────────────────────────────────────

		// The LuckySort table as boxes: mat, stack markers, and one box per cube at the given poses. Cube ids index `cubes`.
		public static List<RenderBox> WorkspaceScene(WorkspaceSpec ws, IReadOnlyList<(ObjectColor Color, float Edge, Vector3 Centre, float Yaw)> cubes,
			float markerSize = 0.075f, float markerThickness = 0.001f)
		{
			float matThickness = ws.SurfaceY;
			var boxes = new List<RenderBox>
			{
				new RenderBox
				{
					Centre = new Vector3(0.5f * (ws.MatMinX + ws.MatMaxX), matThickness / 2f, 0.5f * (ws.MatMinZ + ws.MatMaxZ)),
					HalfExtents = new Vector3(0.5f * (ws.MatMaxX - ws.MatMinX), matThickness / 2f, 0.5f * (ws.MatMaxZ - ws.MatMinZ)),
					Albedo = VisionPalette.Mat,
				},
			};
			foreach (var (color, basePos) in ws.StackBase)
				boxes.Add(new RenderBox
				{
					Centre = new Vector3(basePos.X, ws.SurfaceY + markerThickness / 2f, basePos.Z),
					HalfExtents = new Vector3(markerSize / 2f, markerThickness / 2f, markerSize / 2f),
					Albedo = VisionPalette.Marker(color),
					Label = PixelClass.Marker(color),
				});
			for (int i = 0; i < cubes.Count; i++)
			{
				var cube = cubes[i];
				boxes.Add(new RenderBox
				{
					Centre = cube.Centre,
					HalfExtents = new Vector3(cube.Edge / 2f),
					Yaw = cube.Yaw,
					Albedo = VisionPalette.Cube[cube.Color],
					Label = PixelClass.Cube(cube.Color),
					Id = i,
				});
			}
			return boxes;
		}
	}
}
