using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using LuckySortStack.Core;

namespace LuckySortStack.Perception
{
	public sealed class CubeDetection
	{
		public ObjectColor Color;
		public ObjectSize Size;
		public Vector3 Centre;            // world, body centre
		public float Yaw;                 // radians, in (−45°, 45°] (a cube is symmetric under 90°)
		public float Mismatch;            // misfit pixels / silhouette pixels (0 = perfect)
		public int Pixels;                // pixels of this colour in its cluster
		public ObjectColor? RestsOn;      // colour of the cube directly below, in a pile (null = on the mat)
		public Vector2[] Silhouette = Array.Empty<Vector2>();   // fitted outline in the image (pixels), for debug drawing

		public string PoolTag => SceneTags.PoolObject(Color, Size);

		public override string ToString() =>
			$"{Color} {Size} at ({Centre.X:F4}, {Centre.Y:F4}, {Centre.Z:F4}) yaw {Yaw * 180f / MathF.PI:F1}° " +
			$"misfit {Mismatch:P0}{(RestsOn is ObjectColor c ? $" on {c}" : "")}";
	}

	// Cube-coloured pixels the detector couldn't explain as cubes of the expected set (a stack of same-colour cubes,
	// a cube half hidden by the arm, an unexpected object). Still occupied space.
	public sealed class UnexplainedCluster
	{
		public Dictionary<ObjectColor, int> PixelsByColor = new Dictionary<ObjectColor, int>();
		public Vector2 PixelCentroid;
		public Vector3 TableCentre;       // ray through the centroid meets the mat
		public string Reason = "";

		public override string ToString() =>
			$"{string.Join("+", PixelsByColor.Select(kv => $"{kv.Key}:{kv.Value}px"))} near ({TableCentre.X:F3}, {TableCentre.Z:F3}): {Reason}";
	}

	public sealed class DetectionResult
	{
		public List<CubeDetection> Cubes = new List<CubeDetection>();
		public List<UnexplainedCluster> Unexplained = new List<UnexplainedCluster>();

		public CubeDetection? Find(ObjectColor c, ObjectSize s) => Cubes.FirstOrDefault(d => d.Color == c && d.Size == s);
	}

	public sealed class DetectorSettings
	{
		public int MinClusterPixels = 150;      // smaller blobs are noise (a Tiny top face is ~800 px at 1 mm/px)
		public int MinColorPixels = 80;         // a colour in a cluster needs this many pixels to count (blended edge pixels don't)
		public float MaxMismatch = 0.30f;       // fits worse than this are not trusted
		public int MaxPileHeight = 4;
	}

	// Finds the expected cubes in a labelled image (ColorClassifier output) seen through a known PinholeCamera.
	//
	// 1. Connected clusters of cube-coloured pixels. A loose cube is one single-colour cluster; a pile is one cluster
	//    with one colour per cube (vision mode only piles smaller cubes on larger ones of a different colour,
	//    VISION_PLAN §4).
	// 2. Each cube is located by fitting a model: the silhouette of a cube of known edge, resting at a known height, is
	//    the convex hull of its 8 projected corners, which includes the side faces the perspective camera sees off-axis
	//    (up to ~15 mm at the mat's corners, too much to ignore). The pose (x, z, yaw) is found by pattern search,
	//    minimising misfit = silhouette area not covered by the cube's colour + cube pixels outside the silhouette,
	//    with horizontal sub-pixel coverage per image row. In a pile, pixels of the cubes above are "don't care" for the
	//    cube below, so a lower cube seen only as a ring still fits.
	// 3. Size: every size the colour can still be is tried and the best fit wins; in a pile, every size assignment
	//    (largest at the bottom) is tried and the lowest total misfit wins. Identity follows: one cube per colour × size.
	public static class CubeDetector
	{
		public static DetectionResult Detect(byte[] labels, PinholeCamera cam, float surfaceY,
			IReadOnlyDictionary<ObjectSize, float> edges, IEnumerable<(ObjectColor Color, ObjectSize Size)> expected,
			DetectorSettings? settings = null)
		{
			settings ??= new DetectorSettings();
			var result = new DetectionResult();
			var available = new Dictionary<ObjectColor, List<ObjectSize>>();
			foreach (var (c, s) in expected.Distinct())
			{
				if (!available.TryGetValue(c, out var list)) available[c] = list = new List<ObjectSize>();
				list.Add(s);
			}

			List<Cluster> clusters = FindClusters(labels, cam.Width, cam.Height, settings);

			// Piles first: their size assignment is constrained by order, and claims sizes before the loose cubes pick.
			foreach (Cluster cl in clusters.Where(c => c.Colors.Count > 1))
			{
				var fits = FitPile(cl, cam, surfaceY, edges, available, settings);
				if (fits == null)
				{
					result.Unexplained.Add(Unexplained(cl, cam, surfaceY, "no size assignment fits this pile"));
					continue;
				}
				foreach (CubeDetection d in fits) available[d.Color].Remove(d.Size);
				result.Cubes.AddRange(fits);
			}

			// Loose cubes: per colour, assign sizes to clusters by lowest total misfit.
			foreach (var group in clusters.Where(c => c.Colors.Count == 1).GroupBy(c => c.Colors[0]))
			{
				ObjectColor color = group.Key;
				List<Cluster> cls = group.ToList();
				List<ObjectSize> sizes = available.TryGetValue(color, out var a) ? a.ToList() : new List<ObjectSize>();
				var fit = new CubeDetection?[cls.Count, sizes.Count];
				for (int i = 0; i < cls.Count; i++)
					for (int j = 0; j < sizes.Count; j++)
						fit[i, j] = FitCube(cls[i], color, sizes[j], edges[sizes[j]], surfaceY, cam, Array.Empty<ObjectColor>());

				int[] best = AssignSizes(cls.Count, sizes.Count, (i, j) => fit[i, j]?.Mismatch ?? float.MaxValue, settings.MaxMismatch);
				for (int i = 0; i < cls.Count; i++)
				{
					if (best[i] < 0)
					{
						string why = sizes.Count == 0 ? $"no {color} cube expected"
							: $"best fit misfit {Enumerable.Range(0, sizes.Count).Min(j => fit[i, j]?.Mismatch ?? 9f):P0}";
						result.Unexplained.Add(Unexplained(cls[i], cam, surfaceY, why));
						continue;
					}
					CubeDetection d = fit[i, best[i]]!;
					available[color].Remove(d.Size);
					result.Cubes.Add(d);
				}
			}
			return result;
		}

		// ── Clusters ────────────────────────────────────────────────────────

		private sealed class Cluster
		{
			public int X0 = int.MaxValue, Y0 = int.MaxValue, X1 = int.MinValue, Y1 = int.MinValue;
			public readonly Dictionary<ObjectColor, List<int>> PixelsByColor = new Dictionary<ObjectColor, List<int>>();
			public List<ObjectColor> Colors = new List<ObjectColor>();   // colours with enough pixels
			public int Width;                                              // image width (pixel index = y * Width + x)
			public int Total;
		}

		private static List<Cluster> FindClusters(byte[] labels, int w, int h, DetectorSettings settings)
		{
			var visited = new bool[labels.Length];
			var clusters = new List<Cluster>();
			var stack = new Stack<int>();
			for (int start = 0; start < labels.Length; start++)
			{
				if (visited[start] || !PixelClass.IsCube(labels[start])) continue;
				var cl = new Cluster { Width = w };
				visited[start] = true;
				stack.Push(start);
				while (stack.Count > 0)
				{
					int i = stack.Pop();
					int x = i % w, y = i / w;
					ObjectColor c = PixelClass.ColorOf(labels[i]);
					if (!cl.PixelsByColor.TryGetValue(c, out var list)) cl.PixelsByColor[c] = list = new List<int>();
					list.Add(i);
					cl.Total++;
					cl.X0 = Math.Min(cl.X0, x); cl.X1 = Math.Max(cl.X1, x);
					cl.Y0 = Math.Min(cl.Y0, y); cl.Y1 = Math.Max(cl.Y1, y);
					if (x > 0) Visit(i - 1);
					if (x < w - 1) Visit(i + 1);
					if (y > 0) Visit(i - w);
					if (y < h - 1) Visit(i + w);
				}
				if (cl.Total < settings.MinClusterPixels) continue;
				cl.Colors = cl.PixelsByColor.Where(kv => kv.Value.Count >= settings.MinColorPixels)
					.OrderByDescending(kv => kv.Value.Count).Select(kv => kv.Key).ToList();
				if (cl.Colors.Count > 0) clusters.Add(cl);
			}
			return clusters;

			void Visit(int j)
			{
				if (visited[j] || !PixelClass.IsCube(labels[j])) return;
				visited[j] = true;
				stack.Push(j);
			}
		}

		private static UnexplainedCluster Unexplained(Cluster cl, PinholeCamera cam, float surfaceY, string reason)
		{
			double sx = 0, sy = 0;
			int n = 0;
			foreach (var list in cl.PixelsByColor.Values)
				foreach (int i in list) { sx += i % cl.Width + 0.5; sy += i / cl.Width + 0.5; n++; }
			var c = new Vector2((float)(sx / n), (float)(sy / n));
			return new UnexplainedCluster
			{
				PixelsByColor = cl.PixelsByColor.ToDictionary(kv => kv.Key, kv => kv.Value.Count),
				PixelCentroid = c,
				TableCentre = cam.IntersectHorizontal(c.X, c.Y, surfaceY) ?? Vector3.Zero,
				Reason = reason,
			};
		}

		// ── Piles ───────────────────────────────────────────────────────────

		private static List<CubeDetection>? FitPile(Cluster cl, PinholeCamera cam, float surfaceY,
			IReadOnlyDictionary<ObjectSize, float> edges, Dictionary<ObjectColor, List<ObjectSize>> available, DetectorSettings settings)
		{
			List<ObjectColor> colors = cl.Colors.Take(settings.MaxPileHeight).ToList();
			var cache = new Dictionary<(ObjectColor, ObjectSize, int, string), CubeDetection?>();
			List<CubeDetection>? best = null;
			float bestCost = float.MaxValue;

			foreach (ObjectSize[] sizes in Assignments(colors, available))
			{
				// Bottom → top by size (largest at the bottom).
				int[] order = Enumerable.Range(0, colors.Count).OrderByDescending(i => (int)sizes[i]).ToArray();
				var fits = new List<CubeDetection>();
				float baseY = surfaceY, total = 0f;
				bool ok = true;
				for (int k = 0; k < order.Length; k++)
				{
					ObjectColor color = colors[order[k]];
					ObjectSize size = sizes[order[k]];
					ObjectColor[] above = order.Skip(k + 1).Select(i => colors[i]).ToArray();
					string aboveKey = string.Join(",", above.OrderBy(c => c));
					int baseKey = (int)MathF.Round(baseY * 10000f);
					var key = (color, size, baseKey, aboveKey);
					if (!cache.TryGetValue(key, out CubeDetection? fit))
						cache[key] = fit = FitCube(cl, color, size, edges[size], baseY, cam, above);
					if (fit == null || fit.Mismatch > settings.MaxMismatch) { ok = false; break; }
					var d = Copy(fit);
					d.RestsOn = k == 0 ? null : colors[order[k - 1]];
					fits.Add(d);
					total += fit.Mismatch * cl.PixelsByColor[color].Count;
					baseY += edges[size];
				}
				if (!ok || total >= bestCost) continue;
				bestCost = total;
				best = fits;
			}
			return best;
		}

		// Every way to give each colour a distinct size it can still be.
		private static IEnumerable<ObjectSize[]> Assignments(List<ObjectColor> colors, Dictionary<ObjectColor, List<ObjectSize>> available)
		{
			var current = new ObjectSize[colors.Count];
			var used = new HashSet<ObjectSize>();
			var results = new List<ObjectSize[]>();
			void Rec(int i)
			{
				if (i == colors.Count) { results.Add((ObjectSize[])current.Clone()); return; }
				if (!available.TryGetValue(colors[i], out var sizes)) return;
				foreach (ObjectSize s in sizes)
				{
					if (used.Contains(s)) continue;   // strictly smaller on larger: sizes in a pile are distinct
					used.Add(s);
					current[i] = s;
					Rec(i + 1);
					used.Remove(s);
				}
			}
			Rec(0);
			return results;
		}

		private static CubeDetection Copy(CubeDetection d) => new CubeDetection
		{
			Color = d.Color, Size = d.Size, Centre = d.Centre, Yaw = d.Yaw, Mismatch = d.Mismatch, Pixels = d.Pixels,
			RestsOn = d.RestsOn, Silhouette = d.Silhouette,
		};

		// Lowest-total-cost assignment of rows (clusters) to distinct columns (sizes); −1 = unassigned. Small (≤ 4×4).
		private static int[] AssignSizes(int rows, int cols, Func<int, int, float> cost, float maxCost)
		{
			var best = Enumerable.Repeat(-1, rows).ToArray();
			var current = new int[rows];
			var used = new bool[cols];
			int bestCount = -1;
			float bestTotal = float.MaxValue;
			void Rec(int i, int count, float total)
			{
				if (i == rows)
				{
					if (count > bestCount || (count == bestCount && total < bestTotal))
					{
						bestCount = count; bestTotal = total;
						Array.Copy(current, best, rows);
					}
					return;
				}
				for (int j = 0; j < cols; j++)
				{
					if (used[j]) continue;
					float c = cost(i, j);
					if (c > maxCost) continue;
					used[j] = true; current[i] = j;
					Rec(i + 1, count + 1, total + c);
					used[j] = false;
				}
				current[i] = -1;
				Rec(i + 1, count, total);
			}
			Rec(0, 0, 0f);
			return best;
		}

		// ── Model fit ───────────────────────────────────────────────────────

		private const int Margin = 24;   // pixels around the cluster's box that the silhouette may spill into

		// Fit one cube of `color` and `edge` resting at baseY. Pixels of `occluders` (cubes above it) don't count against it.
		private static CubeDetection? FitCube(Cluster cl, ObjectColor color, ObjectSize size, float edge, float baseY,
			PinholeCamera cam, ObjectColor[] occluders)
		{
			if (!cl.PixelsByColor.TryGetValue(color, out var own)) return null;
			var raster = new FitRaster(cl, color, occluders, cam.Width, cam.Height);
			float centreY = baseY + edge / 2f;

			// Start: the ray through the centroid of the pixels this cube may cover meets its mid-height plane.
			Vector2 c = raster.AllowedCentroid;
			Vector3? start = cam.IntersectHorizontal(c.X, c.Y, centreY);
			if (start == null) return null;
			float x = start.Value.X, z = start.Value.Z;

			float Cost(float px, float pz, float pyaw) => raster.Misfit(cam, new Vector3(px, centreY, pz), edge, pyaw);

			// Coarse yaw (a cube repeats every 90°).
			float yaw = 0f, best = float.MaxValue;
			for (int k = 0; k < 15; k++)
			{
				float a = k * 6f * MathF.PI / 180f;
				float cost = Cost(x, z, a);
				if (cost < best) { best = cost; yaw = a; }
			}

			// Pattern search.
			float step = 0.004f, yawStep = 4f * MathF.PI / 180f;
			for (int iter = 0; iter < 400 && step > 0.00005f; iter++)
			{
				bool improved = false;
				foreach (var (dx, dz, dy) in new[] { (step, 0f, 0f), (-step, 0f, 0f), (0f, step, 0f), (0f, -step, 0f), (0f, 0f, yawStep), (0f, 0f, -yawStep) })
				{
					float cost = Cost(x + dx, z + dz, yaw + dy);
					if (cost < best - 1e-4f)
					{
						best = cost; x += dx; z += dz; yaw += dy;
						improved = true;
					}
				}
				if (!improved) { step *= 0.5f; yawStep *= 0.5f; }
			}

			var centre = new Vector3(x, centreY, z);
			float area = raster.LastArea(cam, centre, edge, yaw);
			return new CubeDetection
			{
				Color = color,
				Size = size,
				Centre = centre,
				Yaw = NormalizeYaw(yaw),
				Mismatch = area > 0f ? best / area : float.MaxValue,
				Pixels = own.Count,
				Silhouette = Silhouette(cam, centre, edge, yaw),
			};
		}

		public static float NormalizeYaw(float yaw)
		{
			float q = MathF.PI / 2f;
			float r = yaw % q;
			if (r <= -q / 2f) r += q;
			if (r > q / 2f) r -= q;
			return r;
		}

		// Convex hull (pixels) of the 8 projected corners of a cube.
		public static Vector2[] Silhouette(PinholeCamera cam, Vector3 centre, float edge, float yaw)
		{
			var box = new RenderBox { Centre = centre, HalfExtents = new Vector3(edge / 2f), Yaw = yaw };
			var pts = SyntheticRenderer.Corners(box).Select(p => cam.Project(p)).ToList();
			return ConvexHull(pts);
		}

		private static Vector2[] ConvexHull(List<Vector2> pts)
		{
			pts = pts.Where(p => !float.IsNaN(p.X)).OrderBy(p => p.X).ThenBy(p => p.Y).ToList();
			if (pts.Count < 3) return pts.ToArray();
			var hull = new List<Vector2>();
			for (int pass = 0; pass < 2; pass++)
			{
				int start = hull.Count;
				foreach (Vector2 p in pts)
				{
					while (hull.Count >= start + 2 && Cross(hull[hull.Count - 2], hull[hull.Count - 1], p) <= 0f) hull.RemoveAt(hull.Count - 1);
					hull.Add(p);
				}
				hull.RemoveAt(hull.Count - 1);
				pts.Reverse();
			}
			return hull.ToArray();
		}

		private static float Cross(Vector2 o, Vector2 a, Vector2 b) => (a.X - o.X) * (b.Y - o.Y) - (a.Y - o.Y) * (b.X - o.X);

		// Row prefix sums over the cluster's box (plus margin) for the cube's own pixels and for "allowed" pixels
		// (own ∪ occluders), so a silhouette's misfit costs O(rows) per evaluation.
		private sealed class FitRaster
		{
			private readonly int m_X0, m_Y0, m_W, m_H;
			private readonly float[] m_Own;       // (m_W + 1) per row: prefix sums
			private readonly float[] m_Allowed;
			private readonly int m_OwnTotal;
			public readonly Vector2 AllowedCentroid;

			public FitRaster(Cluster cl, ObjectColor color, ObjectColor[] occluders, int imgW, int imgH)
			{
				m_X0 = Math.Max(0, cl.X0 - Margin);
				m_Y0 = Math.Max(0, cl.Y0 - Margin);
				int x1 = Math.Min(imgW - 1, cl.X1 + Margin), y1 = Math.Min(imgH - 1, cl.Y1 + Margin);
				m_W = x1 - m_X0 + 1;
				m_H = y1 - m_Y0 + 1;
				var own = new byte[m_W * m_H];
				var allowed = new byte[m_W * m_H];
				double sx = 0, sy = 0;
				int n = 0;
				void Mark(List<int> list, bool isOwn)
				{
					foreach (int i in list)
					{
						int x = i % cl.Width - m_X0, y = i / cl.Width - m_Y0;
						int j = y * m_W + x;
						if (isOwn) own[j] = 1;
						allowed[j] = 1;
						sx += x + m_X0 + 0.5; sy += y + m_Y0 + 0.5; n++;
					}
				}
				Mark(cl.PixelsByColor[color], true);
				foreach (ObjectColor o in occluders)
					if (cl.PixelsByColor.TryGetValue(o, out var list)) Mark(list, false);
				m_OwnTotal = cl.PixelsByColor[color].Count;
				AllowedCentroid = new Vector2((float)(sx / n), (float)(sy / n));

				m_Own = new float[(m_W + 1) * m_H];
				m_Allowed = new float[(m_W + 1) * m_H];
				for (int y = 0; y < m_H; y++)
				{
					int r = y * (m_W + 1);
					for (int x = 0; x < m_W; x++)
					{
						m_Own[r + x + 1] = m_Own[r + x] + own[y * m_W + x];
						m_Allowed[r + x + 1] = m_Allowed[r + x] + allowed[y * m_W + x];
					}
				}
			}

			// ∫ mask over [a, b) along row y (image pixel coordinates), with pixel i covering [i, i+1).
			private float RowIntegral(float[] prefix, int row, float a, float b)
			{
				return At(prefix, row, b) - At(prefix, row, a);
			}

			private float At(float[] prefix, int row, float x)
			{
				float lx = x - m_X0;
				int r = row * (m_W + 1);
				if (lx <= 0f) return 0f;
				if (lx >= m_W) return prefix[r + m_W];
				int i = (int)lx;
				return prefix[r + i] + (lx - i) * (prefix[r + i + 1] - prefix[r + i]);
			}

			public float Misfit(PinholeCamera cam, Vector3 centre, float edge, float yaw) => Evaluate(cam, centre, edge, yaw, out _);
			public float LastArea(PinholeCamera cam, Vector3 centre, float edge, float yaw) { Evaluate(cam, centre, edge, yaw, out float a); return a; }

			// misfit = (silhouette area not allowed) + (own pixels outside the silhouette)
			//        = area − ∫allowed·sil + ownTotal − ∫own·sil
			private float Evaluate(PinholeCamera cam, Vector3 centre, float edge, float yaw, out float area)
			{
				Vector2[] poly = Silhouette(cam, centre, edge, yaw);
				area = 0f;
				if (poly.Length < 3) return float.MaxValue;
				float minY = poly.Min(p => p.Y), maxY = poly.Max(p => p.Y);
				int j0 = (int)MathF.Ceiling(minY - 0.5f), j1 = (int)MathF.Floor(maxY - 0.5f);
				float inAllowed = 0f, inOwn = 0f;
				for (int j = j0; j <= j1; j++)
				{
					float yc = j + 0.5f;
					if (!RowSpan(poly, yc, out float xl, out float xr)) continue;
					area += xr - xl;
					int row = j - m_Y0;
					if (row < 0 || row >= m_H) continue;
					inAllowed += RowIntegral(m_Allowed, row, xl, xr);
					inOwn += RowIntegral(m_Own, row, xl, xr);
				}
				return area - inAllowed + m_OwnTotal - inOwn;
			}

			private static bool RowSpan(Vector2[] poly, float y, out float xl, out float xr)
			{
				xl = float.MaxValue; xr = float.MinValue;
				for (int i = 0; i < poly.Length; i++)
				{
					Vector2 a = poly[i], b = poly[(i + 1) % poly.Length];
					if ((a.Y <= y && b.Y > y) || (b.Y <= y && a.Y > y))
					{
						float t = (y - a.Y) / (b.Y - a.Y);
						float x = a.X + t * (b.X - a.X);
						xl = MathF.Min(xl, x); xr = MathF.Max(xr, x);
					}
				}
				return xr > xl;
			}
		}
	}
}
