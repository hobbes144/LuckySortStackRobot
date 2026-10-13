using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Numerics;
using System.Text;
using LuckySortStack.Core;

namespace LuckySortStack.Perception
{
	// Labels each pixel with a PixelClass by nearest reference colour.
	//
	// Feature space is the HSV cone: (s·cos h, s·sin h, k·v). Hue and saturation separate the four cube colours from each
	// other and from the grey mat and robot; saturation separates a cube from its stack marker (the same hue, washed out);
	// a small weight on value keeps shaded side faces with their cube. Background has references too (mat, floor, greys),
	// so there are no thresholds to tune, only references, which `Calibrate` learns from a real capture of a known layout.
	// Lookups go through a 32×32×32 table, so classifying a 512×900 frame is one array read per pixel.
	public sealed class ColorClassifier
	{
		public const float ValueWeight = 0.35f;

		public readonly struct Reference
		{
			public readonly byte Label;
			public readonly Vector3 Feature;
			public readonly string Note;
			public Reference(byte label, Vector3 feature, string note = "") { Label = label; Feature = feature; Note = note; }
		}

		private readonly List<Reference> m_Refs;
		private readonly byte[] m_Lut = new byte[32 * 32 * 32];

		public IReadOnlyList<Reference> References => m_Refs;

		public ColorClassifier(IEnumerable<Reference> references)
		{
			m_Refs = references.ToList();
			if (m_Refs.Count == 0) throw new ArgumentException("no references", nameof(references));
			for (int r = 0; r < 32; r++)
				for (int g = 0; g < 32; g++)
					for (int b = 0; b < 32; b++)
						m_Lut[(r << 10) | (g << 5) | b] = Nearest(Feature((byte)(r * 8 + 4), (byte)(g * 8 + 4), (byte)(b * 8 + 4)));
		}

		public static Vector3 Feature(byte r8, byte g8, byte b8)
		{
			float r = r8 / 255f, g = g8 / 255f, b = b8 / 255f;
			float max = MathF.Max(r, MathF.Max(g, b)), min = MathF.Min(r, MathF.Min(g, b));
			float v = max, s = max > 1e-5f ? (max - min) / max : 0f;
			float h = 0f;
			if (max - min > 1e-5f)
			{
				if (max == r) h = (g - b) / (max - min);
				else if (max == g) h = 2f + (b - r) / (max - min);
				else h = 4f + (r - g) / (max - min);
				h *= MathF.PI / 3f;
			}
			// Saturation means little in near-black pixels (noise dominates): fade it out below v = 0.15.
			float sw = s * MathF.Min(1f, v / 0.15f);
			return new Vector3(sw * MathF.Cos(h), sw * MathF.Sin(h), ValueWeight * v);
		}

		public static Vector3 Feature((byte R, byte G, byte B) c) => Feature(c.R, c.G, c.B);

		public byte Classify(byte r, byte g, byte b) => m_Lut[((r >> 3) << 10) | ((g >> 3) << 5) | (b >> 3)];

		public byte[] Classify(RgbaImage img)
		{
			var labels = new byte[img.Width * img.Height];
			byte[] d = img.Data;
			for (int i = 0, p = 0; i < labels.Length; i++, p += 4) labels[i] = Classify(d[p], d[p + 1], d[p + 2]);
			return labels;
		}

		private byte Nearest(Vector3 f)
		{
			float best = float.MaxValue;
			byte label = PixelClass.Background;
			foreach (Reference r in m_Refs)
			{
				float d = Vector3.DistanceSquared(f, r.Feature);
				if (d < best) { best = d; label = r.Label; }
			}
			return label;
		}

		// ── Defaults and calibration ────────────────────────────────────────

		// References predicted from the scene's material colours under a simple light (what SyntheticRenderer draws).
		public static ColorClassifier Default(RenderSettings? light = null)
		{
			light ??= new RenderSettings();
			var refs = new List<Reference>();
			float top = light.Ambient + light.Diffuse * MathF.Max(0f, light.LightDirection.Y);
			float[] sides = { light.Ambient, light.Ambient + 0.5f * light.Diffuse * 0.3f, light.Ambient + light.Diffuse * 0.3f };
			foreach (ObjectColor c in Enum.GetValues(typeof(ObjectColor)))
			{
				refs.Add(new Reference(PixelClass.Cube(c), Feature(VisionPalette.ToSrgb8(VisionPalette.Cube[c] * top)), "top"));
				foreach (float s in sides)
					refs.Add(new Reference(PixelClass.Cube(c), Feature(VisionPalette.ToSrgb8(VisionPalette.Cube[c] * s)), "side"));
				refs.Add(new Reference(PixelClass.Marker(c), Feature(VisionPalette.ToSrgb8(VisionPalette.Marker(c) * top)), "marker"));
			}
			refs.Add(new Reference(PixelClass.Background, Feature(VisionPalette.ToSrgb8(VisionPalette.Mat * top)), "mat"));
			refs.Add(new Reference(PixelClass.Background, Feature(VisionPalette.ToSrgb8(light.FloorAlbedo * top)), "floor"));
			foreach (Vector3 n in VisionPalette.Neutral)
				refs.Add(new Reference(PixelClass.Background, Feature(VisionPalette.ToSrgb8(n)), "grey"));
			return new ColorClassifier(refs);
		}

		// Learn references from a real capture of a known layout: `truth` is the synthetic render of the same scene from
		// the same camera (ground-truth poses), which says what each pixel shows. Edge pixels (any 4-neighbour showing
		// something else) are skipped, since they blend colours. Per cube colour: one top-face reference and up to three
		// side-face references (by brightness); per marker colour one; background: up to six clusters. Classes with no
		// clean pixels in the frame keep their default references.
		public static ColorClassifier Calibrate(RgbaImage image, RenderOutput truth, ColorClassifier? fallback = null)
		{
			fallback ??= Default();
			int w = image.Width, h = image.Height;
			var groups = new Dictionary<(byte Label, bool Top), List<Vector3>>();
			for (int y = 1; y < h - 1; y++)
				for (int x = 1; x < w - 1; x++)
				{
					int i = y * w + x;
					byte l = truth.Labels[i];
					int id = truth.Ids[i];
					bool top = truth.TopFace[i];
					bool interior = true;
					foreach (int j in new[] { i - 1, i + 1, i - w, i + w })
						if (truth.Labels[j] != l || truth.Ids[j] != id || truth.TopFace[j] != top) { interior = false; break; }
					if (!interior) continue;
					var key = (l, PixelClass.IsCube(l) ? top : true);
					if (!groups.TryGetValue(key, out var list)) groups[key] = list = new List<Vector3>();
					list.Add(Feature(image.Get(x, y)));
				}

			var refs = new List<Reference>();
			var learned = new HashSet<byte>();
			const int MinPixels = 30;
			foreach (var ((label, top), feats) in groups.OrderBy(g => g.Key.Label).ThenBy(g => g.Key.Top))
			{
				if (feats.Count < MinPixels) continue;
				int clusters = label == PixelClass.Background ? 6 : PixelClass.IsCube(label) && !top ? 3 : 1;
				foreach (Vector3 centre in KMeans(feats, clusters))
					refs.Add(new Reference(label, centre, label == PixelClass.Background ? "bg" : top ? "top" : "side"));
				learned.Add(label);
			}
			foreach (Reference r in fallback.References)
				if (!learned.Contains(r.Label)) refs.Add(r);
			return new ColorClassifier(refs);
		}

		// Deterministic k-means on the value axis-sorted points (init at value quantiles).
		private static List<Vector3> KMeans(List<Vector3> pts, int k)
		{
			k = Math.Max(1, Math.Min(k, pts.Count / 10 + 1));
			var sorted = pts.OrderBy(p => p.Z).ThenBy(p => p.X).ToList();
			var centres = Enumerable.Range(0, k).Select(i => sorted[(int)((i + 0.5f) / k * (sorted.Count - 1))]).ToList();
			var assign = new int[sorted.Count];
			for (int iter = 0; iter < 15; iter++)
			{
				for (int i = 0; i < sorted.Count; i++)
				{
					float best = float.MaxValue;
					for (int c = 0; c < k; c++)
					{
						float d = Vector3.DistanceSquared(sorted[i], centres[c]);
						if (d < best) { best = d; assign[i] = c; }
					}
				}
				for (int c = 0; c < k; c++)
				{
					Vector3 sum = Vector3.Zero;
					int n = 0;
					for (int i = 0; i < sorted.Count; i++) if (assign[i] == c) { sum += sorted[i]; n++; }
					if (n > 0) centres[c] = sum / n;
				}
			}
			return centres;
		}

		// ── Persistence (one reference per line: label x y z note) ───────────

		public string Serialize()
		{
			var sb = new StringBuilder("# LuckySort colour references: label feature.x feature.y feature.z note\n");
			foreach (Reference r in m_Refs)
				sb.Append(string.Format(CultureInfo.InvariantCulture, "{0} {1:R} {2:R} {3:R} {4}\n",
					PixelClass.Name(r.Label), r.Feature.X, r.Feature.Y, r.Feature.Z, r.Note));
			return sb.ToString();
		}

		public static ColorClassifier? Deserialize(string text)
		{
			var refs = new List<Reference>();
			foreach (string raw in text.Split('\n'))
			{
				string line = raw.Trim();
				if (line.Length == 0 || line.StartsWith("#")) continue;
				string[] p = line.Split(' ', StringSplitOptions.RemoveEmptyEntries);
				if (p.Length < 4) return null;
				byte? label = null;
				for (byte l = 0; l < PixelClass.Count; l++) if (PixelClass.Name(l) == p[0]) label = l;
				if (label == null) return null;
				var ci = CultureInfo.InvariantCulture;
				refs.Add(new Reference(label.Value,
					new Vector3(float.Parse(p[1], ci), float.Parse(p[2], ci), float.Parse(p[3], ci)), p.Length > 4 ? p[4] : ""));
			}
			return refs.Count == 0 ? null : new ColorClassifier(refs);
		}
	}
}
