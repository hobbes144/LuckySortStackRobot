using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Numerics;
using System.Text;
using LuckySortStack.Core;

namespace LuckySortStack.Perception
{
	public sealed class TruthCube
	{
		public ObjectColor Color;
		public ObjectSize Size;
		public float Edge;
		public Vector3 Centre;
		public float Yaw;
		public string Tag => SceneTags.PoolObject(Color, Size);
	}

	public sealed class CubeError
	{
		public string Tag = "";
		public bool Found;
		public float PositionMm;
		public float YawDeg;
		public float HeightMm;
		public float Misfit;
	}

	public sealed class CaptureCheckReport
	{
		public int Width, Height;
		public bool Blank;                       // the capture is one flat colour: the camera didn't render
		public float IoUTopDown, IoUBottomUp;    // cube pixels seen vs predicted, for each row-order reading
		public bool BottomUp;                    // chosen row order
		public Vector2 CentroidScale;            // fitted seen/predicted scale of cube positions about the image centre (1,1 = model right)
		public Vector2 CentroidOffset;           // mean seen − predicted cube centroid (pixels)
		public float CaptureMs = -1f;            // filled in by the caller
		public RgbaImage Corrected = null!;      // capture in top-row-first order
		public RgbaImage Annotated = null!;      // + predicted (green) and detected (magenta) outlines, marker crosses
		public RgbaImage LabelView = null!;      // classifier output in flat colours
		public RgbaImage TruthRender = null!;    // what the synthetic renderer predicts for this camera and layout
		public ColorClassifier Classifier = null!;   // calibrated from this frame
		public DetectionResult Detection = null!;
		public List<CubeError> Errors = new List<CubeError>();

		public bool ModelLooksRight => !Blank && MathF.Max(IoUTopDown, IoUBottomUp) > 0.6f
			&& MathF.Abs(CentroidScale.X - 1f) < 0.03f && MathF.Abs(CentroidScale.Y - 1f) < 0.03f;

		public string Describe()
		{
			var ci = CultureInfo.InvariantCulture;
			var sb = new StringBuilder();
			sb.Append(string.Format(ci, "Capture {0}x{1}{2}\n", Width, Height, CaptureMs >= 0 ? string.Format(ci, ", {0:F1} ms per capture", CaptureMs) : ""));
			if (Blank) { sb.Append("BLANK: the capture is a single colour. The vision camera isn't rendering (non-primary camera?).\n"); return sb.ToString(); }
			sb.Append(string.Format(ci, "Row order: {0} (cube-pixel IoU with the prediction: top-down {1:F2}, bottom-up {2:F2})\n",
				BottomUp ? "BOTTOM-UP (first row = bottom of picture)" : "TOP-DOWN (first row = top of picture)", IoUTopDown, IoUBottomUp));
			sb.Append(string.Format(ci, "Camera model: seen/predicted scale x {0:F3}, y {1:F3}; mean offset ({2:F1}, {3:F1}) px → {4}\n",
				CentroidScale.X, CentroidScale.Y, CentroidOffset.X, CentroidOffset.Y,
				ModelLooksRight ? "OK" : "MISMATCH (check FOV units, aspect handling, camera rotation)"));
			int found = Errors.Count(e => e.Found);
			sb.Append($"Detected {found}/{Errors.Count} cubes ({Detection.Unexplained.Count} unexplained clusters)\n");
			if (found > 0)
			{
				var f = Errors.Where(e => e.Found).ToList();
				sb.Append(string.Format(ci, "Position error mean {0:F2} mm, max {1:F2} mm; yaw error mean {2:F2}°, max {3:F2}°\n",
					f.Average(e => e.PositionMm), f.Max(e => e.PositionMm), f.Average(e => e.YawDeg), f.Max(e => e.YawDeg)));
			}
			foreach (CubeError e in Errors)
				sb.Append(e.Found
					? string.Format(ci, "  {0,-22} {1,5:F2} mm  {2,5:F2}°  height {3,5:F1} mm  misfit {4:P0}\n", e.Tag, e.PositionMm, e.YawDeg, e.HeightMm, e.Misfit)
					: $"  {e.Tag,-22} NOT FOUND\n");
			foreach (UnexplainedCluster u in Detection.Unexplained) sb.Append("  unexplained: ").Append(u).Append('\n');
			return sb.ToString();
		}
	}

	// V0 of Milestone 9: everything that can be learned from one real capture of a known layout. Engine-free (the editor
	// hands in the raw buffer, the camera model built from the camera entity, and the true cube poses).
	//  - Blank frame?                → the engine didn't render the camera.
	//  - Row order                   → classify both readings, compare with the synthetic render of the true layout.
	//  - Camera model                → per-cube centroid seen vs predicted: scale ≠ 1 means FOV or aspect is handled
	//                                  differently by the engine; an offset means the pose/rotation is off.
	//  - Classifier calibration      → references learned from this frame (ColorClassifier.Calibrate).
	//  - Detector accuracy           → detect with the calibrated classifier, compare with the true poses.
	public static class CaptureCheck
	{
		public static CaptureCheckReport Run(byte[] raw, int width, int height, PinholeCamera cameraModel, WorkspaceSpec ws,
			IReadOnlyList<TruthCube> truth)
		{
			PinholeCamera cam = cameraModel.WithSize(width, height);
			var report = new CaptureCheckReport { Width = width, Height = height };

			var topDown = RgbaImage.FromCapture(raw, width, height, bottomUp: false);
			report.Blank = IsBlank(topDown);
			report.Corrected = topDown;

			var boxes = SyntheticRenderer.WorkspaceScene(ws, truth.Select(t => (t.Color, t.Edge, t.Centre, t.Yaw)).ToList());
			RenderOutput predicted = SyntheticRenderer.Render(cam, boxes);
			report.TruthRender = predicted.Image;
			report.Detection = new DetectionResult();
			if (report.Blank)
			{
				report.Classifier = ColorClassifier.Default();
				report.Annotated = topDown.Clone();
				report.LabelView = LabelImage(new byte[width * height], width, height);
				return report;
			}

			// Row order: which reading puts cube colours where the cubes should be?
			ColorClassifier defaults = ColorClassifier.Default();
			var bottomUp = RgbaImage.FromCapture(raw, width, height, bottomUp: true);
			report.IoUTopDown = CubeIoU(defaults.Classify(topDown), predicted.Labels);
			report.IoUBottomUp = CubeIoU(defaults.Classify(bottomUp), predicted.Labels);
			report.BottomUp = report.IoUBottomUp > report.IoUTopDown;
			RgbaImage image = report.BottomUp ? bottomUp : topDown;
			report.Corrected = image;

			// Camera model check on the default labels (calibration would hide nothing here, but needs the model right).
			byte[] coarse = defaults.Classify(image);
			FitCentroids(coarse, predicted, truth, cam, out report.CentroidScale, out report.CentroidOffset);

			// Calibrate on this frame, then detect.
			report.Classifier = ColorClassifier.Calibrate(image, predicted, defaults);
			byte[] labels = report.Classifier.Classify(image);
			report.LabelView = LabelImage(labels, width, height);
			report.Detection = CubeDetector.Detect(labels, cam, ws.SurfaceY, ws.EdgeLength, truth.Select(t => (t.Color, t.Size)));

			foreach (TruthCube t in truth)
			{
				CubeDetection? d = report.Detection.Find(t.Color, t.Size);
				var e = new CubeError { Tag = t.Tag, Found = d != null };
				if (d != null)
				{
					e.PositionMm = 1000f * MathF.Sqrt((d.Centre.X - t.Centre.X) * (d.Centre.X - t.Centre.X) + (d.Centre.Z - t.Centre.Z) * (d.Centre.Z - t.Centre.Z));
					e.YawDeg = MathF.Abs(CubeDetector.NormalizeYaw(d.Yaw - t.Yaw)) * 180f / MathF.PI;
					e.HeightMm = 1000f * (d.Centre.Y - t.Centre.Y);
					e.Misfit = d.Mismatch;
				}
				report.Errors.Add(e);
			}

			report.Annotated = Annotate(image, cam, ws, truth, report.Detection);
			return report;
		}

		private static bool IsBlank(RgbaImage img)
		{
			byte[] d = img.Data;
			int min = 255, max = 0;
			for (int i = 0; i < d.Length; i += 4 * 7)
				for (int c = 0; c < 3; c++) { min = Math.Min(min, d[i + c]); max = Math.Max(max, d[i + c]); }
			return max - min < 6;
		}

		private static float CubeIoU(byte[] seen, byte[] predicted)
		{
			int inter = 0, union = 0;
			for (int i = 0; i < seen.Length; i++)
			{
				bool a = PixelClass.IsCube(seen[i]), b = PixelClass.IsCube(predicted[i]);
				if (a && b) inter++;
				if (a || b) union++;
			}
			return union == 0 ? 0f : inter / (float)union;
		}

		// For each cube: centroid of its predicted pixels vs centroid of pixels of its colour near there. Least-squares scale
		// about the image centre per axis, and the mean offset.
		private static void FitCentroids(byte[] labels, RenderOutput predicted, IReadOnlyList<TruthCube> truth, PinholeCamera cam,
			out Vector2 scale, out Vector2 offset)
		{
			int w = cam.Width, h = cam.Height;
			var c0 = new Vector2(w / 2f, h / 2f);
			double sxx = 0, sxo = 0, syy = 0, syo = 0, ox = 0, oy = 0;
			int n = 0;
			for (int id = 0; id < truth.Count; id++)
			{
				double px = 0, py = 0;
				int pn = 0, x0 = w, y0 = h, x1 = 0, y1 = 0;
				for (int i = 0; i < predicted.Ids.Length; i++)
				{
					if (predicted.Ids[i] != id) continue;
					int x = i % w, y = i / w;
					px += x + 0.5; py += y + 0.5; pn++;
					x0 = Math.Min(x0, x); x1 = Math.Max(x1, x); y0 = Math.Min(y0, y); y1 = Math.Max(y1, y);
				}
				if (pn < 50) continue;
				const int Pad = 25;   // room for a model error of a few % at the edge of the image
				byte want = PixelClass.Cube(truth[id].Color);
				double sx = 0, sy = 0;
				int sn = 0;
				for (int y = Math.Max(0, y0 - Pad); y <= Math.Min(h - 1, y1 + Pad); y++)
					for (int x = Math.Max(0, x0 - Pad); x <= Math.Min(w - 1, x1 + Pad); x++)
											{
						int i = y * w + x;
						// Skip pixels predicted to belong to another cube (a neighbour of the same colour would bias the centroid).
						if (labels[i] == want && (predicted.Ids[i] == id || predicted.Ids[i] < 0)) { sx += x + 0.5; sy += y + 0.5; sn++; }
					}
				if (sn < 50) continue;
				var p = new Vector2((float)(px / pn), (float)(py / pn)) - c0;
				var o = new Vector2((float)(sx / sn), (float)(sy / sn)) - c0;
				sxx += p.X * p.X; sxo += p.X * o.X; syy += p.Y * p.Y; syo += p.Y * o.Y;
				ox += o.X - p.X; oy += o.Y - p.Y; n++;
			}
			scale = new Vector2(sxx > 1 ? (float)(sxo / sxx) : float.NaN, syy > 1 ? (float)(syo / syy) : float.NaN);
			offset = n > 0 ? new Vector2((float)(ox / n), (float)(oy / n)) : new Vector2(float.NaN, float.NaN);
		}

		public static RgbaImage Annotate(RgbaImage image, PinholeCamera cam, WorkspaceSpec ws, IReadOnlyList<TruthCube> truth, DetectionResult detection)
		{
			RgbaImage a = image.Clone();
			foreach (TruthCube t in truth)
				a.DrawPolygon(CubeDetector.Silhouette(cam, t.Centre, t.Edge, t.Yaw), (0, 255, 0));
			foreach (CubeDetection d in detection.Cubes)
			{
				a.DrawPolygon(d.Silhouette, (255, 0, 255));
				a.DrawCross(cam.Project(d.Centre + new Vector3(0, ws.EdgeLength[d.Size] / 2f, 0)), 3, (255, 0, 255));
			}
			foreach (Vector3 b in ws.StackBase.Values) a.DrawCross(cam.Project(b), 6, (255, 255, 255));
			var corners = new[]
			{
				new Vector3(ws.MatMinX, ws.SurfaceY, ws.MatMinZ), new Vector3(ws.MatMaxX, ws.SurfaceY, ws.MatMinZ),
				new Vector3(ws.MatMaxX, ws.SurfaceY, ws.MatMaxZ), new Vector3(ws.MatMinX, ws.SurfaceY, ws.MatMaxZ),
			};
			a.DrawPolygon(corners.Select(c => cam.Project(c)).ToArray(), (255, 255, 255));
			return a;
		}

		public static RgbaImage LabelImage(byte[] labels, int w, int h)
		{
			var img = new RgbaImage(w, h);
			for (int i = 0; i < labels.Length; i++)
			{
				byte l = labels[i];
				(byte, byte, byte) c = l == PixelClass.Background ? ((byte)30, (byte)30, (byte)30)
					: VisionPalette.ToSrgb8(PixelClass.IsCube(l) ? VisionPalette.Cube[PixelClass.ColorOf(l)] : VisionPalette.Cube[PixelClass.ColorOf(l)] * 0.35f);
				img.Set(i % w, i / w, c);
			}
			return img;
		}
	}
}
