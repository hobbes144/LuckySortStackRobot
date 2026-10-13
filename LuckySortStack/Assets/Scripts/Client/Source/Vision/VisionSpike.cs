using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using Hazel;
using LuckySortStack.Core;
using LuckySortStack.Perception;

namespace LuckySortStack.Vision
{
	// Milestone 9 / V0: capture the vision camera a few times (one attempt per simulation step, so a capture that renders
	// asynchronously gets its frame), then run CaptureCheck on the last frame against the true cube poses and write
	// everything to <recordings>/<time>_vision/:
	//   capture_raw.png   the buffer exactly as returned, rows in buffer order
	//   annotated.png     row order corrected, predicted outlines (green), detections (magenta), stack bases + mat (white)
	//   labels.png        what the calibrated classifier thinks each pixel is
	//   predicted.png     the synthetic render of the same layout through the camera model
	//   report.txt        row order, camera model check, per-cube error, timing
	//   classifier.txt    calibrated colour references (also copied to <recordings>/vision_classifier.txt)
	public sealed class VisionSpike
	{
		private readonly VisionSensor m_Sensor;
		private readonly Func<List<TruthCube>> m_Truth;
		private readonly WorkspaceSpec m_Workspace;
		private readonly string m_RecordingRoot;
		private readonly int m_Captures;
		private readonly int m_MaxAttempts;
		private readonly List<double> m_Times = new List<double>();
		private int m_Attempts;
		private int m_Failures;
		private byte[]? m_Last;
		private int m_W, m_H;

		public bool Done { get; private set; }
		public CaptureCheckReport? Report { get; private set; }

		public VisionSpike(VisionSensor sensor, Func<List<TruthCube>> truth, WorkspaceSpec ws, string recordingRoot, int captures = 5, int maxAttempts = 100)
		{
			m_Sensor = sensor;
			m_Truth = truth;
			m_Workspace = ws;
			m_RecordingRoot = recordingRoot;
			m_Captures = Math.Max(1, captures);
			m_MaxAttempts = Math.Max(m_Captures, maxAttempts);
			Log.Info($"[Vision] Capture spike: camera '{m_Sensor.Camera.Tag}' (entity {m_Sensor.Camera.ID}), requesting " +
				$"{m_Sensor.RequestedWidth}x{m_Sensor.RequestedHeight}, {m_Captures} captures. Model: {m_Sensor.Model(m_Sensor.RequestedWidth, m_Sensor.RequestedHeight)}");
			if (!m_Sensor.HasCameraComponent) Log.Warn("[Vision] The vision camera entity has no CameraComponent; using the default FOV.");
		}

		// Once per simulation step until Done.
		public void Update()
		{
			if (Done) return;
			m_Attempts++;
			if (m_Sensor.TryCapture(out byte[] rgba, out int w, out int h, out double ms))
			{
				m_Times.Add(ms);
				m_Last = rgba; m_W = w; m_H = h;
				if (m_Times.Count == 1)
					Log.Info($"[Vision] First capture on attempt {m_Attempts}: {w}x{h}, {rgba.Length} bytes, {ms:F1} ms");
			}
			else m_Failures++;

			if (m_Times.Count >= m_Captures || m_Attempts >= m_MaxAttempts) Finish();
		}

		private void Finish()
		{
			Done = true;
			if (m_Last == null)
			{
				Log.Error($"[Vision] No capture succeeded in {m_Attempts} attempts. TryCaptureCameraRgba returned false every time: " +
					"the capture bridge may need the gRPC server running, or a primary camera. Try making LS_Camera_Vision primary for one run.");
				return;
			}

			string dir = Path.Combine(m_RecordingRoot, DateTime.Now.ToString("yyyyMMdd_HHmmss", CultureInfo.InvariantCulture) + "_vision");
			try
			{
				Directory.CreateDirectory(dir);
				var raw = RgbaImage.FromCapture(m_Last, m_W, m_H, bottomUp: false);
				raw.SavePng(Path.Combine(dir, "capture_raw.png"));

				List<TruthCube> truth = m_Truth();
				PinholeCamera model = m_Sensor.Model(m_W, m_H);
				CaptureCheckReport report = CaptureCheck.Run(m_Last, m_W, m_H, model, m_Workspace, truth);
				var sorted = m_Times.OrderBy(t => t).ToList();
				report.CaptureMs = (float)sorted[sorted.Count / 2];
				Report = report;

				report.Annotated.SavePng(Path.Combine(dir, "annotated.png"));
				report.LabelView.SavePng(Path.Combine(dir, "labels.png"));
				report.TruthRender.SavePng(Path.Combine(dir, "predicted.png"));
				string text = report.Describe() +
					$"\nCaptures: {m_Times.Count} ok, {m_Failures} failed, attempts {m_Attempts}; times ms: {string.Join(", ", m_Times.Select(t => t.ToString("F1", CultureInfo.InvariantCulture)))}" +
					$"\nCamera model: {model}\n";
				File.WriteAllText(Path.Combine(dir, "report.txt"), text);
				string classifier = report.Classifier.Serialize();
				File.WriteAllText(Path.Combine(dir, "classifier.txt"), classifier);
				File.WriteAllText(Path.Combine(m_RecordingRoot, "vision_classifier.txt"), classifier);

				foreach (string line in text.Split('\n'))
					if (line.Length > 0) Log.Info("[Vision] " + line);
				Log.Info($"[Vision] Wrote {dir}");
			}
			catch (Exception e) when (e is IOException || e is UnauthorizedAccessException)
			{
				Log.Error($"[Vision] Could not write the spike output to {dir}: {e.Message}");
			}
		}
	}
}
