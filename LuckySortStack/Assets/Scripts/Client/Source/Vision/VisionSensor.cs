using System;
using System.Diagnostics;
using Hazel;
using Hazel.Net.Grpc.Capture;
using LuckySortStack.Perception;
using Num = System.Numerics;

namespace LuckySortStack.Vision
{
	// The robot's camera: the LS_Camera_Vision entity, read through the engine's capture bridge.
	//
	// CaptureBridge.TryCaptureCameraRgba(ulong entityId, uint width, uint height, out byte[] rgba, out uint outWidth,
	// out uint outHeight) is public but undocumented (it backs the gRPC CameraService). Signature and parameter names
	// checked against Hazel-ScriptCore.dll 2026.1.2 metadata (LUCKY_API_NOTES §11); behaviour (cost, row order, whether a
	// non-primary camera renders) is what the V0 spike measures.
	public sealed class VisionSensor
	{
		public readonly Entity Camera;
		public readonly int RequestedWidth;
		public readonly int RequestedHeight;
		private readonly CameraComponent? m_Component;

		private VisionSensor(Entity camera, int width, int height)
		{
			Camera = camera;
			RequestedWidth = width;
			RequestedHeight = height;
			m_Component = camera.GetComponent<CameraComponent>();
		}

		public static VisionSensor? Find(Entity anyEntity, string tag, int width, int height)
		{
			Entity? camera = anyEntity.FindEntityByTag(tag);
			if (camera == null) return null;
			return new VisionSensor(camera, width, height);
		}

		public bool HasCameraComponent => m_Component != null;

		// The camera model, built from the entity as it is now (so moving the camera in the editor needs no recalibration).
		public PinholeCamera Model(int width, int height)
		{
			Vector3 p = Camera.Transform.WorldTranslation;
			Quaternion q = Camera.Transform.WorldRotationQuat;
			return new PinholeCamera
			{
				Position = new Num.Vector3(p.X, p.Y, p.Z),
				Rotation = Num.Quaternion.Normalize(new Num.Quaternion(q.X, q.Y, q.Z, q.W)),
				VerticalFovDegrees = m_Component?.VerticalFOV ?? VisionRig.VerticalFovDegrees,   // degrees (AgentDocs components/Camera.md)
				Width = width,
				Height = height,
			};
		}

		// One capture. Wall-clock time is measured for the report only; nothing waits on it (rule 6).
		public bool TryCapture(out byte[] rgba, out int width, out int height, out double milliseconds)
		{
			var sw = Stopwatch.StartNew();
			bool ok;
			uint w = 0, h = 0;
			try
			{
				ok = CaptureBridge.TryCaptureCameraRgba(Camera.ID, (uint)RequestedWidth, (uint)RequestedHeight, out rgba, out w, out h);
			}
			catch (Exception e)
			{
				Log.Error($"[Vision] CaptureBridge.TryCaptureCameraRgba threw {e.GetType().Name}: {e.Message}");
				rgba = Array.Empty<byte>();
				ok = false;
			}
			sw.Stop();
			milliseconds = sw.Elapsed.TotalMilliseconds;
			width = (int)w;
			height = (int)h;
			rgba ??= Array.Empty<byte>();
			return ok && width > 0 && height > 0 && rgba.Length >= width * height * 4;
		}
	}
}
