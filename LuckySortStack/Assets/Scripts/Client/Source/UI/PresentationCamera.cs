using System;
using Hazel;
using LuckySortStack.Presentation;
using Num = System.Numerics;

namespace LuckySortStack.UI
{
	// Clean demo camera (Milestone 8): a few framed views of the workspace with smooth transitions. Drives the scene's
	// own camera entity, so the overlay (parented to it) comes along. Eye and look-at point are blended separately and
	// the rotation is rebuilt with CameraViews.LookAtEuler every frame, so there's never any roll.
	public sealed class PresentationCamera
	{
		private readonly Entity m_Camera;
		private readonly Num.Vector3 m_AuthoredEye, m_AuthoredTarget;
		private Num.Vector3 m_Eye, m_Target;
		private bool m_Moved;

		public CameraView View { get; private set; }
		public float Sharpness = 4f;   // higher = snappier transitions

		public PresentationCamera(Entity camera, CameraView initial)
		{
			m_Camera = camera;
			Vector3 p = camera.Transform.Translation;
			Vector3 r = camera.Transform.Rotation;
			m_AuthoredEye = new Num.Vector3(p.X, p.Y, p.Z);
			m_AuthoredTarget = m_AuthoredEye + CameraViews.Forward(new Num.Vector3(r.X, r.Y, r.Z));
			m_Eye = m_AuthoredEye;
			m_Target = m_AuthoredTarget;
			View = initial;
		}

		public void Next()
		{
			View = (CameraView)(((int)View + 1) % CameraViews.Count);
			Log.Info($"[Camera] View: {View}");
		}

		// focus: what the Follow view tracks (the cube being picked, or the gripper).
		public void Update(float dt, Num.Vector3 focus)
		{
			Num.Vector3 eye, target;
			if (View == CameraView.Authored) { eye = m_AuthoredEye; target = m_AuthoredTarget; }
			else (eye, target) = CameraViews.Pose(View, focus);

			// Untouched until the first non-authored view, so the scene's camera stays exactly as saved by default.
			if (View == CameraView.Authored && !m_Moved) return;
			m_Moved = true;

			float k = CameraViews.Blend(dt, Sharpness);
			m_Eye += (eye - m_Eye) * k;
			m_Target += (target - m_Target) * k;
			Num.Vector3 euler = CameraViews.LookAtEuler(m_Eye, m_Target);
			m_Camera.Transform.Translation = new Vector3(m_Eye.X, m_Eye.Y, m_Eye.Z);
			m_Camera.Transform.Rotation = new Vector3(euler.X, euler.Y, euler.Z);
		}
	}
}
