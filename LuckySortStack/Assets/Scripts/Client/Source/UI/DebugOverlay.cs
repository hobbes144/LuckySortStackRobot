using System;
using Hazel;
using LuckySortStack.Core;
using LuckySortStack.Planning;
using LuckySortStack.Presentation;
using LuckySortStack.StateMachine;
using Num = System.Numerics;

namespace LuckySortStack.UI
{
	public sealed class DebugOverlaySettings
	{
		public float Distance = 0.6f;          // metres in front of the camera (must be beyond the near clip)
		public float LineHeight = 0.022f;      // one text line as a fraction of the screen height
		public float Margin = 0.02f;           // gap from the top-left corner, fraction of the screen height
		public float FallbackAspect = 16f / 9f; // width/height used if the viewport aspect can't be measured
		public bool DrawMarkers = true;
	}

	// Presentation overlay (plan §20, Milestone 8). Lucky's C# API has no screen-space UI, so the HUD is three
	// TextComponent entities parented to the scene camera (a camera child stays fixed on screen), and the
	// in-world markers are DebugRenderer lines drawn every frame from OnLateUpdate.
	// Confirmed APIs (Hazel-ScriptCore metadata): static Scene.CreateEntity / Scene.DestroyEntity, CreateComponent<T>, TextComponent.Text/Color,
	// Entity.Parent (settable), Transform.Translation/Scale, DebugRenderer.DrawLine/LineWidth.
	// Unconfirmed, so exposed as settings: text size per unit of scale, the text's anchor, and which way it faces.
	public sealed class DebugOverlay
	{
		private static readonly Vector4 White  = new Vector4(1f, 1f, 1f, 1f);
		private static readonly Vector4 Grey   = new Vector4(0.75f, 0.75f, 0.75f, 1f);
		private static readonly Vector4 Green  = new Vector4(0.30f, 0.95f, 0.40f, 1f);
		private static readonly Vector4 Red    = new Vector4(1f, 0.35f, 0.30f, 1f);
		private static readonly Vector4 Amber  = new Vector4(1f, 0.80f, 0.25f, 1f);
		private static readonly Vector4 Cyan   = new Vector4(0.30f, 0.85f, 1f, 1f);
		private static readonly Vector4 Pink   = new Vector4(1f, 0.40f, 0.90f, 1f);

		// Measured from the 2026-10-12 screenshot: one text line is ~0.96 x the entity scale, the text's anchor is its
		// left edge, and it sits roughly on the first line's centre.
		private const float LinePerScale = 0.96f;

		private readonly DebugOverlaySettings m_Settings;
		private readonly Entity? m_Camera;
		private readonly CameraComponent? m_CameraComponent;
		private readonly Entity? m_Title, m_Status, m_Body;
		private float m_LastAspect = -1f;
		private bool m_AspectLogged;

		public DebugOverlay(Entity? camera, DebugOverlaySettings settings)
		{
			m_Settings = settings;
			m_Camera = camera;
			m_CameraComponent = camera?.GetComponent<CameraComponent>();
			m_Title  = CreateText("LS_Overlay_Title",  camera, White);
			m_Status = CreateText("LS_Overlay_Status", camera, Grey);
			m_Body   = CreateText("LS_Overlay_Body",   camera, White);
			if (camera == null)
				Log.Warn("[Overlay] Camera not found: the overlay text sits at the world origin. Set EpisodeManager → Overlay → CameraTag.");
			Layout();
		}

		// Anchor the panel to the top-left of the screen. Camera-local space looks down −Z, so at distance d the visible
		// half-height is d·tan(fov/2) and the half-width is that times the viewport aspect. Re-run on every refresh so a
		// resized viewport or a changed FOV keeps the panel in the corner.
		public void Layout()
		{
			float d = m_Settings.Distance;
			float fovDeg = m_CameraComponent?.VerticalFOV ?? 60f;          // degrees (AgentDocs components/Camera.md)
			float halfH = d * Mathf.Tan(0.5f * fovDeg * Mathf.PI / 180f);
			float aspect = MeasureAspect(halfH) ?? m_Settings.FallbackAspect;
			float screenH = 2f * halfH;

			float line = m_Settings.LineHeight * screenH;
			float scale = line / LinePerScale;
			float left = -halfH * aspect + m_Settings.Margin * screenH;
			float top = halfH - m_Settings.Margin * screenH - 0.5f * line;

			Place(m_Title,  left, top, scale);
			Place(m_Status, left, top - 1.2f * line, scale);
			Place(m_Body,   left, top - 2.6f * line, scale);
		}

		private void Place(Entity? e, float x, float y, float scale)
		{
			if (e == null) return;
			e.Transform.Translation = new Vector3(x, y, -m_Settings.Distance);
			e.Transform.Scale = new Vector3(scale, scale, scale);
		}

		// The C# API has no viewport-size query, so project three camera-local points through CameraComponent.ToScreenSpace:
		// the centre, half a screen-height up, and half a screen-height right. Its output units aren't documented, so both
		// readings are handled: with square-pixel coordinates "up" and "right" land the same distance from the centre and
		// aspect = centre.x / that distance (origin at the left edge); with coordinates normalised per axis, aspect = up / right.
		private float? MeasureAspect(float halfH)
		{
			if (m_Camera == null || m_CameraComponent == null) return null;
			Vector3 origin = m_Camera.Transform.WorldTranslation;
			Quaternion rot = m_Camera.Transform.WorldRotationQuat;
			float d = m_Settings.Distance;
			Vector2 c = m_CameraComponent.ToScreenSpace(origin + rot * new Vector3(0f, 0f, -d));
			Vector2 u = m_CameraComponent.ToScreenSpace(origin + rot * new Vector3(0f, halfH, -d));
			Vector2 r = m_CameraComponent.ToScreenSpace(origin + rot * new Vector3(halfH, 0f, -d));
			float du = Mathf.Sqrt((u.X - c.X) * (u.X - c.X) + (u.Y - c.Y) * (u.Y - c.Y));
			float dr = Mathf.Sqrt((r.X - c.X) * (r.X - c.X) + (r.Y - c.Y) * (r.Y - c.Y));

			float? aspect = null;
			if (du > 1e-6f && dr > 1e-6f)
				aspect = Mathf.Abs(du - dr) < 0.05f * du ? c.X / du : du / dr;
			if (aspect is float a && (a < 0.5f || a > 5f)) aspect = null;

			if (!m_AspectLogged || (aspect is float b && Mathf.Abs(b - m_LastAspect) > 0.05f))
			{
				Log.Info($"[Overlay] ToScreenSpace: centre ({c.X:F3}, {c.Y:F3}), up ({u.X:F3}, {u.Y:F3}), right ({r.X:F3}, {r.Y:F3}) " +
					$"→ aspect {(aspect.HasValue ? aspect.Value.ToString("F2") : $"unmeasurable, using {m_Settings.FallbackAspect:F2}")}");
				m_AspectLogged = true;
				m_LastAspect = aspect ?? m_LastAspect;
			}
			return aspect;
		}

		private Entity? CreateText(string name, Entity? camera, Vector4 color)
		{
			// Static in this engine version (2026.1.2); the AgentDocs' Scene.Instance doesn't exist.
			Entity? e = Scene.CreateEntity(name);
			TextComponent? text = e?.CreateComponent<TextComponent>();
			if (e == null || text == null)
			{
				Log.Error($"[Overlay] Could not create text entity '{name}'.");
				return null;
			}
			text.Color = color;
			text.Text = "";
			if (camera != null) e.Parent = camera;
			return e;
		}

		public void Show(OverlayText t)
		{
			SetText(m_Title, t.Title);
			SetText(m_Status, t.Status, t.Tone switch
			{
				OverlayTone.Success => Green,
				OverlayTone.Failure => Red,
				OverlayTone.Running => Amber,
				_ => Grey,
			});
			SetText(m_Body, t.Body);
		}

		private static void SetText(Entity? e, string value, Vector4? color = null)
		{
			TextComponent? text = e?.GetComponent<TextComponent>();
			if (text == null) return;
			if (text.Text != value) text.Text = value;
			if (color.HasValue) text.Color = color.Value;
		}

		// World markers, every frame: stack bases (the active one brighter), the current target cube, the arm's
		// commanded target, the line from the gripper to it, and the chosen finger axis (where the open fingers come down).
		public void DrawMarkers(TaskDefinition task, TaskStateMachine? machine, Func<string, Num.Vector3> objectPos,
			Func<string, float> objectYaw, Num.Vector3 gripper)
		{
			if (!m_Settings.DrawMarkers) return;
			DebugRenderer.LineWidth = 2f;

			PlannedStep? step = machine != null && !machine.IsFinished ? machine.CurrentStep : null;
			for (int s = 0; s < task.Targets.Count; s++)
			{
				StackTarget target = task.Targets[s];
				bool active = step != null && step.StackIndex == s;
				Vector4 c = ColorOf(target.Color, active ? 1f : 0.45f);
				Square(ToHz(target.BasePosition) + new Vector3(0f, 0.002f, 0f), 0f, active ? 0.09f : 0.075f, c);
			}

			if (step == null || machine == null) return;

			string tag = step.Object.PoolTag;
			Vector3 obj = ToHz(objectPos(tag));
			float yaw = objectYaw(tag);
			Square(obj + new Vector3(0f, step.Object.Edge / 2f + 0.003f, 0f), yaw, step.Object.Edge + 0.012f, Amber);

			Vector3 moveTarget = ToHz(machine.MoveTarget);
			Cross(moveTarget, 0.02f, Cyan);
			DebugRenderer.DrawLine(ToHz(gripper), moveTarget, new Vector4(Cyan.X, Cyan.Y, Cyan.Z, 0.5f));

			if (machine.WantedFingerAxis is Num.Vector3 axis)
			{
				Vector3 a = ToHz(axis) * machine.FingerReach;
				Vector3 top = obj + new Vector3(0f, step.Object.Edge / 2f + 0.006f, 0f);
				DebugRenderer.DrawLine(top - a, top + a, Pink);
				Cross(top + a, 0.012f, Pink);
				Cross(top - a, 0.012f, Pink);
			}
		}

		public void Destroy()
		{
			foreach (Entity? e in new[] { m_Title, m_Status, m_Body })
				if (e != null) Scene.DestroyEntity(e);
		}

		// ── Drawing helpers (world space, Hazel Y-up) ──────────────────────

		private static void Square(Vector3 centre, float yaw, float size, Vector4 color)
		{
			float h = size / 2f;
			Vector3 fwd = new Vector3(Mathf.Sin(yaw), 0f, Mathf.Cos(yaw)) * h;
			Vector3 right = new Vector3(Mathf.Cos(yaw), 0f, -Mathf.Sin(yaw)) * h;
			Vector3 p0 = centre + fwd + right, p1 = centre + fwd - right, p2 = centre - fwd - right, p3 = centre - fwd + right;
			DebugRenderer.DrawLine(p0, p1, color);
			DebugRenderer.DrawLine(p1, p2, color);
			DebugRenderer.DrawLine(p2, p3, color);
			DebugRenderer.DrawLine(p3, p0, color);
		}

		private static void Cross(Vector3 p, float size, Vector4 color)
		{
			float h = size / 2f;
			DebugRenderer.DrawLine(p - new Vector3(h, 0f, 0f), p + new Vector3(h, 0f, 0f), color);
			DebugRenderer.DrawLine(p - new Vector3(0f, h, 0f), p + new Vector3(0f, h, 0f), color);
			DebugRenderer.DrawLine(p - new Vector3(0f, 0f, h), p + new Vector3(0f, 0f, h), color);
		}

		private static Vector4 ColorOf(ObjectColor color, float intensity)
		{
			Vector4 c = color switch
			{
				ObjectColor.Red    => new Vector4(1f, 0.2f, 0.2f, 1f),
				ObjectColor.Blue   => new Vector4(0.25f, 0.45f, 1f, 1f),
				ObjectColor.Green  => new Vector4(0.25f, 0.9f, 0.3f, 1f),
				ObjectColor.Yellow => new Vector4(1f, 0.9f, 0.2f, 1f),
				_ => White,
			};
			return new Vector4(c.X * intensity, c.Y * intensity, c.Z * intensity, 1f);
		}

		private static Vector3 ToHz(Num.Vector3 v) => new Vector3(v.X, v.Y, v.Z);
	}
}
