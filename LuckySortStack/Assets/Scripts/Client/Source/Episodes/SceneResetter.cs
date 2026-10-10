using System;
using System.Collections.Generic;
using Hazel;

namespace LuckySortStack.Episodes
{
	// Puts the scene back to an episode start while Play keeps running, so benchmark mode can run
	// many episodes in one session. Between episodes only — the robot never teleports anything
	// during an episode (rule 7).
	//
	// 1. MujocoSceneComponent.ResetToInitialState(preserveTime: true) restores the MuJoCo state
	//    (robot joints, velocities) to the start of Play.
	// 2. Each pool cube's free joint qpos is written directly (SetQposByIndex) with the new layout.
	//
	// Nothing about cube body naming or the Hazel↔MuJoCo frame is documented, so both are measured:
	//  - Cube body ↔ entity: each free-joint body's Hazel-space position is matched to the nearest
	//    pool entity (they coincide to well under a millimetre).
	//  - Frame: mj = R·hz + t with R = GetAxisHzToMj() or its inverse (direction undocumented, so both are
	//    tried); t is measured per body from
	//    GetPosition(id, hazelSpace: false/true), and its spread across bodies is logged as a check.
	//  - qpos layout: a free joint is [x y z  qw qx qy qz]. The first three are checked against the
	//    body's MuJoCo-space position at calibration time.
	// Cubes look identical under 90° turns, so orientation only needs the yaw about the vertical:
	// qpos quat = rotation by yaw about the MuJoCo-space image of Hazel's up axis.
	public sealed class SceneResetter
	{
		private struct Body
		{
			public uint Id;
			public uint QposAdr;
		}

		private readonly MujocoSceneComponent m_Mujoco;
		private readonly Dictionary<string, Body> m_Bodies = new Dictionary<string, Body>();
		private Quaternion m_HzToMj;
		private Vector3 m_Offset;     // t in mj = R·hz + t

		public bool IsCalibrated { get; private set; }

		public SceneResetter(MujocoSceneComponent mujoco)
		{
			m_Mujoco = mujoco;
		}

		// Call once physics is running and the pool objects are at rest. Returns false (with logs) if
		// any pool entity couldn't be matched to a MuJoCo free body or the frame check is inconsistent.
		public bool Calibrate(IReadOnlyDictionary<string, Entity> pool)
		{
			m_Bodies.Clear();

			// Every body with a free joint is a candidate.
			var candidates = new List<(uint Id, uint QposAdr, Vector3 Hz, Vector3 Mj)>();
			for (uint b = 0; b < m_Mujoco.ModelNbody; b++)
			{
				uint joint = m_Mujoco.GetBodyFreeJointIndex(b);
				if (joint >= m_Mujoco.ModelNjnt || m_Mujoco.GetJointType(joint) != 0) continue;   // 0 = mjJNT_FREE
				candidates.Add((b, m_Mujoco.GetJointQposAdr(joint), m_Mujoco.GetPosition(b, true), m_Mujoco.GetPosition(b, false)));
			}

			var matched = new List<(Vector3 Hz, Vector3 Mj)>();
			float worstQpos = 0f;
			foreach (var kv in pool)
			{
				Vector3 p = kv.Value.Transform.WorldTranslation;
				int best = -1;
				float bestDist = 0.005f;
				for (int i = 0; i < candidates.Count; i++)
				{
					float d = (candidates[i].Hz - p).Length();
					if (d < bestDist) { best = i; bestDist = d; }
				}
				if (best < 0)
				{
					Log.Error($"[Reset] No MuJoCo free body within 5 mm of {kv.Key} at ({p.X:F3}, {p.Y:F3}, {p.Z:F3}); " +
						$"{candidates.Count} free bodies in the model ({m_Mujoco.ModelNbody} bodies total):");
					// Every body, free or not: names are lucky_body_<entity ID>, so each maps back to a scene entity.
					for (uint b = 0; b < m_Mujoco.ModelNbody; b++)
					{
						uint j = m_Mujoco.GetBodyFreeJointIndex(b);
						string joint = j < m_Mujoco.ModelNjnt
							? $"joint {j} '{m_Mujoco.GetJointNameByIndex(j)}' type {m_Mujoco.GetJointType(j)}"
							: $"no free joint (index {j})";
						Vector3 bp = m_Mujoco.GetPosition(b, true);
						Log.Error($"[Reset]   body {b} '{m_Mujoco.GetBodyNameByIndex(b)}' parent {m_Mujoco.GetBodyParentID(b)}, {joint}, " +
							$"mass {m_Mujoco.GetBodyMass(b):G3} at ({bp.X:F3}, {bp.Y:F3}, {bp.Z:F3})");
					}
					return false;
				}

				var c = candidates[best];
				m_Bodies[kv.Key] = new Body { Id = c.Id, QposAdr = c.QposAdr };
				matched.Add((c.Hz, c.Mj));
				Vector3 qposPos = new Vector3(m_Mujoco.GetJointPosition(c.QposAdr), m_Mujoco.GetJointPosition(c.QposAdr + 1), m_Mujoco.GetJointPosition(c.QposAdr + 2));
				worstQpos = Math.Max(worstQpos, (qposPos - c.Mj).Length());
			}

			// Which way GetAxisHzToMj rotates isn't documented: keep whichever of it and its inverse
			// explains every body with one translation.
			Quaternion axis = m_Mujoco.GetAxisHzToMj();
			float spread = float.MaxValue;
			foreach (Quaternion r in new[] { axis, axis.Conjugate })
			{
				var (offset, s) = FitOffset(r, matched);
				if (s < spread) { spread = s; m_HzToMj = r; m_Offset = offset; }
			}

			Log.Info($"[Reset] Calibrated {m_Bodies.Count} cubes: HzToMj=({m_HzToMj.X:F3}, {m_HzToMj.Y:F3}, {m_HzToMj.Z:F3}, {m_HzToMj.W:F3})" +
				$"{(m_HzToMj.Equals(axis) ? "" : " (inverse of GetAxisHzToMj)")} " +
				$"offset=({m_Offset.X:F4}, {m_Offset.Y:F4}, {m_Offset.Z:F4}) spread={spread * 1000f:F2} mm, " +
				$"qpos-vs-body={worstQpos * 1000f:F2} mm");
			if (spread > 0.002f || worstQpos > 0.002f)
			{
				Log.Error("[Reset] Frame calibration is inconsistent (>2 mm); the Hazel↔MuJoCo mapping isn't a single rigid transform. Reset disabled.");
				return false;
			}

			IsCalibrated = true;
			return true;
		}

		// Reset MuJoCo, then write every pool cube's pose (Hazel-space position + yaw in radians).
		public void Reset(IReadOnlyDictionary<string, (Vector3 Position, float Yaw)> poses)
		{
			if (!IsCalibrated) throw new InvalidOperationException("Calibrate() first");

			// Pre-reset poses, for diagnosing a cube that doesn't end up where it was written.
			var before = new Dictionary<string, Vector3>();
			foreach (var kv in m_Bodies) before[kv.Key] = m_Mujoco.GetPosition(kv.Value.Id, true);

			m_Mujoco.ResetToInitialState(true);

			Vector3 up = m_HzToMj * new Vector3(0f, 1f, 0f);
			float worstReadback = 0f;
			string worstTag = "";
			foreach (var kv in poses)
			{
				if (!m_Bodies.TryGetValue(kv.Key, out Body body)) continue;
				Vector3 mj = (m_HzToMj * kv.Value.Position) + m_Offset;
				float half = 0.5f * kv.Value.Yaw;
				float s = Mathf.Sin(half);
				float[] q = { mj.X, mj.Y, mj.Z, Mathf.Cos(half), up.X * s, up.Y * s, up.Z * s };

				for (uint i = 0; i < 7; i++) m_Mujoco.SetQposByIndex(body.QposAdr + i, q[i]);

				// Read the qpos back: a mismatch here means the write itself is wrong, not the physics afterwards.
				float err = 0f;
				for (uint i = 0; i < 7; i++) err = Math.Max(err, Math.Abs(m_Mujoco.GetJointPosition(body.QposAdr + i) - q[i]));
				if (err > worstReadback) { worstReadback = err; worstTag = kv.Key; }
			}
			Log.Info($"[Reset] Wrote {poses.Count} cube poses; worst qpos readback error {worstReadback:G3}{(worstTag.Length > 0 ? $" ({worstTag})" : "")}");
			LastPreResetPositions = before;
		}

		// Where each cube was (Hazel space) just before the last Reset(). Used to explain placement failures.
		public IReadOnlyDictionary<string, Vector3> LastPreResetPositions { get; private set; } = new Dictionary<string, Vector3>();

		// Mean translation t for mj = r·hz + t, and the largest deviation from it.
		private static (Vector3 Offset, float Spread) FitOffset(Quaternion r, List<(Vector3 Hz, Vector3 Mj)> matched)
		{
			Vector3 mean = new Vector3(0f, 0f, 0f);
			foreach (var m in matched) mean += m.Mj - (r * m.Hz);
			mean *= 1f / matched.Count;
			float spread = 0f;
			foreach (var m in matched) spread = Math.Max(spread, (m.Mj - (r * m.Hz) - mean).Length());
			return (mean, spread);
		}
	}
}
