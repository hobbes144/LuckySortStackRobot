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
	//    (robot joints, velocities, actuator controls). Only between episodes: on the first frame of
	//    Play the state is already initial, and resetting there also wipes the arm's actuator targets.
	// 2. Each pool cube's free joint qpos is written directly (SetQposByIndex) with the new layout.
	//
	// Nothing about cube body naming or the Hazel↔MuJoCo frame is documented, so all of it is measured:
	//  - Cube body ↔ entity: each free-joint body's Hazel-space position is matched to the nearest
	//    pool entity (they coincide to well under a millimetre).
	//  - Position frame: mj = R·hz + t with R = GetAxisHzToMj() or its inverse (direction undocumented, so
	//    both are tried); t is fitted across all cubes and its spread is logged as a check.
	//  - Orientation frame: q_mj = R ⊗ q_hz ⊗ C. C is measured per cube from the saved scene pose
	//    (entity WorldRotationQuat vs the qpos quaternion) and must agree across all cubes.
	//    2026-10-12: assuming q_mj was just "yaw about MuJoCo up" (i.e. C = R⁻¹) put every cube at the
	//    right spot, but every rotated one failed the yaw check (7/16 passed: exactly the yaw-0 parked cubes).
	//  - qpos layout: a free joint is [x y z  qw qx qy qz].
	// Cubes look identical under 90° turns, so the target orientation is an upright cube turned by `yaw`
	// about Hazel's +Y, converted to MuJoCo through R and C.
	public sealed class SceneResetter
	{
		private struct Body
		{
			public uint Id;
			public uint QposAdr;
		}

		// Plain [w x y z] quaternion (MuJoCo's order), Hamilton product. Kept local so the maths doesn't
		// depend on undocumented Hazel.Quaternion operator conventions.
		private readonly struct Q
		{
			public readonly float W, X, Y, Z;
			public Q(float w, float x, float y, float z) { W = w; X = x; Y = y; Z = z; }
			public static Q From(Quaternion q) => new Q(q.W, q.X, q.Y, q.Z);
			public static Q operator *(Q a, Q b) => new Q(
				a.W * b.W - a.X * b.X - a.Y * b.Y - a.Z * b.Z,
				a.W * b.X + a.X * b.W + a.Y * b.Z - a.Z * b.Y,
				a.W * b.Y - a.X * b.Z + a.Y * b.W + a.Z * b.X,
				a.W * b.Z + a.X * b.Y - a.Y * b.X + a.Z * b.W);
			public Q Conj => new Q(W, -X, -Y, -Z);
			public Q Normalized()
			{
				float n = MathF.Sqrt(W * W + X * X + Y * Y + Z * Z);
				return n > 0f ? new Q(W / n, X / n, Y / n, Z / n) : new Q(1f, 0f, 0f, 0f);
			}
			public static float Dot(Q a, Q b) => a.W * b.W + a.X * b.X + a.Y * b.Y + a.Z * b.Z;
			// Angle between two rotations, ignoring the q / −q double cover.
			public static float AngleDeg(Q a, Q b) => 2f * MathF.Acos(Math.Min(1f, Math.Abs(Dot(a, b)))) * 180f / MathF.PI;
			public Vector3 Rotate(Vector3 v)
			{
				// v' = v + 2w(u×v) + 2u×(u×v), u = (x, y, z)
				float tx = 2f * (Y * v.Z - Z * v.Y), ty = 2f * (Z * v.X - X * v.Z), tz = 2f * (X * v.Y - Y * v.X);
				return new Vector3(
					v.X + W * tx + (Y * tz - Z * ty),
					v.Y + W * ty + (Z * tx - X * tz),
					v.Z + W * tz + (X * ty - Y * tx));
			}
			public static Q YawAboutHazelUp(float yaw) => new Q(MathF.Cos(0.5f * yaw), 0f, MathF.Sin(0.5f * yaw), 0f);
			public override string ToString() => $"[w {W:F3}, x {X:F3}, y {Y:F3}, z {Z:F3}]";
		}

		private readonly MujocoSceneComponent m_Mujoco;
		private readonly Dictionary<string, Body> m_Bodies = new Dictionary<string, Body>();
		private Quaternion m_HzToMj;
		private Q m_R;                // m_HzToMj as [w x y z]
		private Q m_C;                // q_mj = R ⊗ q_hz ⊗ C
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

			if (candidates.Count != pool.Count)
			{
				Log.Error($"[Reset] {candidates.Count} MuJoCo free bodies for {pool.Count} pool cubes. " +
					"A pool entity is likely parented under another and compiled into its body instead of getting " +
					"its own free joint (EpisodeManager.ReadWorkspace should have already caught this on a root-entity check).");
				return false;
			}

			var matched = new List<(Vector3 Hz, Vector3 Mj)>();
			var rests = new List<(string Tag, Q Hz, Q Mj)>();
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
				rests.Add((kv.Key, Q.From(kv.Value.Transform.WorldRotationQuat).Normalized(), ReadQposQuat(c.QposAdr)));
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
			m_R = Q.From(m_HzToMj).Normalized();

			Log.Info($"[Reset] Calibrated {m_Bodies.Count} cubes: HzToMj=({m_HzToMj.X:F3}, {m_HzToMj.Y:F3}, {m_HzToMj.Z:F3}, {m_HzToMj.W:F3})" +
				$"{(m_HzToMj.Equals(axis) ? "" : " (inverse of GetAxisHzToMj)")} " +
				$"offset=({m_Offset.X:F4}, {m_Offset.Y:F4}, {m_Offset.Z:F4}) spread={spread * 1000f:F2} mm, " +
				$"qpos-vs-body={worstQpos * 1000f:F2} mm");
			if (spread > 0.002f || worstQpos > 0.002f)
			{
				Log.Error("[Reset] Frame calibration is inconsistent (>2 mm); the Hazel↔MuJoCo mapping isn't a single rigid transform. Reset disabled.");
				return false;
			}

			// Orientation frame, measured on the saved scene poses (no writes, so nothing can be read back stale):
			// C = (R ⊗ q_hz)⁻¹ ⊗ q_mj must be the same rotation for every cube.
			// (The previous write-then-read self-test read GetOrientation in the same frame as the qpos write,
			// before MuJoCo recomputed body poses, so it always saw the old orientation.)
			m_C = ((m_R * rests[0].Hz).Conj * rests[0].Mj).Normalized();
			float worstC = 0f;
			string worstCTag = rests[0].Tag;
			foreach (var r in rests)
			{
				Q ci = ((m_R * r.Hz).Conj * r.Mj).Normalized();
				float a = Q.AngleDeg(ci, m_C);
				if (a > worstC) { worstC = a; worstCTag = r.Tag; }
			}
			string convention =
				Q.AngleDeg(m_C, new Q(1f, 0f, 0f, 0f)) < 1f ? "q_mj = R ⊗ q_hz" :
				Q.AngleDeg(m_C, m_R.Conj) < 1f ? "q_mj = R ⊗ q_hz ⊗ R⁻¹" : "other";
			Log.Info($"[Reset] Orientation frame: C={m_C} ({convention}), worst disagreement across cubes {worstC:F2}° ({worstCTag})");
			if (worstC > 2f)
			{
				Log.Error("[Reset] Orientation calibration is inconsistent (>2°); cube rotations can't be written reliably. Reset disabled.");
				return false;
			}

			IsCalibrated = true;
			return true;
		}

		// Write every pool cube's pose (Hazel-space position + yaw in radians). resetPhysics also calls
		// ResetToInitialState first: needed between episodes (robot back to its start, velocities zeroed),
		// wrong on the first frame of Play, where it only wipes the arm's actuator targets.
		public void Reset(IReadOnlyDictionary<string, (Vector3 Position, float Yaw)> poses, bool resetPhysics)
		{
			if (!IsCalibrated) throw new InvalidOperationException("Calibrate() first");

			// Pre-reset poses, for diagnosing a cube that doesn't end up where it was written.
			var before = new Dictionary<string, Vector3>();
			foreach (var kv in m_Bodies) before[kv.Key] = m_Mujoco.GetPosition(kv.Value.Id, true);

			if (resetPhysics) m_Mujoco.ResetToInitialState(true);

			float worstReadback = 0f;
			string worstTag = "";
			foreach (var kv in poses)
			{
				if (!m_Bodies.TryGetValue(kv.Key, out Body body)) continue;
				Vector3 mj = (m_HzToMj * kv.Value.Position) + m_Offset;
				Q rot = (m_R * Q.YawAboutHazelUp(kv.Value.Yaw) * m_C).Normalized();
				float[] q = { mj.X, mj.Y, mj.Z, rot.W, rot.X, rot.Y, rot.Z };

				for (uint i = 0; i < 7; i++) m_Mujoco.SetQposByIndex(body.QposAdr + i, q[i]);

				// Read the qpos back: a mismatch here means the write itself is wrong, not the physics afterwards.
				float err = 0f;
				for (uint i = 0; i < 7; i++) err = Math.Max(err, Math.Abs(m_Mujoco.GetJointPosition(body.QposAdr + i) - q[i]));
				if (err > worstReadback) { worstReadback = err; worstTag = kv.Key; }
			}
			Log.Info($"[Reset] Wrote {poses.Count} cube poses{(resetPhysics ? " after a MuJoCo reset" : "")}; " +
				$"worst qpos readback error {worstReadback:G3}{(worstTag.Length > 0 ? $" ({worstTag})" : "")}");
			LastPreResetPositions = before;
		}

		// Where each cube was (Hazel space) just before the last Reset(). Used to explain placement failures.
		public IReadOnlyDictionary<string, Vector3> LastPreResetPositions { get; private set; } = new Dictionary<string, Vector3>();

		// The yaw currently in MuJoCo's qpos for this cube, converted to Hazel space through the calibrated
		// frame (q_hz = R⁻¹ ⊗ q_mj ⊗ C⁻¹) and measured the same way SceneWorldState does. Ground truth,
		// independent of the engine's qpos->Transform sync: lets us tell apart "physics changed the rotation"
		// from "the Hazel-side readout of it is wrong".
		public bool TryGetRawQposYawDegrees(string tag, out float yawDeg)
		{
			if (!m_Bodies.TryGetValue(tag, out Body b)) { yawDeg = 0f; return false; }
			Q hz = m_R.Conj * ReadQposQuat(b.QposAdr) * m_C.Conj;
			Vector3 fwd = hz.Rotate(new Vector3(0f, 0f, 1f));
			yawDeg = Mathf.Atan2(fwd.X, fwd.Z) * 180f / Mathf.PI;
			return true;
		}

		private Q ReadQposQuat(uint qposAdr) => new Q(
			m_Mujoco.GetJointPosition(qposAdr + 3), m_Mujoco.GetJointPosition(qposAdr + 4),
			m_Mujoco.GetJointPosition(qposAdr + 5), m_Mujoco.GetJointPosition(qposAdr + 6)).Normalized();

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
