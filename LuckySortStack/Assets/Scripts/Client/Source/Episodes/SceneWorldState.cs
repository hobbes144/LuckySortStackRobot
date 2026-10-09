using System.Collections.Generic;
using Hazel;
using LuckySortStack.Robot;
using Num = System.Numerics;

namespace LuckySortStack.Episodes
{
	// IWorldState backed by scene entities. MuJoCo writes body poses back to the
	// entity transforms every step, so these reads are the physical truth.
	public sealed class SceneWorldState : IWorldState
	{
		private readonly IReadOnlyDictionary<string, Entity> m_Objects;

		public SceneWorldState(IReadOnlyDictionary<string, Entity> objectsByTag)
		{
			m_Objects = objectsByTag;
		}

		public Num.Vector3 GetObjectPosition(string tag)
		{
			Vector3 p = m_Objects[tag].Transform.WorldTranslation;
			return new Num.Vector3(p.X, p.Y, p.Z);
		}

		// Yaw from the quaternion, not Euler.Y: equivalent Euler triples can report a flipped Y.
		public float GetObjectYaw(string tag)
		{
			Vector3 forward = m_Objects[tag].Transform.WorldRotationQuat * new Vector3(0f, 0f, 1f);
			return Mathf.Atan2(forward.X, forward.Z);
		}

		public float GetObjectTilt(string tag)
		{
			Quaternion q = m_Objects[tag].Transform.WorldRotationQuat;
			float up = 0f;
			foreach (Vector3 axis in new[] { new Vector3(1f, 0f, 0f), new Vector3(0f, 1f, 0f), new Vector3(0f, 0f, 1f) })
				up = System.Math.Max(up, System.Math.Abs((q * axis).Y));
			return System.MathF.Acos(System.Math.Min(1f, up));
		}
	}
}
