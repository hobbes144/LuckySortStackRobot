using System;
using System.Collections.Generic;
using System.Numerics;
using LuckySortStack.Core;
using LuckySortStack.Robot;
using LuckySortStack.StateMachine;

namespace LuckySort.Core.Tests
{
	// Minimal kinematic stand-in for the arm + MuJoCo: the gripper moves linearly to
	// its target over the commanded duration; closing near an object picks it up;
	// opening drops it onto whatever is below. Descending onto a cube with the
	// fingers misaligned shoves it, like the real collision. Faults can be injected.
	public sealed class FakeSim : IManipulator, IWorldState
	{
		public sealed class Body
		{
			public Vector3 Position;
			public float Yaw;
			public float Tilt;   // only set by tests; the fake never tips cubes itself
			public float Edge;
		}

		public readonly Dictionary<string, Body> Bodies = new();
		public float SurfaceY = 0.01f;
		public float MaxReach = 0.80f;   // targets beyond this horizontal radius are clamped

		// Physical yaw = YawSign * commanded + YawOffset: the convention the state machine must learn.
		public float YawSign = 1f;
		public float YawOffset = 0f;
		public float MaxMisalignment = 10f * MathF.PI / 180f;   // fingers clear the cube within this
		public float FingerOpening = 0.08f;                     // Panda: 40 mm per finger

		// Steady-state tracking error: the grasp site settles this far from where it was commanded.
		public Vector3 TrackingBias = Vector3.Zero;

		// Fault injection.
		public readonly Dictionary<string, int> GraspMisses = new();             // tag → number of grasps that slip
		public readonly Dictionary<string, Vector3> DropOffsetOnce = new();      // tag → offset applied on next drop
		public readonly Dictionary<string, Vector3> PushOnce = new();            // tag → shove on next descent onto it
		public Action<string>? OnDropped;
		// (target, commanded yaw) → true if the arm can't reach this pose in that wrist configuration: it stops 50 mm
		// above the target, like the hover that stalled near the robot base on seed 5594.
		public Func<Vector3, float, bool>? MoveBlocked;
		// Open fingers that come down on a neighbouring cube shove it, and deflect the hand into the target
		// (the 50-seed benchmark's failure mode). Off by default so older tests keep their exact behaviour.
		public bool FingersHitNeighbours;
		public float FingerReach = 0.05f;
		public float FingerHalfWidth = 0.012f;
		public int Collisions { get; private set; }

		public string? Held { get; private set; }
		private float m_HeldYawOffset;
		private Vector3 m_Gripper;
		private Vector3 m_From, m_To;
		private float m_YawFrom, m_YawTo, m_Yaw;
		private float m_Duration, m_Elapsed;
		private float m_Open = 1f;

		public FakeSim(TaskDefinition task, Vector3 start)
		{
			foreach (var o in task.Objects)
				Bodies[o.PoolTag] = new Body { Position = o.InitialPosition, Yaw = o.InitialYaw, Edge = o.Edge };
			// Objects spawn slightly above the surface; settle them like physics would.
			foreach (var b in Bodies.Values)
				b.Position = new Vector3(b.Position.X, SurfaceY + b.Edge / 2f, b.Position.Z);
			m_Gripper = m_From = m_To = start;
		}

		public Vector3 GripperPosition => m_Gripper;
		public float GripperYaw => m_Yaw;
		// Fingers open and close along the gripper's physical yaw direction.
		public Vector3? FingerAxis => m_Open >= 0.5f ? new Vector3(MathF.Sin(m_Yaw), 0f, MathF.Cos(m_Yaw)) : null;

		public void MoveTo(Vector3 position, float yaw, float durationSeconds)
		{
			float r = MathF.Sqrt(position.X * position.X + position.Z * position.Z);
			if (r > MaxReach)
				position = new Vector3(position.X * MaxReach / r, position.Y, position.Z * MaxReach / r);
			if (MoveBlocked?.Invoke(position, yaw) == true)
				position += new Vector3(0f, 0.05f, 0f);
			m_From = m_Gripper;
			m_To = position + TrackingBias;
			m_YawFrom = m_Yaw;
			m_YawTo = YawSign * yaw + YawOffset;
			m_Duration = Math.Max(durationSeconds, 1e-4f);
			m_Elapsed = 0f;
		}

		public void SetGripper(float openFraction) => m_Open = openFraction;

		public Vector3 GetObjectPosition(string tag) => Bodies[tag].Position;
		public float GetObjectYaw(string tag) => Bodies[tag].Yaw;
		public float GetObjectTilt(string tag) => Bodies[tag].Tilt;

		public void Step(float dt)
		{
			m_Elapsed += dt;
			float t = Math.Min(1f, m_Elapsed / m_Duration);
			m_Gripper = Vector3.Lerp(m_From, m_To, t);
			m_Yaw = m_YawFrom + (m_YawTo - m_YawFrom) * t;

			if (Held == null)
				CheckDescentContact();
			if (Held == null && FingersHitNeighbours && m_Open >= 0.5f)
				CheckFingerNeighbourContact();

			if (Held == null && m_Open < 0.5f)
				TryGrasp();
			else if (Held != null && m_Open >= 0.5f)
				Drop();

			if (Held != null)
			{
				Bodies[Held].Position = m_Gripper;
				Bodies[Held].Yaw = m_Yaw + m_HeldYawOffset;
			}
		}

		// Open fingers reaching the top of a cube they don't straddle push it aside: either the
		// gripper is twisted relative to the cube, or off-centre by more than the finger clearance.
		private void CheckDescentContact()
		{
			foreach (var (tag, body) in Bodies)
			{
				bool fingersAtCube = m_Gripper.Y < body.Position.Y + body.Edge / 2f + 0.01f && m_Gripper.Y > body.Position.Y - body.Edge / 2f;
				Vector3 offset = new Vector3(m_Gripper.X - body.Position.X, 0f, m_Gripper.Z - body.Position.Z);
				bool near = offset.Length() < (body.Edge + FingerOpening) / 2f;
				if (!fingersAtCube || !near) continue;

				if (PushOnce.Remove(tag, out Vector3 shove))
				{
					body.Position += shove;
					Collisions++;
				}
				else if (MathF.Abs(TaskStateMachine.WrapQuarter(m_Yaw - body.Yaw)) > MaxMisalignment)
				{
					body.Position += new Vector3(0.012f, 0f, 0f);
					Collisions++;
				}
				else if (offset.Length() > (FingerOpening - body.Edge) / 2f)
				{
					body.Position -= Vector3.Normalize(offset) * 0.012f;   // finger lands on the near edge
					Collisions++;
				}
			}
		}

		// A finger tip inside a cube's (rotated) square footprint, at a height where it overlaps the cube, is a hit.
		// The cube the fingers straddle is never hit this way: its faces are inside the fingers.
		private void CheckFingerNeighbourContact()
		{
			Vector3 axis = FingerAxis!.Value;
			foreach (var (tag, body) in Bodies)
			{
				if (m_Gripper.Y > body.Position.Y + body.Edge / 2f + 0.005f) continue;
				foreach (float side in new[] { 1f, -1f })
				{
					Vector3 finger = m_Gripper + side * FingerReach * axis;
					Vector3 d = new Vector3(finger.X - body.Position.X, 0f, finger.Z - body.Position.Z);
					float lx = d.X * MathF.Cos(body.Yaw) - d.Z * MathF.Sin(body.Yaw);
					float lz = d.X * MathF.Sin(body.Yaw) + d.Z * MathF.Cos(body.Yaw);
					float half = body.Edge / 2f + FingerHalfWidth;
					if (MathF.Abs(lx) >= half || MathF.Abs(lz) >= half) continue;

					// Hit: shove the neighbour away from the finger, and the deflected hand nudges the cube under it.
					Vector3 away = d.Length() > 1e-6f ? Vector3.Normalize(d) : new Vector3(1f, 0f, 0f);
					body.Position += away * 0.012f;
					Collisions++;
					foreach (var (otherTag, other) in Bodies)
					{
						if (otherTag == tag) continue;
						Vector3 g = new Vector3(other.Position.X - m_Gripper.X, 0f, other.Position.Z - m_Gripper.Z);
						if (g.Length() < other.Edge) { other.Position -= away * 0.010f; break; }
					}
					return;
				}
			}
		}

		private void TryGrasp()
		{
			foreach (var (tag, body) in Bodies)
			{
				if (Vector3.Distance(body.Position, m_Gripper) > 0.6f * body.Edge) continue;
				if (GraspMisses.TryGetValue(tag, out int misses) && misses > 0)
				{
					GraspMisses[tag] = misses - 1;
					m_Open = 1f;   // slipped: behave as if nothing was caught this close
					return;
				}
				Held = tag;
				m_HeldYawOffset = body.Yaw - m_Yaw;
				return;
			}
		}

		private void Drop()
		{
			string tag = Held!;
			Held = null;
			Body body = Bodies[tag];
			Vector3 p = body.Position;
			if (DropOffsetOnce.Remove(tag, out Vector3 offset)) p += offset;

			float support = SurfaceY;
			foreach (var (other, ob) in Bodies)
			{
				if (other == tag) continue;
				float reach = (ob.Edge + body.Edge) / 2f;
				if (MathF.Abs(ob.Position.X - p.X) < reach && MathF.Abs(ob.Position.Z - p.Z) < reach && ob.Position.Y < p.Y)
					support = Math.Max(support, ob.Position.Y + ob.Edge / 2f);
			}
			body.Position = new Vector3(p.X, support + body.Edge / 2f, p.Z);
			OnDropped?.Invoke(tag);
		}
	}
}
