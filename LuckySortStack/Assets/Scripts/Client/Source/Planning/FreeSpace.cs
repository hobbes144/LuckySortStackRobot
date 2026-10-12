using System;
using System.Collections.Generic;
using System.Numerics;
using LuckySortStack.Core;
using LuckySortStack.Robot;

namespace LuckySortStack.Planning
{
	// Where on the table a cube can be set aside while unstacking (cluttered starts). No reserved pads: the whole mat is
	// searched against where every cube is *now*, so the robot uses whatever space is actually open.
	//
	// A spot is free when, on a 1 cm grid over the mat:
	//  - the cube's footprint (bounding circle, any yaw) stays inside the mat with a margin,
	//  - it's inside the arm's reach annulus,
	//  - it keeps `Clearance` from every other cube's footprint (the same rule the generator spaces cubes by, plus a little,
	//    so the open fingers fit later; grasp-axis choice handles the rest), and
	//  - it stays clear of every stack base, so stacks are never crowded.
	// Among free spots it takes the one nearest `near` (short arm travel). Deterministic, so a replayed seed repeats it.
	public static class FreeSpace
	{
		public const float GridStep = 0.01f;

		public sealed class Settings
		{
			public float Clearance = 0.03f;       // gap between footprints
			public float StackKeepOut = 0.11f;    // spot centre to any stack base, minimum
			public float EdgeMargin = 0.01f;      // footprint to mat edge
		}

		public static Vector2? Find(SortObjectDefinition cube, IReadOnlyList<SortObjectDefinition> objects, IWorldState world,
			WorkspaceSpec ws, IEnumerable<Vector3> stackBases, Vector2 near, Settings? settings = null)
		{
			settings ??= new Settings();
			float own = Footprint(cube.Edge);

			// Snapshot the obstacles once: centre (XZ) + required distance from the new cube's centre.
			var obstacles = new List<(Vector2 Centre, float MinDist)>();
            // The cube itself is not an obstacle, because it will be moved away before the new cube is placed.
            foreach (SortObjectDefinition o in objects)
			{
				if (o == cube) continue;
				Vector3 p = world.GetObjectPosition(o.PoolTag);
				obstacles.Add((new Vector2(p.X, p.Z), own + Footprint(o.Edge) + settings.Clearance));
			}
            // Stack bases are always obstacles, even if the stack is empty, because the new cube must not crowd the stack.
            foreach (Vector3 b in stackBases)
				obstacles.Add((new Vector2(b.X, b.Z), settings.StackKeepOut));

            // Search the mat on a grid for the nearest free spot. The grid is aligned with the mat, so the search is deterministic.
            float inset = own + settings.EdgeMargin;
			Vector2? best = null;
			float bestDist = float.MaxValue;
			for (float x = ws.MatMinX + inset; x <= ws.MatMaxX - inset + 1e-6f; x += GridStep)
				for (float z = ws.MatMinZ + inset; z <= ws.MatMaxZ - inset + 1e-6f; z += GridStep)
				{
					float r = MathF.Sqrt(x * x + z * z);
					if (r < ws.MinReach || r > ws.MaxReach) continue;
					var spot = new Vector2(x, z);
					float d = Vector2.Distance(spot, near);
					if (d >= bestDist) continue;
					bool free = true;
					foreach (var (centre, minDist) in obstacles)
						if (Vector2.Distance(spot, centre) < minDist) { free = false; break; }
					if (!free) continue;
					best = spot;
					bestDist = d;
				}
			return best;
		}

		public static float Footprint(float edge) => edge * 0.5f * MathF.Sqrt(2f);
	}
}
