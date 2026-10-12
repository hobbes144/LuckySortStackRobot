using System;
using System.Collections.Generic;
using System.Numerics;
using LuckySortStack.Core;
using LuckySortStack.Robot;

namespace LuckySortStack.Planning
{
	// What rests on what, read from live positions (cluttered starts). The planner may use task metadata for *what* is
	// wanted, but where things are has to be observed: a pile can shift, and a push can move a cube onto another.
	public static class SupportGraph
	{
		public const float HeightTolerance = 0.012f;

		// The cube resting directly on `below`: its centre is over below's top face and it sits one half-edge pair higher.
		public static SortObjectDefinition? On(SortObjectDefinition below, IReadOnlyList<SortObjectDefinition> objects, IWorldState world)
		{
			Vector3 b = world.GetObjectPosition(below.PoolTag);
			foreach (SortObjectDefinition o in objects)
			{
				if (o == below) continue;
				Vector3 p = world.GetObjectPosition(o.PoolTag);
				float dx = p.X - b.X, dz = p.Z - b.Z;
				if (MathF.Sqrt(dx * dx + dz * dz) >= below.Edge / 2f) continue;
				float expected = b.Y + (below.Edge + o.Edge) / 2f;
				if (MathF.Abs(p.Y - expected) <= HeightTolerance) return o;
			}
			return null;
		}

		// The top of the column above `obj` (not obj itself), or null if nothing is on it.
		public static SortObjectDefinition? Topmost(SortObjectDefinition obj, IReadOnlyList<SortObjectDefinition> objects, IWorldState world)
		{
			SortObjectDefinition? top = null;
			SortObjectDefinition current = obj;
			for (int guard = 0; guard < objects.Count; guard++)
			{
				SortObjectDefinition? next = On(current, objects, world);
				if (next == null) break;
				top = current = next;
			}
			return top;
		}
	}
}
