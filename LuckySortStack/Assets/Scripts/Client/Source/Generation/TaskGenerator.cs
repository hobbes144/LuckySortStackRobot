using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using LuckySortStack.Core;

namespace LuckySortStack.Generation
{
	// Generates a solvable TaskDefinition from a seed. Pure and deterministic:
	// every random draw comes from one System.Random(seed), consumed in a fixed order.
	// (Hazel.Random can't be seeded, so it must never be used here.)
	public static class TaskGenerator
	{
		private const int AttemptsPerObject = 200;
		private const int LayoutRestarts = 20;

		public static TaskDefinition Generate(TaskConfig config, WorkspaceSpec workspace)
		{
			config.Validate();
			var rng = new Random(config.Seed);

			var task = new TaskDefinition { Seed = config.Seed, Config = config };

			// 1. Colors: random subset, then sorted so stack indices are stable for a given subset.
			var allColors = (ObjectColor[])Enum.GetValues(typeof(ObjectColor));
			task.Colors = Shuffle(allColors.ToList(), rng).Take(config.NumColors).OrderBy(c => c).ToList();

			// 2. Size tiers in play: the N largest.
			var tiers = ((ObjectSize[])Enum.GetValues(typeof(ObjectSize)))
				.OrderByDescending(s => s)
				.Take(config.NumSizeTiers)
				.ToList();

			// 3-5. Objects per color, sizes, stack assignment and expected order (largest at the bottom).
			for (int stackIndex = 0; stackIndex < task.Colors.Count; stackIndex++)
			{
				ObjectColor color = task.Colors[stackIndex];
				int count = rng.Next(config.MinObjectsPerColor, config.MaxObjectsPerColor + 1);
				var sizes = Shuffle(new List<ObjectSize>(tiers), rng).Take(count).OrderByDescending(s => s).ToList();

				var target = new StackTarget { Color = color, BasePosition = workspace.StackBase[color] };
				for (int order = 0; order < sizes.Count; order++)
				{
					var obj = new SortObjectDefinition
					{
						Color       = color,
						Size        = sizes[order],
						Edge        = workspace.EdgeLength[sizes[order]],
						TargetStack = stackIndex,
						TargetOrder = order,
					};
					target.ExpectedObjects.Add(obj);
					task.Objects.Add(obj);
				}
				task.Targets.Add(target);
			}

			// 6-7. Positions and rotations.
			PlaceObjects(task, config, workspace, rng);

			// 8. Cluttered starts. Strictly after every existing draw, and only when on, so clutter-free seeds are unchanged.
			if (config.PileCount > 0) BuildPiles(task, config, workspace, rng);

			task.SurfaceY = workspace.SurfaceY;
			task.Workspace = workspace;
			return task;
		}

		private static void PlaceObjects(TaskDefinition task, TaskConfig config, WorkspaceSpec ws, Random rng)
		{
			// Place big objects first; they're the hardest to fit.
			var order = task.Objects.OrderByDescending(o => o.Edge).ThenBy(o => o.Color).ToList();

			if (config.RandomizePositions)
			{
				for (int restart = 0; restart < LayoutRestarts; restart++)
				{
					if (TryRandomLayout(order, config, ws, rng))
						return;
				}
			}

			GridLayout(order, ws);
			task.UsedFallbackLayout = config.RandomizePositions;
		}

		// Rejection sampling inside the spawn region and reach annulus. Footprints use the
		// bounding circle (half-diagonal) so any yaw is collision-free.
		private static bool TryRandomLayout(List<SortObjectDefinition> objects, TaskConfig config, WorkspaceSpec ws, Random rng)
		{
			var placed = new List<SortObjectDefinition>();
			foreach (var obj in objects)
			{
				float radius = FootprintRadius(obj.Edge);
				bool ok = false;
				for (int attempt = 0; attempt < AttemptsPerObject && !ok; attempt++)
				{
					float x = Lerp(ws.SpawnMinX + radius, ws.SpawnMaxX - radius, (float)rng.NextDouble());
					float z = Lerp(ws.SpawnMinZ + radius, ws.SpawnMaxZ - radius, (float)rng.NextDouble());
					if (!InReach(x, z, ws)) continue;
					if (placed.Any(p => DistanceXZ(p.InitialPosition, x, z) < FootprintRadius(p.Edge) + radius + ws.GripperClearance)) continue;

					obj.InitialPosition = new Vector3(x, RestY(obj, ws), z);
					obj.InitialYaw = config.RandomizeRotations ? (float)rng.NextDouble() * MathF.PI / 2f : 0f;   // cube: 90° symmetric
					placed.Add(obj);
					ok = true;
				}
				if (!ok) return false;
			}
			return true;
		}

		// Pile some cubes on others: pick a base, then 1..MaxPileHeight−1 other cubes (any colour, random order) on top.
		// Without AllowInvertedPiles a cube only goes on one at least as large, so piles are physically stable and their
		// footprint is the base's (which already satisfies the spacing rule). With it, a larger cube must still clear
		// every other cube's footprint. Climbers share the base's yaw so faces rest flat.
		private static void BuildPiles(TaskDefinition task, TaskConfig config, WorkspaceSpec ws, Random rng)
		{
			var used = new HashSet<SortObjectDefinition>();
			int piles = 0;
			foreach (var baseObj in Shuffle(new List<SortObjectDefinition>(task.Objects), rng))
			{
				if (piles >= config.PileCount) break;
				if (used.Contains(baseObj)) continue;

				int height = rng.Next(2, config.MaxPileHeight + 1);
				var pile = new List<SortObjectDefinition> { baseObj };
				var candidates = Shuffle(task.Objects.Where(o => o != baseObj && !used.Contains(o)).ToList(), rng);
				foreach (var c in candidates)
				{
					if (pile.Count >= height) break;
					SortObjectDefinition top = pile[pile.Count - 1];
					if (c.Edge > top.Edge)
					{
						if (!config.AllowInvertedPiles) continue;
						bool clear = task.Objects.Where(o => o != c && !pile.Contains(o)).All(o =>
							DistanceXZ(o.InitialPosition, baseObj.InitialPosition.X, baseObj.InitialPosition.Z) >=
							FootprintRadius(c.Edge) + FootprintRadius(o.Edge) + ws.GripperClearance);
						if (!clear) continue;
					}
					c.InitialPosition = new Vector3(baseObj.InitialPosition.X,
						top.InitialPosition.Y + (top.Edge + c.Edge) / 2f + ws.SpawnDropHeight,
						baseObj.InitialPosition.Z);
					c.InitialYaw = baseObj.InitialYaw;
					c.InitialSupport = top.PoolTag;
					pile.Add(c);
				}
				if (pile.Count < 2) continue;   // nothing fitted on this base
				foreach (var p in pile) used.Add(p);
				piles++;
			}
		}

		// Deterministic grid across the spawn region, yaw 0 (fixed-position difficulty / fallback).
		private static void GridLayout(List<SortObjectDefinition> objects, WorkspaceSpec ws)
		{
			float cell = objects.Max(o => o.Edge) + ws.GripperClearance + 0.01f;
			int cols = Math.Max(1, (int)((ws.SpawnMaxX - ws.SpawnMinX) / cell));
			for (int i = 0; i < objects.Count; i++)
			{
				var obj = objects[i];
				float x = ws.SpawnMinX + cell * (0.5f + i % cols);
				float z = ws.SpawnMaxZ - cell * (0.5f + i / cols);
				obj.InitialPosition = new Vector3(x, RestY(obj, ws), z);
				obj.InitialYaw = 0f;
			}
		}

		private static float RestY(SortObjectDefinition obj, WorkspaceSpec ws) => ws.SurfaceY + obj.Edge / 2f + ws.SpawnDropHeight;
		private static float FootprintRadius(float edge) => edge * 0.5f * MathF.Sqrt(2f);
		private static float Lerp(float a, float b, float t) => a + (b - a) * t;
		private static float DistanceXZ(Vector3 p, float x, float z) => MathF.Sqrt((p.X - x) * (p.X - x) + (p.Z - z) * (p.Z - z));

		private static bool InReach(float x, float z, WorkspaceSpec ws)
		{
			float r = MathF.Sqrt(x * x + z * z);
			return r >= ws.MinReach && r <= ws.MaxReach;
		}

		private static List<T> Shuffle<T>(List<T> list, Random rng)
		{
			for (int i = list.Count - 1; i > 0; i--)
			{
				int j = rng.Next(i + 1);
				(list[i], list[j]) = (list[j], list[i]);
			}
			return list;
		}
	}
}
