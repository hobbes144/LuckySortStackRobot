using System.Collections.Generic;
using System.Numerics;

namespace LuckySortStack.Core
{
	// Physical description of the workspace the generator plans into.
	// At runtime this is read from the scene (so sizes are the real geometry);
	// in tests it comes from Default.
	public sealed class WorkspaceSpec
	{
		// Height of the surface objects rest on (world Y).
		public float SurfaceY = 0.01f;

		// Spawn region on the surface (world X/Z, metres).
		public float SpawnMinX = 0.28f;
		public float SpawnMaxX = 0.70f;
		public float SpawnMinZ = -0.42f;
		public float SpawnMaxZ = -0.05f;

		// Horizontal distance from the robot base that IK can reach reliably.
		public float MinReach = 0.28f;
		public float MaxReach = 0.72f;

		// Extra gap between object footprints so open fingers fit between neighbours.
		public float GripperClearance = 0.02f;

		// Drop height above the surface when spawning, so bodies settle instead of interpenetrating.
		public float SpawnDropHeight = 0.002f;

		// Extent of the mat (world X, Z). Cubes set aside while unstacking go anywhere free on it (Planning/FreeSpace).
		// Read from the LS_Workspace entity at runtime.
		public float MatMinX = 0.25f;
		public float MatMaxX = 0.75f;
		public float MatMinZ = -0.45f;
		public float MatMaxZ = 0.45f;

		// Physical cube edge per size (metres).
		public Dictionary<ObjectSize, float> EdgeLength = new Dictionary<ObjectSize, float>();

		// Base of each stack: centre of the marker's top surface (world).
		public Dictionary<ObjectColor, Vector3> StackBase = new Dictionary<ObjectColor, Vector3>();

		// Matches Tools/SceneBuilder Layout.Default.
		public static WorkspaceSpec Default => new WorkspaceSpec
		{
			EdgeLength = new Dictionary<ObjectSize, float>
			{
				{ ObjectSize.Large,  0.060f },
				{ ObjectSize.Medium, 0.050f },
				{ ObjectSize.Small,  0.040f },
				{ ObjectSize.Tiny,   0.030f },
			},
			StackBase = new Dictionary<ObjectColor, Vector3>
			{
				{ ObjectColor.Red,    new Vector3(0.40f, 0.011f, 0.18f) },
				{ ObjectColor.Blue,   new Vector3(0.55f, 0.011f, 0.18f) },
				{ ObjectColor.Green,  new Vector3(0.40f, 0.011f, 0.33f) },
				{ ObjectColor.Yellow, new Vector3(0.55f, 0.011f, 0.33f) },
			},
		};
	}
}
