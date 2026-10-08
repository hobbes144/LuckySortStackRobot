using System;
using System.Collections.Generic;
using Hazel;
using LuckySortStack.Core;
using LuckySortStack.Generation;
using Num = System.Numerics;

namespace LuckySortStack.Episodes
{
	// Owns one episode: reads the workspace from the scene, generates a task from
	// the seed, logs it, and lays out the object pool to match.
	// Stage 2/3: generation + layout only. Planning/execution/evaluation come later.
	public class EpisodeManager : Entity
	{
		[Group("Task")] [Tooltip("Same seed + config = same task.")]
		public int Seed = 1234;

		[Group("Task")] [Tooltip("Ignore Seed and draw a fresh one each Play (it is logged so it can be replayed).")]
		public bool RandomizeSeedOnPlay = false;

		[Group("Task")] [ClampValue(1, 4)]
		public int NumColors = 3;

		[Group("Task")] [ClampValue(1, 4)]
		public int MinObjectsPerColor = 2;

		[Group("Task")] [ClampValue(1, 4)]
		public int MaxObjectsPerColor = 3;

		[Group("Task")] [ClampValue(1, 4)]
		[Tooltip("Number of size tiers in play, from Large downwards.")]
		public int NumSizeTiers = 3;

		[Group("Task")]
		public bool RandomizePositions = true;

		[Group("Task")]
		public bool RandomizeRotations = true;

		[Group("Scene")]
		[Tooltip("Move pool objects to the generated layout at Play. Off = log the task only.")]
		public bool ApplyLayoutToScene = true;

		[Group("Scene")] [Units("m")]
		[Tooltip("World X of the row where unused pool objects are parked (behind the robot).")]
		public float ParkingX = -0.35f;

		[Group("Scene")] [Units("s")] [Slider(0.1f, 3.0f)]
		[Tooltip("Sim time to wait before checking that objects ended up where the layout put them.")]
		public float PlacementCheckDelay = 0.5f;

		[Group("Scene")] [Units("m")]
		public float PlacementTolerance = 0.005f;

		public TaskDefinition? CurrentTask { get; private set; }

		[HideFromEditor] private readonly Dictionary<string, Entity> m_Pool = new Dictionary<string, Entity>();
		[HideFromEditor] private readonly Dictionary<Entity, Vector3> m_Intended = new Dictionary<Entity, Vector3>();
		[HideFromEditor] private float m_CheckTimer = -1f;

		protected override void OnCreate()
		{
			WorkspaceSpec? workspace = ReadWorkspace();
			if (workspace == null) return;

			TaskConfig config = new TaskConfig
			{
				Seed               = RandomizeSeedOnPlay ? Environment.TickCount & 0x7FFFFFFF : Seed,
				NumColors          = NumColors,
				MinObjectsPerColor = MinObjectsPerColor,
				MaxObjectsPerColor = MaxObjectsPerColor,
				NumSizeTiers       = NumSizeTiers,
				RandomizePositions = RandomizePositions,
				RandomizeRotations = RandomizeRotations,
			};

			try
			{
				CurrentTask = TaskGenerator.Generate(config, workspace);
			}
			catch (ArgumentException e)
			{
				Log.Error($"[Episode] Invalid task config: {e.Message}");
				return;
			}

			LogTask(CurrentTask);

			if (ApplyLayoutToScene)
			{
				ApplyLayout(CurrentTask);
				m_CheckTimer = PlacementCheckDelay;
			}
		}

		protected override void OnUpdate(float ts)
		{
			if (m_CheckTimer < 0f) return;
			m_CheckTimer -= ts;
			if (m_CheckTimer < 0f) CheckPlacement();
		}

		// Build the generator's view of the workspace from actual scene geometry.
		private WorkspaceSpec? ReadWorkspace()
		{
			WorkspaceSpec ws = WorkspaceSpec.Default;
			ws.EdgeLength.Clear();
			ws.StackBase.Clear();

			Entity? surface = FindEntityByTag(SceneTags.Workspace);
			if (surface == null)
			{
				Log.Error($"[Episode] '{SceneTags.Workspace}' not found. Run Tools/SceneBuilder.");
				return null;
			}
			ws.SurfaceY = surface.Transform.WorldTranslation.Y + surface.Transform.Scale.Y * 0.5f;

			foreach (ObjectColor color in Enum.GetValues(typeof(ObjectColor)))
			{
				Entity? marker = FindEntityByTag(SceneTags.StackTarget(color));
				if (marker == null)
				{
					Log.Error($"[Episode] Stack target '{SceneTags.StackTarget(color)}' not found.");
					return null;
				}
				Vector3 top = marker.Transform.WorldTranslation + new Vector3(0f, marker.Transform.Scale.Y * 0.5f, 0f);
				ws.StackBase[color] = ToNum(top);

				foreach (ObjectSize size in Enum.GetValues(typeof(ObjectSize)))
				{
					string tag = SceneTags.PoolObject(color, size);
					Entity? obj = FindEntityByTag(tag);
					if (obj == null)
					{
						Log.Error($"[Episode] Pool object '{tag}' not found.");
						return null;
					}
					m_Pool[tag] = obj;
					ws.EdgeLength[size] = obj.Transform.Scale.Y;   // physical size is the source of truth
				}
			}
			return ws;
		}

		private void ApplyLayout(TaskDefinition task)
		{
			var used = new HashSet<string>();
			foreach (SortObjectDefinition def in task.Objects)
			{
				Entity obj = m_Pool[def.PoolTag];
				Place(obj, ToHazel(def.InitialPosition), def.InitialYaw);
				used.Add(def.PoolTag);
			}

			// Park everything else in a row behind the robot, resting on the ground (y = 0).
			int slot = 0;
			foreach (var kv in m_Pool)
			{
				if (used.Contains(kv.Key)) continue;
				float edge = kv.Value.Transform.Scale.Y;
				Place(kv.Value, new Vector3(ParkingX, edge * 0.5f + 0.002f, -0.50f + slot * 0.07f), 0f);
				slot++;
			}
		}

		private void Place(Entity obj, Vector3 position, float yaw)
		{
			obj.Transform.WorldTranslation = position;
			obj.Transform.WorldRotation = new Vector3(0f, yaw, 0f);
			m_Intended[obj] = position;
		}

		// Placement by transform write before physics starts is an unverified engine
		// behaviour for MuJoCo bodies (see LUCKY_API_NOTES §8). This makes it obvious.
		private void CheckPlacement()
		{
			int ok = 0;
			foreach (var kv in m_Intended)
			{
				Vector3 actual = kv.Key.Transform.WorldTranslation;
				float dx = actual.X - kv.Value.X, dz = actual.Z - kv.Value.Z;
				float horizontal = Mathf.Sqrt(dx * dx + dz * dz);
				if (horizontal <= PlacementTolerance) { ok++; continue; }
				Log.Warn($"[Episode] {kv.Key.Tag}: intended ({kv.Value.X:F3}, {kv.Value.Z:F3}) actual ({actual.X:F3}, {actual.Z:F3}) off by {horizontal * 1000f:F1} mm");
			}

			if (ok == m_Intended.Count)
				Log.Info($"[Episode] Placement check passed: {ok}/{m_Intended.Count} objects within {PlacementTolerance * 1000f:F0} mm.");
			else
				Log.Warn($"[Episode] Placement check FAILED: {ok}/{m_Intended.Count} objects within tolerance.");
		}

		private static void LogTask(TaskDefinition task)
		{
			Log.Info($"[Episode] Generated task {task.Fingerprint()}{(task.UsedFallbackLayout ? " (grid fallback layout)" : "")}");
			foreach (string line in task.Describe().Split('\n'))
				if (line.Length > 0) Log.Info("[Episode] " + line);
		}

		private static Num.Vector3 ToNum(Vector3 v) => new Num.Vector3(v.X, v.Y, v.Z);
		private static Vector3 ToHazel(Num.Vector3 v) => new Vector3(v.X, v.Y, v.Z);
	}
}
