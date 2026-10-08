using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Numerics;
using System.Text;

namespace LuckySortStack.Core
{
	public sealed class SortObjectDefinition
	{
		public ObjectColor Color;
		public ObjectSize Size;
		public float Edge;                 // physical cube edge (m)
		public Vector3 InitialPosition;    // world, body centre
		public float InitialYaw;           // radians about world Y
		public int TargetStack;            // index into TaskDefinition.Targets
		public int TargetOrder;            // 0 = bottom of the stack

		public string PoolTag => SceneTags.PoolObject(Color, Size);
	}

	public sealed class StackTarget
	{
		public ObjectColor Color;
		public Vector3 BasePosition;
		public List<SortObjectDefinition> ExpectedObjects = new List<SortObjectDefinition>();   // bottom → top
	}

	// A complete, solvable task. Produced by TaskGenerator, consumed by the planner and evaluator.
	public sealed class TaskDefinition
	{
		public int Seed;
		public TaskConfig Config = new TaskConfig();
		public List<ObjectColor> Colors = new List<ObjectColor>();
		public List<SortObjectDefinition> Objects = new List<SortObjectDefinition>();
		public List<StackTarget> Targets = new List<StackTarget>();
		public bool UsedFallbackLayout;    // true if random placement failed and the grid was used

		// Stable hash of everything that defines the task; equal seeds+configs must give equal fingerprints.
		public string Fingerprint()
		{
			ulong h = 14695981039346656037UL;
			foreach (byte b in Encoding.UTF8.GetBytes(Describe(includeLayout: true)))
			{
				h ^= b;
				h *= 1099511628211UL;
			}
			return h.ToString("X16");
		}

		public string Describe(bool includeLayout = true)
		{
			var ci = CultureInfo.InvariantCulture;
			var sb = new StringBuilder();
			sb.Append("Seed: ").Append(Seed.ToString(ci)).Append('\n');
			sb.Append("Config: ").Append(Config).Append('\n');
			foreach (var target in Targets)
			{
				sb.Append('\n').Append(target.Color.ToString().ToUpperInvariant()).Append(':');
				if (includeLayout)
					sb.Append(string.Format(ci, "  stack @ ({0:F3}, {1:F3})", target.BasePosition.X, target.BasePosition.Z));
				sb.Append('\n');
				foreach (var obj in target.ExpectedObjects)
				{
					sb.Append("  ").Append(obj.Size.ToString().PadRight(7));
					if (includeLayout)
						sb.Append(string.Format(ci, "{0,3:F0} mm  start ({1:F3}, {2:F3})  yaw {3,5:F1}°",
							obj.Edge * 1000f, obj.InitialPosition.X, obj.InitialPosition.Z, obj.InitialYaw * 180f / System.MathF.PI));
					sb.Append('\n');
				}
			}
			return sb.ToString();
		}

		public IEnumerable<SortObjectDefinition> ObjectsFor(ObjectColor color) => Objects.Where(o => o.Color == color);
	}
}
