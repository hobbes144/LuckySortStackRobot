namespace LuckySortStack.Core
{
	// One stack per color. Values match the scene pool (LS_Obj_<Color>_<Size>).
	public enum ObjectColor
	{
		Red,
		Blue,
		Green,
		Yellow,
	}

	// Ordinal order IS the physical order: a larger value must sit lower in a stack.
	public enum ObjectSize
	{
		Tiny,
		Small,
		Medium,
		Large,
	}

	// Scene naming conventions shared by the SceneBuilder tool and runtime scripts.
	public static class SceneTags
	{
		public const string Workspace = "LS_Workspace";

		public static string PoolObject(ObjectColor color, ObjectSize size) => $"LS_Obj_{color}_{size}";
		public static string StackTarget(ObjectColor color) => $"LS_StackTarget_{color}";
	}
}
