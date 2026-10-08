using System;

namespace LuckySortStack.Core
{
	// Parameters that define the family of tasks a seed is drawn from.
	public sealed class TaskConfig
	{
		public int Seed = 1234;
		public int NumColors = 3;
		public int MinObjectsPerColor = 2;
		public int MaxObjectsPerColor = 3;

		// Uses the N largest size tiers (Large, Medium, ...); more tiers = smaller, harder objects.
		public int NumSizeTiers = 3;

		public bool RandomizePositions = true;
		public bool RandomizeRotations = true;

		public static int MaxColors => Enum.GetValues(typeof(ObjectColor)).Length;
		public static int MaxSizeTiers => Enum.GetValues(typeof(ObjectSize)).Length;

		// Throws on configurations that can't produce a valid task.
		public void Validate()
		{
			if (NumColors < 1 || NumColors > MaxColors)
				throw new ArgumentOutOfRangeException(nameof(NumColors), $"must be 1..{MaxColors}");
			if (NumSizeTiers < 1 || NumSizeTiers > MaxSizeTiers)
				throw new ArgumentOutOfRangeException(nameof(NumSizeTiers), $"must be 1..{MaxSizeTiers}");
			if (MinObjectsPerColor < 1 || MinObjectsPerColor > MaxObjectsPerColor)
				throw new ArgumentOutOfRangeException(nameof(MinObjectsPerColor), "must be 1..MaxObjectsPerColor");
			// The pool holds one object per color/size, so a color can't have more objects than tiers.
			if (MaxObjectsPerColor > NumSizeTiers)
				throw new ArgumentOutOfRangeException(nameof(MaxObjectsPerColor), "can't exceed NumSizeTiers");
		}

		public override string ToString() =>
			$"colors={NumColors} objects/color={MinObjectsPerColor}-{MaxObjectsPerColor} tiers={NumSizeTiers} " +
			$"randPos={RandomizePositions} randRot={RandomizeRotations}";
	}
}
