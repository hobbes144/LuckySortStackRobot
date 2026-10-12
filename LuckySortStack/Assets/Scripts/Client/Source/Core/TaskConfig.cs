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

		// Cluttered starts (difficulty level 5, docs/CLUTTER_DESIGN.md): some cubes begin piled on others, mixed colours,
		// wrong order. 0 = off. With 0, the generator makes no extra random draws, so every seed gives exactly the task it
		// always did (GoldenSeedIsUnchanged).
		public int PileCount = 0;
		public int MaxPileHeight = 3;               // cubes per pile, including the bottom one
		public bool AllowInvertedPiles = false;     // allow a larger cube on a smaller one (topple risk at reset)

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
			if (PileCount < 0 || PileCount > 8)
				throw new ArgumentOutOfRangeException(nameof(PileCount), "must be 0..8");
			if (MaxPileHeight < 2 || MaxPileHeight > 4)
				throw new ArgumentOutOfRangeException(nameof(MaxPileHeight), "must be 2..4");
		}

		public override string ToString() =>
			$"colors={NumColors} objects/color={MinObjectsPerColor}-{MaxObjectsPerColor} tiers={NumSizeTiers} " +
			$"randPos={RandomizePositions} randRot={RandomizeRotations}" +
			// Only when on, so the description (and the fingerprint built from it) of every clutter-free task is unchanged.
			(PileCount > 0 ? $" piles={PileCount} maxHeight={MaxPileHeight}{(AllowInvertedPiles ? " inverted" : "")}" : "");
	}
}
