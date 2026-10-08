using System;
using System.Collections.Generic;
using System.Linq;
using LuckySortStack.Core;
using LuckySortStack.Generation;
using Xunit;

namespace LuckySort.Core.Tests
{
	public class TaskGeneratorTests
	{
		private static TaskDefinition Gen(int seed, Action<TaskConfig>? tweak = null)
		{
			var config = new TaskConfig { Seed = seed };
			tweak?.Invoke(config);
			return TaskGenerator.Generate(config, WorkspaceSpec.Default);
		}

		public static IEnumerable<object[]> Seeds => Enumerable.Range(0, 200).Select(s => new object[] { s * 7919 + 1 });

		[Fact]
		public void SameSeedProducesIdenticalTask()
		{
			var a = Gen(1234);
			var b = Gen(1234);
			Assert.Equal(a.Describe(), b.Describe());
			Assert.Equal(a.Fingerprint(), b.Fingerprint());
		}

		[Fact]
		public void DifferentSeedsProduceDifferentTasks()
		{
			var fingerprints = Enumerable.Range(0, 50).Select(s => Gen(s).Fingerprint()).ToHashSet();
			Assert.Equal(50, fingerprints.Count);
		}

		[Fact]
		public void GoldenSeedIsUnchanged()
		{
			// Guards against accidental changes to RNG consumption order or layout rules, which would
			// silently change the task behind every recorded seed. Update deliberately if generation changes.
			Assert.Equal("ACF5C17F6BE82A85", Gen(1234).Fingerprint());
		}

		[Theory]
		[MemberData(nameof(Seeds))]
		public void StacksAreLargestToSmallest(int seed)
		{
			var task = Gen(seed, c => { c.NumColors = 4; c.MaxObjectsPerColor = 4; c.NumSizeTiers = 4; });
			foreach (var target in task.Targets)
			{
				var sizes = target.ExpectedObjects.Select(o => (int)o.Size).ToList();
				Assert.Equal(sizes.OrderByDescending(s => s), sizes);
				Assert.Equal(sizes.Distinct().Count(), sizes.Count);
				for (int i = 0; i < target.ExpectedObjects.Count; i++)
				{
					Assert.Equal(i, target.ExpectedObjects[i].TargetOrder);
					Assert.Equal(target.Color, target.ExpectedObjects[i].Color);
				}
			}
		}

		[Theory]
		[MemberData(nameof(Seeds))]
		public void CountsRespectConfig(int seed)
		{
			var task = Gen(seed, c => { c.NumColors = 3; c.MinObjectsPerColor = 2; c.MaxObjectsPerColor = 3; c.NumSizeTiers = 3; });
			Assert.Equal(3, task.Colors.Count);
			Assert.Equal(3, task.Targets.Count);
			Assert.All(task.Targets, t => Assert.InRange(t.ExpectedObjects.Count, 2, 3));
			Assert.All(task.Objects, o => Assert.NotEqual(ObjectSize.Tiny, o.Size));   // 3 tiers = Large..Small
			Assert.Equal(task.Objects.Count, task.Objects.Select(o => o.PoolTag).Distinct().Count());
		}

		[Theory]
		[MemberData(nameof(Seeds))]
		public void ObjectsDoNotOverlapAndStayInSpawnRegion(int seed)
		{
			var ws = WorkspaceSpec.Default;
			var task = Gen(seed, c => { c.NumColors = 4; c.MaxObjectsPerColor = 4; c.NumSizeTiers = 4; c.MinObjectsPerColor = 3; });
			foreach (var o in task.Objects)
			{
				float half = o.Edge / 2f;
				Assert.InRange(o.InitialPosition.X, ws.SpawnMinX + half, ws.SpawnMaxX - half);
				Assert.InRange(o.InitialPosition.Z, ws.SpawnMinZ + half, ws.SpawnMaxZ - half);
				Assert.True(o.InitialPosition.Y > ws.SurfaceY + half, "object must start above the surface");
			}

			for (int i = 0; i < task.Objects.Count; i++)
			for (int j = i + 1; j < task.Objects.Count; j++)
			{
				var a = task.Objects[i];
				var b = task.Objects[j];
				float dx = a.InitialPosition.X - b.InitialPosition.X, dz = a.InitialPosition.Z - b.InitialPosition.Z;
				float dist = MathF.Sqrt(dx * dx + dz * dz);
				// Bounding squares (half-diagonal) can never intersect at this distance, whatever the yaw.
				Assert.True(dist >= (a.Edge + b.Edge) * 0.5f * MathF.Sqrt(2f), $"{a.PoolTag} and {b.PoolTag} overlap ({dist:F3} m)");
			}
		}

		[Fact]
		public void FixedPositionsUseGridWithZeroYaw()
		{
			var task = Gen(42, c => { c.RandomizePositions = false; });
			Assert.False(task.UsedFallbackLayout);
			Assert.All(task.Objects, o => Assert.Equal(0f, o.InitialYaw));
		}

		[Fact]
		public void RotationsCanBeDisabled()
		{
			var task = Gen(42, c => { c.RandomizeRotations = false; });
			Assert.All(task.Objects, o => Assert.Equal(0f, o.InitialYaw));
		}

		[Theory]
		[InlineData(0, 2, 3, 3)]   // no colors
		[InlineData(5, 2, 3, 3)]   // more colors than the pool has
		[InlineData(3, 3, 2, 3)]   // min > max
		[InlineData(3, 2, 4, 3)]   // more objects per color than tiers
		[InlineData(3, 1, 1, 0)]   // no tiers
		public void InvalidConfigsAreRejected(int colors, int min, int max, int tiers)
		{
			Assert.Throws<ArgumentOutOfRangeException>(() => Gen(1, c =>
			{
				c.NumColors = colors; c.MinObjectsPerColor = min; c.MaxObjectsPerColor = max; c.NumSizeTiers = tiers;
			}));
		}

		[Fact]
		public void DescribePrintsStacksInPlanFormat()
		{
			string text = Gen(1234).Describe(includeLayout: false);
			Assert.StartsWith("Seed: 1234", text);
			var task = Gen(1234);
			foreach (var t in task.Targets)
				Assert.Contains(t.Color.ToString().ToUpperInvariant() + ":", text);
		}
	}
}
