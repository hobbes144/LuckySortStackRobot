using System.Collections.Generic;
using System.Numerics;
using System.Text;
using LuckySortStack.Core;

namespace LuckySortStack.Planning
{
	// One pick/place: which object, and which slot of which stack it goes to.
	public sealed class PlannedStep
	{
		public SortObjectDefinition Object = null!;
		public StackTarget Stack = null!;
		public int StackIndex;
		public int Order;   // 0 = bottom

		public override string ToString() =>
			$"Pick {Object.Size} {Object.Color} -> {Stack.Color.ToString().ToUpperInvariant()}[{Order}]";
	}

	// Turns task metadata into an ordered action sequence. It answers "what next
	// and where"; it knows nothing about motion. The evaluator, not the planner,
	// decides whether the physical result is right.
	public static class StackPlanner
	{
		public static List<PlannedStep> Plan(TaskDefinition task)
		{
			var steps = new List<PlannedStep>();
			for (int s = 0; s < task.Targets.Count; s++)
			{
				StackTarget stack = task.Targets[s];
				// ExpectedObjects is already bottom → top (largest first), so build in that order.
				for (int i = 0; i < stack.ExpectedObjects.Count; i++)
					steps.Add(new PlannedStep { Object = stack.ExpectedObjects[i], Stack = stack, StackIndex = s, Order = i });
			}
			return steps;
		}

		public static string Describe(IReadOnlyList<PlannedStep> steps)
		{
			var sb = new StringBuilder();
			for (int i = 0; i < steps.Count; i++)
				sb.Append(i + 1).Append(". ").Append(steps[i]).Append('\n');
			return sb.ToString();
		}
	}

	// Where the next object goes, derived from the stack as it physically is now
	// rather than from hard-coded heights, so it adapts to any object sizes and
	// follows small drift of the objects already placed.
	public static class StackPlacement
	{
		// placed: live centre position + edge of each verified object on this stack, bottom → top.
		public static Vector3 PlaceCentre(StackTarget stack, IReadOnlyList<(Vector3 Position, float Edge)> placed, float edge, float gap)
		{
			if (placed.Count == 0)
				return new Vector3(stack.BasePosition.X, stack.BasePosition.Y + edge / 2f + gap, stack.BasePosition.Z);

			var top = placed[placed.Count - 1];
			return new Vector3(top.Position.X, top.Position.Y + top.Edge / 2f + edge / 2f + gap, top.Position.Z);
		}
	}
}
