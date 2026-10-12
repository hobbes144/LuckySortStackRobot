using System.Collections.Generic;
using System.Numerics;
using System.Text;
using LuckySortStack.Core;

namespace LuckySortStack.Planning
{
	// One pick/place: which object, and which slot of which stack it goes to.
	public enum StepKind
	{
		ToStack,    // place the object on its stack (the plan)
		ToBuffer,   // take it off a pile and set it on a buffer spot (cluttered starts); StackIndex = −1
	}

	public sealed class PlannedStep
	{
		public SortObjectDefinition Object = null!;
		public StackTarget Stack = null!;     // the object's own stack (for a buffer move: just for display)
		public int StackIndex;                // −1 for a buffer move
		public int Order;   // 0 = bottom
		public StepKind Kind = StepKind.ToStack;
		public Vector2 BufferSpot;            // world X, Z of the spot, for ToBuffer

		public bool IsBufferMove => Kind == StepKind.ToBuffer;

		public override string ToString() => IsBufferMove
			? $"Move {Object.Size} {Object.Color} -> buffer"
			: $"Pick {Object.Size} {Object.Color} -> {Stack.Color.ToString().ToUpperInvariant()}[{Order}]";
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

		// Every step label any task can produce: colour × size × stack level, where a cube can only sit at a
		// level with that many larger sizes below it. Fixed order, so a label's index is stable across runs
		// (the Observer's task indices are registered from this list).
		public static List<string> AllStepLabels()
		{
			var labels = new List<string>();
			foreach (ObjectColor color in System.Enum.GetValues(typeof(ObjectColor)))
				foreach (ObjectSize size in System.Enum.GetValues(typeof(ObjectSize)))
					for (int order = 0; order <= (int)ObjectSize.Large - (int)size; order++)
						labels.Add(new PlannedStep
						{
							Object = new SortObjectDefinition { Color = color, Size = size },
							Stack  = new StackTarget { Color = color },
							Order  = order,
						}.ToString());
			return labels;
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
