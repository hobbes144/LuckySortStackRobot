using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using LuckySortStack.Core;
using LuckySortStack.Robot;

namespace LuckySortStack.Evaluation
{
	// Tolerances in metres / seconds. Deliberately separate from MotionSettings: the evaluator
	// doesn't share the state machine's idea of "good enough".
	public sealed class EvaluationSettings
	{
		public float MembershipRadius = 0.06f;      // an object this close (XZ) to a stack base counts as in that stack
		public float PositionTolerance = 0.015f;    // allowed XZ offset of any stacked object from the stack base
		public float HeightTolerance = 0.008f;      // allowed gap/penetration against the support below
		public float TiltTolerance = 10f * MathF.PI / 180f;   // a cube leaning on an edge further than this isn't resting flat
		public float StableWindow = 1.0f;           // sim time the scene is watched before judging
		public float StableDrift = 0.002f;          // max movement of any object during the window
	}

	// Why an evaluation failed, most fundamental first. A report can have several; the first
	// one present is the primary failure used to categorise the episode.
	public enum EvalFailure
	{
		None,
		WrongColor,        // an object is on another colour's stack
		WrongOrder,        // a larger object sits on a smaller one
		Unstable,          // something in a stack was still moving
		Unsupported,       // floating, overhanging, sunk into the support, or tilted
		OutOfTolerance,    // resting properly but too far from the stack centre
		Incomplete,        // expected objects missing from their stack
	}

	public sealed class StackReport
	{
		public ObjectColor Color;
		public List<SortObjectDefinition> Expected = new List<SortObjectDefinition>();   // bottom → top
		public List<SortObjectDefinition> Actual = new List<SortObjectDefinition>();     // bottom → top, as found

		public bool ColorCorrect;
		public bool ObjectCountCorrect;
		public bool SizeOrderCorrect;
		public bool PositionWithinTolerance;
		public bool SupportedByPreviousObject;
		public bool StackStable;
		public bool AllObjectsPlaced;
		public List<string> Issues = new List<string>();
		public HashSet<SortObjectDefinition> Faulty = new HashSet<SortObjectDefinition>();   // failed a position, support or stability check

		public bool Success => ColorCorrect && ObjectCountCorrect && SizeOrderCorrect && PositionWithinTolerance
			&& SupportedByPreviousObject && StackStable && AllObjectsPlaced;
	}

	public sealed class EvaluationReport
	{
		public List<StackReport> Stacks = new List<StackReport>();
		public List<SortObjectDefinition> Unplaced = new List<SortObjectDefinition>();   // task objects in no stack

		public bool ColorCorrect              => Stacks.All(s => s.ColorCorrect);
		public bool ObjectCountCorrect        => Stacks.All(s => s.ObjectCountCorrect);
		public bool SizeOrderCorrect          => Stacks.All(s => s.SizeOrderCorrect);
		public bool PositionWithinTolerance   => Stacks.All(s => s.PositionWithinTolerance);
		public bool SupportedByPreviousObject => Stacks.All(s => s.SupportedByPreviousObject);
		public bool StackStable               => Stacks.All(s => s.StackStable);
		public bool AllObjectsPlaced          => Stacks.All(s => s.AllObjectsPlaced) && Unplaced.Count == 0;

		public bool Success => ColorCorrect && ObjectCountCorrect && SizeOrderCorrect && PositionWithinTolerance
			&& SupportedByPreviousObject && StackStable && AllObjectsPlaced;

		public int CorrectlyStacked => Stacks.Sum(s => CountCorrectFromBottom(s));
		public int ExpectedCount => Stacks.Sum(s => s.Expected.Count);

		// Every failure category present, in EvalFailure order.
		public List<EvalFailure> Failures
		{
			get
			{
				var list = new List<EvalFailure>();
				if (!ColorCorrect)                                list.Add(EvalFailure.WrongColor);
				if (!SizeOrderCorrect)                            list.Add(EvalFailure.WrongOrder);
				if (!StackStable)                                 list.Add(EvalFailure.Unstable);
				if (!SupportedByPreviousObject)                   list.Add(EvalFailure.Unsupported);
				if (!PositionWithinTolerance)                     list.Add(EvalFailure.OutOfTolerance);
				if (!AllObjectsPlaced || !ObjectCountCorrect)     list.Add(EvalFailure.Incomplete);
				return list;
			}
		}

		public EvalFailure PrimaryFailure
		{
			get
			{
				List<EvalFailure> failures = Failures;
				return failures.Count == 0 ? EvalFailure.None : failures[0];
			}
		}

		public string Describe()
		{
			var sb = new StringBuilder();
			sb.Append(Success ? "PASS" : "FAIL").Append($": {CorrectlyStacked}/{ExpectedCount} objects correctly stacked");
			if (!Success) sb.Append(" (").Append(string.Join(", ", Failures)).Append(')');
			sb.Append('\n');
			foreach (StackReport s in Stacks)
			{
				sb.Append($"  {s.Color.ToString().ToUpperInvariant()} {(s.Success ? "ok" : "FAIL")}: ")
				  .Append(s.Actual.Count == 0 ? "(empty)" : string.Join(" < ", s.Actual.AsEnumerable().Reverse().Select(o => $"{o.Color} {o.Size}")))
				  .Append('\n');
				foreach (string issue in s.Issues) sb.Append("    - ").Append(issue).Append('\n');
			}
			foreach (SortObjectDefinition o in Unplaced)
				sb.Append($"  not in any stack: {o.PoolTag}\n");
			return sb.ToString();
		}

		// Correct prefix from the bottom (right object, in place, supported, still): what a partial run genuinely got right.
		private static int CountCorrectFromBottom(StackReport s)
		{
			int n = 0;
			while (n < s.Actual.Count && n < s.Expected.Count && s.Actual[n] == s.Expected[n] && !s.Faulty.Contains(s.Actual[n])) n++;
			return n;
		}
	}

	// Judges the episode from physical state only. The task definition says what the stacks
	// should be; IWorldState says where every object really is. Nothing from the planner or
	// the state machine is consulted, so a state machine that believes it succeeded can still fail here.
	//
	// Stability needs time, so the evaluator watches the scene for StableWindow of sim time
	// (driven by Update(dt), no wall clock) and then produces its report.
	public sealed class StackEvaluator
	{
		public bool IsComplete => Report != null;
		public EvaluationReport? Report { get; private set; }

		private readonly TaskDefinition m_Task;
		private readonly IWorldState m_World;
		private readonly EvaluationSettings m_Cfg;
		private readonly Dictionary<string, Vector3> m_Start = new Dictionary<string, Vector3>();
		private readonly Dictionary<string, float> m_MaxDrift = new Dictionary<string, float>();
		private float m_Elapsed = -1f;

		public StackEvaluator(TaskDefinition task, IWorldState world, EvaluationSettings settings)
		{
			m_Task  = task;
			m_World = world;
			m_Cfg   = settings;
		}

		public void Update(float dt)
		{
			if (IsComplete) return;

			if (m_Elapsed < 0f)
			{
				foreach (SortObjectDefinition o in m_Task.Objects)
				{
					m_Start[o.PoolTag] = m_World.GetObjectPosition(o.PoolTag);
					m_MaxDrift[o.PoolTag] = 0f;
				}
				m_Elapsed = 0f;
				return;
			}

			m_Elapsed += dt;
			foreach (SortObjectDefinition o in m_Task.Objects)
			{
				float drift = Vector3.Distance(m_World.GetObjectPosition(o.PoolTag), m_Start[o.PoolTag]);
				m_MaxDrift[o.PoolTag] = Math.Max(m_MaxDrift[o.PoolTag], drift);
			}

			if (m_Elapsed >= m_Cfg.StableWindow)
				Report = Inspect(m_Task, m_World, m_Cfg, m_MaxDrift);
		}

		// One-shot inspection. maxDrift (tag → movement during an observation window) feeds the
		// stability check; without it every stack is assumed stable.
		public static EvaluationReport Inspect(TaskDefinition task, IWorldState world, EvaluationSettings cfg,
			IReadOnlyDictionary<string, float>? maxDrift = null)
		{
			var report = new EvaluationReport();

			// Assign every task object to the nearest stack base within MembershipRadius.
			var columns = task.Targets.Select(_ => new List<(SortObjectDefinition Obj, Vector3 Pos)>()).ToList();
			foreach (SortObjectDefinition o in task.Objects)
			{
				Vector3 p = world.GetObjectPosition(o.PoolTag);
				int best = -1;
				float bestDist = cfg.MembershipRadius;
				for (int i = 0; i < task.Targets.Count; i++)
				{
					float d = HorizontalDistance(p, task.Targets[i].BasePosition);
					if (d <= bestDist) { best = i; bestDist = d; }
				}
				if (best >= 0) columns[best].Add((o, p));
				else report.Unplaced.Add(o);
			}

			for (int i = 0; i < task.Targets.Count; i++)
			{
				StackTarget target = task.Targets[i];
				var column = columns[i].OrderBy(c => c.Pos.Y).ToList();
				report.Stacks.Add(InspectStack(target, column, world, cfg, maxDrift));
			}
			return report;
		}

		private static StackReport InspectStack(StackTarget target, List<(SortObjectDefinition Obj, Vector3 Pos)> column, IWorldState world,
			EvaluationSettings cfg, IReadOnlyDictionary<string, float>? maxDrift)
		{
			var s = new StackReport
			{
				Color    = target.Color,
				Expected = target.ExpectedObjects.ToList(),
				Actual   = column.Select(c => c.Obj).ToList(),
			};

			// ColorCorrect: nothing of another colour in this stack.
			var foreign = column.Where(c => c.Obj.Color != target.Color).ToList();
			s.ColorCorrect = foreign.Count == 0;
			foreach (var f in foreign) s.Issues.Add($"wrong colour: {f.Obj.PoolTag}");

			// ObjectCountCorrect.
			s.ObjectCountCorrect = column.Count == target.ExpectedObjects.Count;
			if (!s.ObjectCountCorrect) s.Issues.Add($"{column.Count} objects, expected {target.ExpectedObjects.Count}");

			// SizeOrderCorrect: strictly smaller going up (physical rule, independent of the expected list).
			s.SizeOrderCorrect = true;
			for (int k = 1; k < column.Count; k++)
			{
				if (column[k].Obj.Size < column[k - 1].Obj.Size) continue;
				s.SizeOrderCorrect = false;
				s.Issues.Add($"size order: {column[k].Obj.Size} on top of {column[k - 1].Obj.Size}");
			}

			// PositionWithinTolerance: every object over the stack base.
			s.PositionWithinTolerance = true;
			foreach (var c in column)
			{
				float off = HorizontalDistance(c.Pos, target.BasePosition);
				if (off <= cfg.PositionTolerance) continue;
				s.PositionWithinTolerance = false;
				s.Faulty.Add(c.Obj);
				s.Issues.Add($"{c.Obj.PoolTag} {off * 1000f:F0} mm from the stack centre (limit {cfg.PositionTolerance * 1000f:F0})");
			}

			// SupportedByPreviousObject: resting on the base (bottom) or on the object below,
			// at contact height and with its centre over the support.
			s.SupportedByPreviousObject = true;
			for (int k = 0; k < column.Count; k++)
			{
				var (obj, pos) = column[k];
				float supportTop;
				string supportName;
				if (k == 0)
				{
					supportTop = target.BasePosition.Y;
					supportName = "the stack base";
				}
				else
				{
					var (below, belowPos) = column[k - 1];
					supportTop = belowPos.Y + below.Edge / 2f;
					supportName = below.PoolTag;
					float overhang = HorizontalDistance(pos, belowPos);
					if (overhang > below.Edge / 2f)
					{
						s.SupportedByPreviousObject = false;
						s.Faulty.Add(obj);
						s.Issues.Add($"{obj.PoolTag} centre overhangs {supportName} by {(overhang - below.Edge / 2f) * 1000f:F0} mm");
					}
				}

				// A cube leaning on an edge can sit near the right height, so check it's face-down too.
				float tilt = world.GetObjectTilt(obj.PoolTag);
				if (tilt > cfg.TiltTolerance)
				{
					s.SupportedByPreviousObject = false;
					s.Faulty.Add(obj);
					s.Issues.Add($"{obj.PoolTag} tilted {tilt * 180f / MathF.PI:F0}° (limit {cfg.TiltTolerance * 180f / MathF.PI:F0}°)");
				}

				float gap = (pos.Y - obj.Edge / 2f) - supportTop;
				if (Math.Abs(gap) > cfg.HeightTolerance)
				{
					s.SupportedByPreviousObject = false;
					s.Faulty.Add(obj);
					s.Issues.Add($"{obj.PoolTag} is {gap * 1000f:F0} mm {(gap > 0 ? "above" : "into")} {supportName}");
				}
			}

			// StackStable: nothing in the stack moved during the observation window.
			s.StackStable = true;
			if (maxDrift != null)
			{
				foreach (var c in column)
				{
					if (!maxDrift.TryGetValue(c.Obj.PoolTag, out float drift) || drift <= cfg.StableDrift) continue;
					s.StackStable = false;
					s.Faulty.Add(c.Obj);
					s.Issues.Add($"{c.Obj.PoolTag} moved {drift * 1000f:F1} mm while being observed");
				}
			}

			// AllObjectsPlaced: every expected object is in this stack.
			var missing = target.ExpectedObjects.Where(e => !column.Any(c => c.Obj == e)).ToList();
			s.AllObjectsPlaced = missing.Count == 0;
			foreach (var m in missing) s.Issues.Add($"missing: {m.PoolTag}");

			return s;
		}

		private static float HorizontalDistance(Vector3 a, Vector3 b) =>
			MathF.Sqrt((a.X - b.X) * (a.X - b.X) + (a.Z - b.Z) * (a.Z - b.Z));
	}
}
