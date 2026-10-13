// SceneBuilder: writes the LuckySort Stage 1 workspace into a Lucky .hscene.
//
// Idempotent: every generated entity/asset gets a deterministic ID derived from
// its name, so re-running replaces previous output instead of duplicating it.
// The Lucky editor must be CLOSED while this runs (its autosave would clobber
// the file).
//
// Usage: dotnet run --project Tools/SceneBuilder -- <path-to-LuckySortStack>

using System.Globalization;
using System.Text;

CultureInfo.DefaultThreadCurrentCulture = CultureInfo.InvariantCulture;
Thread.CurrentThread.CurrentCulture = CultureInfo.InvariantCulture;

string projectDir = args.Length > 0 ? args[0] : "LuckySortStack";
string assetsDir  = Path.Combine(projectDir, "Assets");
string scenePath  = Path.Combine(assetsDir, "Scenes", "Main.hscene");
string registryPath = Path.Combine(assetsDir, "AssetRegistry.hzr");

if (!File.Exists(scenePath) || !File.Exists(registryPath))
{
	Console.Error.WriteLine($"Scene or asset registry not found under '{projectDir}'.");
	return 1;
}

var layout = Layout.Default;
var materials = new List<MaterialDef>();
var entities  = new List<EntityDef>();

// ── Materials ───────────────────────────────────────────────────────────────
var workspaceMat = new MaterialDef("M_Workspace", new Vec3(0.18f, 0.19f, 0.21f), Roughness: 0.8f);
materials.Add(workspaceMat);

var objectMats = new Dictionary<string, MaterialDef>();
var markerMats = new Dictionary<string, MaterialDef>();
foreach (var color in layout.Colors)
{
	objectMats[color.Name] = new MaterialDef($"M_{color.Name}", color.Rgb, Roughness: 0.45f);
	markerMats[color.Name] = new MaterialDef($"M_{color.Name}_Marker", color.Rgb * 0.45f + new Vec3(0.55f, 0.55f, 0.55f) * 0.25f, Roughness: 0.9f);
	materials.Add(objectMats[color.Name]);
	materials.Add(markerMats[color.Name]);
}

// ── Workspace surface (static MuJoCo fixture) ───────────────────────────────
entities.Add(new EntityDef(
	Tag: "LS_Workspace",
	Position: new Vec3(layout.WorkspaceCenterX, layout.WorkspaceThickness / 2f, layout.WorkspaceCenterZ),
	Scale: new Vec3(layout.WorkspaceSizeX, layout.WorkspaceThickness, layout.WorkspaceSizeZ),
	Material: workspaceMat,
	Collider: ColliderKind.Static));

// ── Stack target markers (visual only, no collider) ─────────────────────────
for (int i = 0; i < layout.Colors.Length; i++)
{
	var color = layout.Colors[i];
	var target = layout.StackTargets[i];
	entities.Add(new EntityDef(
		Tag: $"LS_StackTarget_{color.Name}",
		Position: new Vec3(target.X, layout.WorkspaceThickness + layout.MarkerThickness / 2f, target.Z),
		Scale: new Vec3(layout.MarkerSize, layout.MarkerThickness, layout.MarkerSize),
		Material: markerMats[color.Name],
		Collider: ColliderKind.None));
}

// ── Object pool: one dynamic cube per color × size ──────────────────────────
// Rows = color (along Z), columns = size (along X). The task generator will
// re-place these per episode; this is just a valid, settled starting layout.
for (int c = 0; c < layout.Colors.Length; c++)
{
	for (int s = 0; s < layout.Sizes.Length; s++)
	{
		var color = layout.Colors[c];
		var size  = layout.Sizes[s];
		float x = layout.PoolStartX + s * layout.PoolSpacing;
		float z = layout.PoolStartZ - c * layout.PoolSpacing;
		float y = layout.WorkspaceThickness + size.Edge / 2f + layout.SpawnClearance;
		entities.Add(new EntityDef(
			Tag: $"LS_Obj_{color.Name}_{size.Name}",
			Position: new Vec3(x, y, z),
			Scale: new Vec3(size.Edge, size.Edge, size.Edge),
			Material: objectMats[color.Name],
			Collider: ColliderKind.Dynamic));
	}
}

// ── Vision camera (Milestone 9) ─────────────────────────────────────────────
// Straight down over the mat, 2 m up, not primary. Keep in sync with Perception/VisionRig (the tests' copy); at runtime
// the camera model is read from this entity, so moving it in the editor is fine.
var visionCamera = new CameraDef(
	Tag: "LS_Camera_Vision",
	Position: new Vec3(layout.WorkspaceCenterX, 2.0f, layout.WorkspaceCenterZ),
	RotationRadians: new Vec3(-MathF.PI / 2f, 0f, 0f),
	VerticalFovDegrees: 27.5f);

// ── Write material files ────────────────────────────────────────────────────
string matDir = Path.Combine(assetsDir, "Materials", "LuckySort");
Directory.CreateDirectory(matDir);
foreach (var m in materials)
	File.WriteAllText(Path.Combine(matDir, m.Name + ".hmaterial"), m.ToYaml());

// ── Asset registry ──────────────────────────────────────────────────────────
var registry = RegistryFile.Load(registryPath);
foreach (var m in materials)
	registry.Upsert(m.Handle, $"Materials/LuckySort/{m.Name}.hmaterial", "Material");
registry.Save(registryPath);

// ── Scene ───────────────────────────────────────────────────────────────────
var scene = SceneFile.Load(scenePath);
int removed = scene.RemoveEntities(id => entities.Any(e => e.Id == id) || id == visionCamera.Id || scene.TagOf(id)?.StartsWith("LS_") == true);
foreach (var e in entities)
	scene.AddEntity(e.Id, e.ToYaml());
scene.AddEntity(visionCamera.Id, visionCamera.ToYaml());
scene.Save(scenePath);

Console.WriteLine($"Materials: {materials.Count}, entities written: {entities.Count + 1} incl. {visionCamera.Tag} (replaced {removed}).");
return 0;

// ════════════════════════════════════════════════════════════════════════════

record struct Vec3(float X, float Y, float Z)
{
	public static Vec3 operator *(Vec3 v, float s) => new(v.X * s, v.Y * s, v.Z * s);
	public static Vec3 operator +(Vec3 a, Vec3 b) => new(a.X + b.X, a.Y + b.Y, a.Z + b.Z);
	public override string ToString() => $"[{F(X)}, {F(Y)}, {F(Z)}]";
	static string F(float f) => f.ToString("0.######", CultureInfo.InvariantCulture);
}

record ColorDef(string Name, Vec3 Rgb);
record SizeDef(string Name, float Edge);
record struct XZ(float X, float Z);

// All workspace geometry in one place (metres, Hazel Y-up, robot base at origin, +X forward).
record Layout(
	ColorDef[] Colors, SizeDef[] Sizes, XZ[] StackTargets,
	float WorkspaceCenterX, float WorkspaceCenterZ, float WorkspaceSizeX, float WorkspaceSizeZ, float WorkspaceThickness,
	float MarkerSize, float MarkerThickness,
	float PoolStartX, float PoolStartZ, float PoolSpacing, float SpawnClearance)
{
	public static Layout Default => new(
		Colors: new[]
		{
			new ColorDef("Red",    new Vec3(0.85f, 0.12f, 0.10f)),
			new ColorDef("Blue",   new Vec3(0.10f, 0.30f, 0.85f)),
			new ColorDef("Green",  new Vec3(0.12f, 0.65f, 0.20f)),
			new ColorDef("Yellow", new Vec3(0.95f, 0.80f, 0.10f)),
		},
		// Largest gripped dimension must stay well under the Panda's ~80 mm jaw opening.
		Sizes: new[]
		{
			new SizeDef("Large",  0.060f),
			new SizeDef("Medium", 0.050f),
			new SizeDef("Small",  0.040f),
			new SizeDef("Tiny",   0.030f),
		},
		StackTargets: new[] { new XZ(0.40f, 0.18f), new XZ(0.55f, 0.18f), new XZ(0.40f, 0.33f), new XZ(0.55f, 0.33f) },
		WorkspaceCenterX: 0.50f, WorkspaceCenterZ: 0.0f, WorkspaceSizeX: 0.50f, WorkspaceSizeZ: 0.90f, WorkspaceThickness: 0.01f,
		MarkerSize: 0.075f, MarkerThickness: 0.001f,
		PoolStartX: 0.33f, PoolStartZ: -0.08f, PoolSpacing: 0.09f, SpawnClearance: 0.002f);
}

static class Ids
{
	// Deterministic 64-bit FNV-1a so re-runs produce identical IDs.
	public static ulong From(string name)
	{
		ulong h = 14695981039346656037UL;
		foreach (byte b in Encoding.UTF8.GetBytes("LuckySort:" + name)) { h ^= b; h *= 1099511628211UL; }
		return h == 0 ? 1 : h;
	}
}

record MaterialDef(string Name, Vec3 Albedo, float Roughness)
{
	public ulong Handle => Ids.From("material/" + Name);
	public string ToYaml() =>
		"Material:\n" +
		"  Transparent: false\n" +
		$"  AlbedoColor: {Albedo}\n" +
		"  Emission: 0\n" +
		"  UVScale: [1, 1]\n" +
		"  UVOffset: [0, 0]\n" +
		"  UseNormalMap: false\n" +
		"  Metalness: 0\n" +
		$"  Roughness: {Roughness.ToString(CultureInfo.InvariantCulture)}\n" +
		"  AlbedoMap: 0\n" +
		"  NormalMap: 0\n" +
		"  MetalnessMap: 0\n" +
		"  RoughnessMap: 0\n" +
		"  MaterialFlags: 6";
}

enum ColliderKind { None, Static, Dynamic }

record EntityDef(string Tag, Vec3 Position, Vec3 Scale, MaterialDef Material, ColliderKind Collider)
{
	const ulong CubeMeshHandle = 4241663774595030205UL;   // Meshes/Default/Cube.hsmesh (unit cube)

	public ulong Id => Ids.From("entity/" + Tag);

	public string ToYaml()
	{
		var sb = new StringBuilder();
		sb.Append($"  - Entity: {Id}\n");
		sb.Append("    Domain: Client\n");
		sb.Append("    TagComponent:\n");
		sb.Append($"      Tag: {Tag}\n");
		sb.Append("    Parent: 0\n");
		sb.Append("    Children:\n");
		sb.Append("      []\n");
		sb.Append("    TransformComponent:\n");
		sb.Append($"      Position: {Position}\n");
		sb.Append("      Rotation: [0, 0, 0]\n");
		sb.Append($"      Scale: {Scale}\n");
		sb.Append("    StaticMeshComponent:\n");
		sb.Append($"      AssetID: {CubeMeshHandle}\n");
		sb.Append("      MaterialTable:\n");
		sb.Append($"        0: {Material.Handle}\n");
		sb.Append("      Visible: true\n");
		if (Collider != ColliderKind.None)
		{
			// Collider size comes from the entity scale (unit cube).
			sb.Append("    MujocoBoxColliderComponent:\n");
			sb.Append("      Density: 500\n");
			sb.Append("      Friction: [1, 0.1, 0.01]\n");
			sb.Append("      Contype: 1\n");
			sb.Append("      Conaffinity: 1\n");
			sb.Append("      Condim: -1\n");
		}
		if (Collider == ColliderKind.Dynamic)
		{
			// Free joint (type 0) gives the body 6 DOF, same as make_mujoco_dynamic_body.
			sb.Append("    MujocoJointComponent:\n");
			sb.Append("      Type: \"\\x00\"\n");
			sb.Append("      Position: [0, 0, 0]\n");
			sb.Append("      Axis: [0, 1, 0]\n");
			sb.Append("      Limited: false\n");
			sb.Append("      LimitMin: -3.14159\n");
			sb.Append("      LimitMax: 3.14159\n");
			sb.Append("      Stiffness: 0\n");
			sb.Append("      Damping: 0\n");
			sb.Append("      Armature: 0\n");
			sb.Append("      JointId: -1\n");
		}
		return sb.ToString().TrimEnd('\n');
	}
}

// A camera entity (CameraComponent fields as the editor writes them). Primary stays false: the scene camera keeps the view.
record CameraDef(string Tag, Vec3 Position, Vec3 RotationRadians, float VerticalFovDegrees)
{
	public ulong Id => Ids.From("entity/" + Tag);

	public string ToYaml()
	{
		var sb = new StringBuilder();
		sb.Append($"  - Entity: {Id}\n");
		sb.Append("    Domain: Client\n");
		sb.Append("    TagComponent:\n");
		sb.Append($"      Tag: {Tag}\n");
		sb.Append("    Parent: 0\n");
		sb.Append("    Children:\n");
		sb.Append("      []\n");
		sb.Append("    TransformComponent:\n");
		sb.Append($"      Position: {Position}\n");
		sb.Append($"      Rotation: {RotationRadians}\n");
		sb.Append("      Scale: [1, 1, 1]\n");
		sb.Append("    CameraComponent:\n");
		sb.Append("      Camera:\n");
		sb.Append("        ProjectionType: 0\n");
		sb.Append($"        PerspectiveFOV: {VerticalFovDegrees.ToString("0.###", CultureInfo.InvariantCulture)}\n");
		sb.Append("        PerspectiveNear: 0.1\n");
		sb.Append("        PerspectiveFar: 1000\n");
		sb.Append("        OrthographicSize: 10\n");
		sb.Append("        OrthographicNear: -1\n");
		sb.Append("        OrthographicFar: 1\n");
		sb.Append("      Primary: false");
		return sb.ToString();
	}
}

// Line-based editor for the `Entities:` section of a .hscene. Entity blocks start
// with "  - Entity: <id>" and are kept sorted by id, as the editor writes them.
class SceneFile
{
	readonly List<string> m_Head = new();
	readonly List<string> m_Tail = new();
	readonly SortedDictionary<ulong, string> m_Blocks = new();
	string m_Newline = "\n";

	public static SceneFile Load(string path)
	{
		var file = new SceneFile();
		string text = File.ReadAllText(path);
		if (text.Contains("\r\n")) file.m_Newline = "\r\n";
		var lines = text.Replace("\r\n", "\n").Split('\n');

		int i = 0;
		while (i < lines.Length && lines[i] != "Entities:") file.m_Head.Add(lines[i++]);
		if (i == lines.Length) throw new InvalidDataException("No 'Entities:' section in scene.");
		file.m_Head.Add(lines[i++]);

		ulong currentId = 0;
		var current = new List<string>();
		void Flush() { if (current.Count > 0) file.m_Blocks[currentId] = string.Join("\n", current); current.Clear(); }

		for (; i < lines.Length; i++)
		{
			string line = lines[i];
			if (line.Length > 0 && !char.IsWhiteSpace(line[0])) break;   // next top-level key
			if (line.StartsWith("  - Entity: "))
			{
				Flush();
				currentId = ulong.Parse(line.Substring("  - Entity: ".Length).Trim());
			}
			current.Add(line);
		}
		Flush();
		for (; i < lines.Length; i++) file.m_Tail.Add(lines[i]);
		return file;
	}

	public string? TagOf(ulong id)
	{
		if (!m_Blocks.TryGetValue(id, out var block)) return null;
		foreach (var line in block.Split('\n'))
			if (line.StartsWith("      Tag: ")) return line.Substring("      Tag: ".Length).Trim();
		return null;
	}

	public int RemoveEntities(Func<ulong, bool> predicate)
	{
		var doomed = m_Blocks.Keys.Where(predicate).ToList();
		foreach (var id in doomed) m_Blocks.Remove(id);
		return doomed.Count;
	}

	public void AddEntity(ulong id, string yaml) => m_Blocks[id] = yaml;

	public void Save(string path)
	{
		var all = new List<string>(m_Head);
		all.AddRange(m_Blocks.Values);
		all.AddRange(m_Tail);
		File.WriteAllText(path, string.Join(m_Newline, all.SelectMany(s => s.Split('\n'))));
	}
}

// Line-based editor for AssetRegistry.hzr ("  - Handle / FilePath / Type" triples, sorted by handle).
class RegistryFile
{
	readonly SortedDictionary<ulong, (string Path, string Type)> m_Entries = new();
	string m_Newline = "\n";

	public static RegistryFile Load(string path)
	{
		var reg = new RegistryFile();
		string text = File.ReadAllText(path);
		if (text.Contains("\r\n")) reg.m_Newline = "\r\n";
		var lines = text.Replace("\r\n", "\n").Split('\n');
		for (int i = 0; i < lines.Length; i++)
		{
			if (!lines[i].StartsWith("  - Handle: ")) continue;
			ulong handle = ulong.Parse(lines[i].Substring("  - Handle: ".Length).Trim());
			string filePath = lines[i + 1].Trim().Substring("FilePath: ".Length);
			string type = lines[i + 2].Trim().Substring("Type: ".Length);
			reg.m_Entries[handle] = (filePath, type);
			i += 2;
		}
		return reg;
	}

	public void Upsert(ulong handle, string filePath, string type)
	{
		foreach (var stale in m_Entries.Where(kv => kv.Value.Path == filePath && kv.Key != handle).Select(kv => kv.Key).ToList())
			m_Entries.Remove(stale);
		m_Entries[handle] = (filePath, type);
	}

	public void Save(string path)
	{
		var sb = new StringBuilder("Assets:");
		foreach (var (handle, (filePath, type)) in m_Entries)
			sb.Append(m_Newline).Append($"  - Handle: {handle}")
			  .Append(m_Newline).Append($"    FilePath: {filePath}")
			  .Append(m_Newline).Append($"    Type: {type}");
		File.WriteAllText(path, sb.ToString());
	}
}
