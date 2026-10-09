# Lucky API Notes

Confirmed Lucky Engine APIs for LuckySort. Every entry is backed by one of:

- **DLL**: reflected from the installed `Hazel-ScriptCore.dll` (public + protected members)
- **Docs**: the engine's bundled `Data/Resources/AgentDocs/` (ships with the install)
- **Example**: engine-shipped code (`Data/RobotSandbox`, `Data/Resources/Templates/AgentScripts`)
- **Project**: files in `LuckySortStack/`
- **VERIFY**: inferred, not yet confirmed by running in the editor

| | |
|---|---|
| **Engine** | LuckyEngine 2026.1.2 (Windows x64), installed at `C:\Users\nick_\Downloads\LuckyEngine-2026.1.2-Windows-x64\LuckyEngine-2026.1-Windows-x64` |
| **Scripting** | C# / .NET 9, namespace `Hazel`, scripts inherit `Hazel.Entity` |
| **Robot** | Franka Emika Panda (MJCF `Assets/ContentVault/Robots/Panda/panda.xml`) |
| **Motion graph** | `Assets/ContentVault/Robots/Panda/MG_Panda.hmograph` (LimbIK, end effector = `grasp_site`) |
| **Physics** | MuJoCo for robot + manipulables. `FixedTimestep` 0.01 s (100 Hz), gravity `(0, -9.81, 0)` → **Y-up** |

Engine API reference dump (all public types/members): regenerate with the reflection tool if needed
(see "How this was gathered" at the bottom).

---

## Reconnaissance report (plan §33)

### 1. Robot
Franka Panda, entity tag `Panda` at world origin, rotation 0. Has `ScriptComponent` (`LuckySortStack.Panda`),
`RobotControllerComponent` (motion graph `MG_Panda`), `MujocoSceneComponent`. *(Project: `Main.hscene`)*

- 7 arm joints `joint1..joint7`, position actuators `actuator1..actuator7`.
- Gripper: `actuator8`, tendon `split` (equal force on `finger_joint1/2`), **ctrlrange [0, 255]** → finger travel 0–0.04 m each.
  **255 = open (≈80 mm jaw opening), 0 = closed.** *(Project: panda.xml)*
- `grasp_site` sits 0.105 m beyond the hand along its axis (between the fingertips).
- Rest pose: `grasp_site` at world **(0.555, 0.520, 0.0)**, rotation ≈ **(90°, 90°, 0°)** (pointing down). *(Project: Main.hscene)*

**Size constraint:** max jaw opening ≈ 80 mm, so the gripped dimension of the *largest* object must be ≲ 60 mm (leave clearance).

### 2. Motion graph inputs (MG_Panda.hmograph)
| Input | Type | Default | Notes |
|---|---|---|---|
| `Target_Position` | Vec3 | 0,0,0 | World-space metres (per Docs; VERIFY for Panda) |
| `Target_Orientation` | Vec3 | 0,0,0 | Euler **degrees** |
| `Solve_Orientation` | Bool | true | false = position-only IK |
| `Duration` | Float | 3 | Seconds; drives a Min-Jerk curve |
| `Start` | Flow (trigger) | – | Fires the move |
| `Pole_Target` | Vec3 | 0,0,0 | Elbow hint |
| `Pole_Weight` | Float | 0 | |
| `Tracking_Mode` | Bool | false | |
| `High_Accuracy` | Bool | false | |

### 3. Setting an IK target *(DLL + Docs + Example)*
```csharp
static readonly Identifier k_Pos = new Identifier("Target_Position");   // static readonly: ctor hits an InternalCall
robot.SetInputVector3(k_Pos, worldPos);
robot.SetInputVector3(k_Rot, eulerRadians * Mathf.Rad2Deg);            // degrees at the boundary only
robot.SetInputFloat(k_Dur, seconds);
robot.SetInputTrigger(k_Go);
```
- `RobotControllerComponent`: `SetInput{Bool,Int,Float,Vector3}`, `GetInput{Bool,Int,Float,Vector3}`, `SetInputTrigger`,
  `SetMotionGraphActive`, `IsMotionGraphActive`. Triggers are write-only.
- **Each trigger cancels the in-flight move.** No waypoint queue in the graph → one trigger per step, wait ≥ duration.
- **No "target reached" event.** Sequence by duration, or poll the `grasp_site` entity / `GetSitePositionMj` for convergence
  (we should do the latter for verify-states).
- Never write `Translation`/`Rotation` of robot links; MuJoCo overwrites them.

### 4. Gripper *(DLL + Docs)*
```csharp
var gripper = mujoco.GetActuator("actuator8");
gripper.IK = false;        // script ownership; otherwise the graph overwrites ctrl every substep
gripper.Value = 255;       // open; 0 = closed
```
- Ramp the open over ~5 steps on release (template trick) to avoid flicking the object.
- Grasp check candidates: finger joint positions via `GetJointID("finger_joint1")` → `GetJointQposAdr` → `GetJointPosition`
  (fully closed ≈ 0 means a miss), and `IsGeomInContact` on finger geoms.

### 5. Creating / spawning objects
- `Scene.InstantiatePrefab(...)`, `Entity.Instantiate(prefab, pos, rot, scale)`, `Scene.CreateEntity`, `Scene.DestroyEntity`. *(DLL)*
- Manipulable objects **must be MuJoCo bodies** (MuJoCo collider + `free` joint), never Jolt `RigidBody`. *(Docs: mujoco-objects)*
- **Runtime instantiation of MuJoCo bodies is NOT documented** and the MuJoCo model is built per scene.
  **Design decision:** author a fixed **object pool** in the scene (every color × size up to the max), and have the generator
  *activate and place* a seeded subset at episode start, parking unused objects off-table. VERIFY that runtime-instantiated
  MuJoCo prefabs don't work before ruling it out entirely.
- Edit-time authoring is done in the editor (the built-in agent has `make_mujoco_dynamic_body`, `duplicate_entity`,
  `batch_set_entity_property`). Boxes use the `box` collider; round graspables use `mesh`.

### 6. Reading object transform / physics state *(DLL)*
- Hazel space: `entity.Transform.WorldTranslation`, `WorldRotation` (radians), `WorldRotationQuat`.
- MuJoCo: `GetBodyID(name)` → `GetPosition(id, hazelSpace:true)`, `GetOrientation`, `GetVelocity`, `GetAngularVelocity`;
  `GetBodyPoseMj`, `GetSitePositionMj`, `GetGeomPoseMj`; `GetAxisHzToMj()` converts frames.
- Yaw from quaternion, not Euler.Y (`fwd = q * (0,0,1); yaw = atan2(fwd.X, fwd.Z)`). *(Example)*
- VERIFY how a scene entity's tag maps to a MuJoCo body name for `GetBodyID`.

### 7. Contacts *(DLL)*
- `MujocoSceneComponent.IsGeomInContact(geomId)`, `GetGeomContactForce(geomId)`, `GetGeomID(name)`.
- `Entity.CollisionBeginEvent` etc. exist but are Jolt-side; VERIFY whether they fire for MuJoCo bodies (likely not).

### 8. Reset *(DLL)*
- `MujocoSceneComponent.ResetToInitialState(bool preserveTime)` resets MuJoCo state.
- Free-body placement: `WeldBody(bodyID, posMj, quatMj, persistent)` / `UnweldBody`;
  `GetBodyFreeJointIndex` + `GetJointQposAdr` + `SetQposByIndex` / `SetFullQpos` to write a free joint's pose directly.
  (Docs say joint state is read-only, but the DLL exposes `SetQposByIndex`/`SetFullQpos`. Use these for reset-time placement
  only, never during manipulation. VERIFY.)
- Whole-scene: `Application.RequestExitPlayMode()` / `RequestEnterPlayMode()`; gRPC `ResetScene` / `ResetAgent`.
- Engine's `DomainRandomization` example places objects by writing `Transform.WorldTranslation` in `OnCreate` (before
  physics starts). VERIFY this works for MuJoCo free bodies; it would be the simplest initial-placement path.

### 9. Seeding / randomization
- `Hazel.Random` (`Float`, `Range`, `Vec3`, …) has **no seed API** → not reproducible.
- **Use `System.Random(seed)`** owned by the generator, passed explicitly. Never call `Hazel.Random` in task code.

### 10. Observer episodes *(DLL: `Hazel.Data.Observer`)*
```csharp
Observer.StartRecording();  Observer.StopRecording();  Observer.IsRecording
Observer.BeginEpisode();
Observer.EndCurrentEpisode(bool success);
Observer.EndCurrentEpisodeBeginNext(bool success);
Observer.RegisterTask(int index, string description);  Observer.SetCurrentTaskIndex(int index);
```
- Project setting `Physics.CaptureOnPlay: true` in `LuckySortStack.hproj`. VERIFY output location/format (Parquet + video per docs).
- `RegisterTask` / `SetCurrentTaskIndex` are a natural fit for per-pick sub-tasks or per-difficulty indices.

### 11. Camera observations
- `CameraComponent`: projection settings, `Primary`, `ToScreenSpace`, `GetRayDirection`. **No C# pixel-readback API found.**
- Frames are available via Observer capture and the gRPC `CameraService` (`GetCameraFrame`, `StreamCamera`, `ListCameras`).
  For now: camera = visualization + recorded observation only (matches plan §14).

### 12. Time / runners *(DLL + Project)*
- Entity hooks: `OnCreate`, `OnFreePreUpdate`, `OnPreUpdate`, `OnUpdate`, `OnPhysicsUpdate`, `OnLateUpdate`, `OnPostUpdate`,
  `OnFreePostUpdate`, `OnDestroy`.
- `Scene.ControlRunnerHz`, `Scene.TimeScale`, `Scene.SimulationMode`, `TimeSync.SyncHz/IsAtSyncPoint`,
  `Timer.Set(cb, rate, looping, firstDelay)`, `MujocoSceneComponent.MujocoTime`.
- Proposed ownership (VERIFY rates by logging `ControlRunnerHz` and `ts` in each hook):
  | Subsystem | Hook |
  |---|---|
  | State machine + robot commands | `OnUpdate` (control runner) |
  | Evaluator / settle detection | `OnPhysicsUpdate` or `OnUpdate` with sim-time windows |
  | Debug overlay / DebugRenderer | `OnLateUpdate` |
  | Observer episode boundaries | state-machine transitions |
- Measure time with accumulated `ts` / `MujocoTime`, never wall clock.

### 13. Confirmation status
- **Confirmed from installed DLL:** all signatures above.
- **Confirmed from project files:** robot, motion graph inputs, actuator names/ranges, rest pose, physics timestep.
- **Needs in-editor verification:** items marked VERIFY. The Stage 0 smoke test (`Panda.cs`) checks §2/§3/§4.

---

## Verified in editor

- **IK + trigger sequencing + gripper** (Stage 0 smoke test, 2026-10-08): world-space `Target_Position`, orientation
  (90°, 90°, 0°) top-down, one trigger per step, `actuator8` 255/0 open/close all behave as documented.
- **Generated MuJoCo cubes** (Stage 1, 2026-10-08): collider + free-joint form settles correctly and can be grasped,
  lifted and placed by the Panda (grasp test, `GraspOffsetY = 0`).
- **Initial placement of MuJoCo free bodies** (Stage 3, 2026-10-08): writing `Transform.WorldTranslation` /
  `WorldRotation` in a script's `OnCreate` (before physics runs) **works**. 16/16 pool objects were within 5 mm after
  0.5 s of sim time. This is the episode-start placement path; the `SetQpos*` fallback isn't needed.
  Still unverified: moving bodies mid-Play (needed for reset without leaving Play mode).
- **Determinism engine vs. offline** (Stage 3): seed 1234 gives fingerprint `ACF5C17F6BE82A85` both in-engine
  (workspace read from the scene) and in the unit tests (`WorkspaceSpec.Default`).
- **Entity tag ↔ entity lookup**: `FindEntityByTag` resolves the `LS_*` scene entities from scripts.

## Open issues from in-editor runs

- **Gripper yaw convention (2026-10-08):** seed 12345 drove the open fingers into a rotated Large cube and
  pushed it aside; seed 1234 stalled 51 mm above a Large Green grasp. The cause is a yaw mismatch (a 60 mm cube
  only fits the ~80 mm jaw within about ±20°). Fix: `AlignGripper` measures the real yaw from `grasp_site`'s
  quaternion (either horizontal axis, modulo 90°) and learns `physical = sign·command + offset`.
  **TODO:** record the learned sign/offset from the first `Yaw check:` log line.
- **IK tracking error vs. finger clearance (2026-10-09):** with yaw fixed, seed 12345 still pushed
  `LS_Obj_Yellow_Large` (the Large cube nearest the base, r = 0.42 m) on every attempt, with the yaw only 1.4° off.
  The arm settles a few mm from the commanded `grasp_site` target, and that's inside `MoveTolerance` (15 mm) but
  more than a Large cube's ~10 mm finger clearance. Fix: `CenterOverObject` hovers 40 mm above the cube,
  measures the horizontal error and cancels it before descending. **Resolved:** random 4-colour × 4-size seeds
  now complete in editor. Details: `ENGINEERING_NOTES.md` §3.2.

## Scene file format (for `Tools/SceneBuilder`)

Taken from the engine's `RobotSandbox/.../Welcome.hscene` and this project's `Main.hscene`.
- Files are CRLF, no trailing newline. Entities live under `Entities:` as `  - Entity: <uint64>` blocks, sorted by ID.
- Static MuJoCo fixture = `StaticMeshComponent` + `MujocoBoxColliderComponent` (size from entity scale on the unit cube).
- Dynamic MuJoCo body = the above + `MujocoJointComponent` with `Type: "\x00"` (free joint). This matches what the editor
  agent's `make_mujoco_dynamic_body` produces. An older prefab (`P_BlueBlock`) used `Dynamic: true` on the collider instead.
  If generated cubes don't fall/settle, try that form.
- Materials: `.hmaterial` YAML + an entry in `AssetRegistry.hzr` (`Handle` / `FilePath` / `Type: Material`).
- Unit cube mesh handle: `4241663774595030205` (`Meshes/Default/Cube.hsmesh`).

## Other useful APIs

| API | Purpose |
|---|---|
| `DebugRenderer.DrawLine/DrawCircle/DrawQuadBillboard` | Target markers, stack outlines, EE target (plan §20) |
| `TextComponent.Text/Color` | In-world text for the debug overlay |
| `Log.Info/Warn/Error` | Console logging |
| `Physics.CastRay/OverlapShape` | Jolt-side queries (probably not MuJoCo; VERIFY) |
| Inspector attributes `[Group] [Tooltip] [Units] [Slider] [ClampValue] [HideFromEditor] [Button] [Dropdown]` | Expose tunables |
| `Hazel.Net.Grpc.*`, `Hazel.Rpc.*` | gRPC services: Scene, Mujoco, Agent, Camera, Telemetry, Viewport, Debug (plan §18) |

## Engine reference material on disk
- `Data/Resources/AgentDocs/` — engine docs for agents (patterns: robot-control, pick-and-place, mujoco-objects, spawning, …)
- `Data/Resources/Templates/AgentScripts/PickPlaceArmDriver.cs` — calibrated pick/place queue (Piper-tuned; Panda needs recalibration)
- `Data/RobotSandbox/` — example project (TaskManager w/ Observer, DomainRandomization, Panda gRPC agent)

## How this was gathered
A throwaway .NET 9 console app loaded `Hazel-ScriptCore.dll` with an `AssemblyLoadContext` resolver pointed at the
engine's `Scripts` folder and printed every exported type's members via reflection.
