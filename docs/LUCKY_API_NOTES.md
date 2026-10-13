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
- AgentDocs `patterns/robot-control.md`: "There is no `SetJointPosition` — joint writes go through actuators only";
  `WeldBody` + `GetDefaultFreeJointPosition/Quaternion` are the documented free-body "teleport"/reset path.
- **Layout path (2026-10-09):** every layout, including the first, now goes through `SceneResetter`'s qpos
  write, calibrated in the first frame of Play on the saved scene positions. The transform write is only a
  logged fallback when calibration fails. My first guess that transform writes fling overlapping cubes was
  **wrong**; the real cause was a re-parented cube (see the open bug below).
- **In-Play reset for benchmark mode (`Episodes/SceneResetter`, 2026-10-09, VERIFY with `Debug → ResetSpike`):**
  `ResetToInitialState(preserveTime: true)`, then each cube's free-joint qpos `[x y z qw qx qy qz]` is written with
  `SetQposByIndex`. Nothing about body naming or the Hazel↔MuJoCo frame is documented, so `Calibrate()` measures them:
  free bodies (`GetBodyFreeJointIndex` + `GetJointType == 0`) are matched to pool entities by Hazel-space position
  (`GetPosition(id, true)`). The frame `mj = R·hz + t` uses `R = GetAxisHzToMj()` or its inverse (whichever fits),
  with `t` fitted across all cubes, and the fit spread and the qpos-vs-body agreement are logged (>2 mm disables
  the reset). Assumed: body id == body index (MuJoCo convention), and `ModelNjnt` bounds a valid joint index.
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
- **Rechecked for Milestone 6 (2026-10-09):** the DLL exposes nothing else (no metadata/info API, no output-path
  property), and the local AgentDocs don't mention the Observer. The only engine usage is RobotSandbox:
  `TaskManager.Start()` calls `StartRecording()`, and `Welcome.cs` calls `EndCurrentEpisodeBeginNext(true)` between
  runs, with **no `BeginEpisode()`**. That suggests `StartRecording()` opens the first episode. The Python SDK README
  says the engine writes per-substep `qpos`/`ctrl` rows to `data/chunk-XXX/file-YYY.parquet` with a `frame_flags`
  column described in the recording's `info.json` (a LeRobot-style layout). No `.parquet` existed anywhere on this
  machine before Milestone 6, so recording had never been triggered.
  **VERIFY on the first recorded run:** (1) where the dataset lands, (2) whether one episode is recorded without
  `BeginEpisode()`, (3) whether the per-pick `RegisterTask` labels appear in it.
- Because the Observer has no metadata API, task metadata goes in our own sidecar JSON (`Recording/EpisodeRecorder`).
- **Found on the first recorded run (2026-10-09):**
  - **Observer recording requires a Lucky Hub sign-in.** When signed out, `StartRecording()` is refused with the
    log line `HUB: Recording start refused: user is not authenticated.` Check `Hazel.Auth.IsAuthenticated()` first
    (`Auth` also has `Login()`, `Logout()`, `GetUsername()`). `ObserverSession` checks it and skips with a warning.
  - **Video capture needs FFmpeg:** `FFmpeg is required for video recording and was not found. You can install
    FFmpeg via Tools->FFmpeg Settings, or to your system PATH`.
  - **Dataset location and format:** `<project>/DataSessions/session_<yyyy-MM-dd_HH-mm-ss>/`, LeRobot v3 layout:
    `data/chunk-000/file-000.parquet` (per-frame `index, robot_id, episode_index, frame_index, timestamp, task_index,
    frame_flags, action[8], observation.state[8]`, the 7 Panda joints + `finger_joint1`, actions named
    `actuator1..8`), `meta/info.json` (features, fps), `meta/stats.json`, `meta/tasks.parquet`,
    `meta/episodes/chunk-000/file-000.parquet` (per-episode `length`, **`success`**, frame ranges, video timestamps)
    and `videos/observation.images.<Camera>/chunk-000/file-000.mp4`. `info.json` says `robot_type: "ur5e"` for the
    Panda; that looks like an engine default, not something we set.
  - **Recording is configured per scene** in `ObserverConfig` (in `Main.hscene`; edit it in the editor, since
    autosave overwrites hand edits): `MaxEpisodeDurationSeconds: 90` **cuts episodes at 90 s** (our run got 4,500
    frames at 50 Hz and was split into 2 episodes), `DataHz/VideoFramerate: 50`, `TotalEpisodes: 300`,
    `EnableDiscardFailedEpisodes: false`, `OutputDirectory: ""` (= DataSessions), and
    `Cameras: [{CameraName: Camera_Record, VideoWidth: 256, VideoHeight: 256}]`, which is why the video is small.
    For full episodes, set `MaxEpisodeDurationSeconds` ≥ 600 (the state machine's episode timeout).
  - **Task labels must be registered before `StartRecording()`.** Labels registered during the episode never
    reached `tasks.parquet` (only `"Default Task"`; `RegisteredTasks: []` in the scene). `ObserverSession` now
    registers all 40 possible pick labels (`StackPlanner.AllStepLabels()`) first. VERIFY on the next run.
  - Per-episode boundaries in a multi-episode session: `EndCurrentEpisode(success)` then `BeginEpisode()` for the
    next. VERIFY in `meta/episodes` after the first benchmark run.
  - **Scripts can't see the project folder directly.** The editor's working directory is `C:\WINDOWS\system32`
    (not writable), the script assembly's `Assembly.Location` is empty (loaded from memory), and the DLL exposes no
    project-path API (`Application.DataDirectoryPath` is the only path property). The working anchor is the
    compile-time `[CallerFilePath]` of a script, which the build embeds unmapped. Walk up from it to the `.hproj`.

### 11. Camera observations
- `CameraComponent`: projection settings, `Primary`, `ToScreenSpace`, `GetRayDirection`, `SetOrthographic(size, near, far)`.
- **C# pixel readback DOES exist (found 2026-10-12, undocumented):** `Hazel.Net.Grpc.Capture.CaptureBridge` is public:
  `public static bool TryCaptureCameraRgba(ulong entityId, uint width, uint height, out byte[] rgba, out uint outWidth, out uint outHeight)`
  and `TryCaptureViewportRgba(uint width, uint height, out byte[] rgba, out uint outWidth, out uint outHeight)`.
  **Confirmed 2026-10-12 from the 2026.1.2 DLL metadata** (parameter names and `[Out]` flags read with
  System.Reflection.Metadata): the ulong is the **entity ID** (`Entity.ID` is a public `UInt64` field). The project's
  `Vision/VisionSensor.cs` compiles against it. This is what the gRPC `CameraService` uses. **RGBA only: no depth API**
  (searched every type and native binding). Also present, unused: `GrpcStepCapture_QueueCamera / Submit / WaitAndGetCount /
  GetFrameDataPtr` natives (a batched per-step capture path behind the gRPC server). VERIFY in editor with the V0 spike
  (`docs/VISION_PLAN.md` §7): cost per call, pixel row order, whether a non-primary camera renders, whether the capture
  honours the requested size/aspect.
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
- **RESOLVED 2026-10-12: `LS_Obj_Red_Small` was re-parented under `LS_Obj_Yellow_Large` (2026-10-09).** Scene re-rooted;
  `ReadWorkspace` now rejects parented pool cubes. Write-up of this and the two bugs behind it: `ENGINEERING_NOTES.md` §6.1.
  - **Symptom.** With `ResetSpike` on and seed 748253065 (4 colours × 4 sizes), the first placement check fails
    every time: `LS_Obj_Red_Small` intended (0.546, 0.032, −0.266), actual **(0.918, 0.021, −0.049)**, 430.5 mm
    off, yaw 28.9° off. Same pose to the millimetre on every run, and deleting `Cache/Mujoco/Models/*.mjb` made no
    difference. `SceneResetter.Calibrate` found **15 MuJoCo free bodies for 16 cubes**. Bodies are named
    `lucky_body_<entity ID>`, and mapping them back showed Red_Small (entity `6530565878512278646`) is the one
    without a free joint.
  - **Root cause.** In the working-copy `Main.hscene` (not in the 17:37 commit `64ed38b`), Red_Small has
    `Parent: 5093231954624172071` = **`LS_Obj_Yellow_Large`**, which lists it under `Children`. Its transform became
    parent-relative (`Position: [3, -0.166667, 4.5]`, `Scale: [0.666667, …]`). The engine compiles a child entity
    into its parent's MuJoCo body, so Red_Small has no free joint and moves rigidly with Yellow_Large, wherever
    that cube is placed. Almost certainly an accidental drag-and-drop in the editor's hierarchy panel.
  - **Lesson.** I first blamed overlapping transform writes, then a stale cache. The body count and the
    `lucky_body_<entity ID>` names were what ruled both out. A side-by-side diff of the scene entries missed the
    `Parent:` line because I had masked long numbers to hide entity IDs.
  - **Fix (pick one, with the editor closed):**
    - `git diff LuckySortStack/Assets/Scenes/Main.hscene` should show only this re-parent plus the new
      `BenchmarkEpisodes`/`ResetSpike` field values; if so, `git checkout -- LuckySortStack/Assets/Scenes/Main.hscene`
      restores it. Those two fields then take their code defaults (1 / false).
    - or, in the editor, drag `LS_Obj_Red_Small` back to the scene root and confirm its transform reads
      Position (0.51, 0.032, −0.08), Scale 0.04;
    - or regenerate the workspace with `Tools/SceneBuilder` (it replaces all `LS_*` entities).
  - **Hardening to add:** at startup, check that every pool cube is a root entity (`Parent: 0`) and a MuJoCo
    free body (free-body count == pool size), and name the offending cube before laying anything out.

- **RESOLVED 2026-10-12, reset rotations + arm slump (verified: ResetSpike seeds 5555–5557 all OK).**
  **Measured convention:** `[Reset] Orientation frame: C=[w 1.000, x 0.000, y 0.000, z 0.000] (q_mj = R ⊗ q_hz), worst
  disagreement across cubes 0.00°`. So a body's MuJoCo quaternion is the Hazel world rotation pre-multiplied by `R`
  (`GetAxisHzToMj`, +90° about X); the body's local axes are not converted. A Hazel yaw θ is written as `R ⊗ Ry(θ)`,
  not `Rz(θ)`. after the re-parent fix, seed 5555 placed
  every cube at the right spot but only 7/16 passed the yaw check: exactly the 7 parked yaw-0 cubes. `HzToMj` is +90° about X,
  and the old write assumed a cube's MuJoCo quaternion is just "yaw about MuJoCo up", i.e. `q_mj = R·q_hz·R⁻¹`. The engine evidently
  uses a different convention. `Calibrate` now measures `C` in `q_mj = R ⊗ q_hz ⊗ C` from the saved scene poses (logged as
  `[Reset] Orientation frame: …`) and writes `R ⊗ yaw ⊗ C`. The old write-then-read axis self-test was removed: it read
  `GetOrientation` in the same frame as the qpos write, before MuJoCo recomputed body poses, so it always saw 0°.
  Arm slump: `ResetToInitialState` ran on frame 1 of every Play and cleared the arm's actuator targets. Now it runs only between
  episodes, followed by `RehomeArm()` (start-of-Play grasp-site pose, gripper open).

## Scene file format (for `Tools/SceneBuilder`)

Taken from the engine's `RobotSandbox/.../Welcome.hscene` and this project's `Main.hscene`.
- Files are CRLF, no trailing newline. Entities live under `Entities:` as `  - Entity: <uint64>` blocks, sorted by ID.
- Static MuJoCo fixture = `StaticMeshComponent` + `MujocoBoxColliderComponent` (size from entity scale on the unit cube).
- Dynamic MuJoCo body = the above + `MujocoJointComponent` with `Type: "\x00"` (free joint). This matches what the editor
  agent's `make_mujoco_dynamic_body` produces. An older prefab (`P_BlueBlock`) used `Dynamic: true` on the collider instead.
  If generated cubes don't fall/settle, try that form.
- Materials: `.hmaterial` YAML + an entry in `AssetRegistry.hzr` (`Handle` / `FilePath` / `Type: Material`).
- Unit cube mesh handle: `4241663774595030205` (`Meshes/Default/Cube.hsmesh`).

## Overlay / text (2026-10-12, Milestone 8)
- **No screen-space UI in the C# API** (no ImGui, no canvas; `Hazel-ScriptCore.dll` exports only `TextComponent` and
  `DebugRenderer` for drawing). The overlay is therefore world text parented to the camera entity.
- Confirmed: **static** `Scene.CreateEntity(name)` / `Scene.DestroyEntity(e)` (Transform + Tag only). The AgentDocs' `Scene.Instance`
  **does not exist** in 2026.1.2 (CS0117); checked against the DLL's metadata tables. `entity.CreateComponent<T>()`,
  `TextComponent.Text` (string) / `.Color` (Vector4), `Entity.Parent` get **and set**, `Transform.Translation/Scale` setters,
  `OnLateUpdate(float)`, `DebugRenderer.DrawLine(p0, p1, Vector4)`, `DrawCircle(centre, rotation, radius, color)`,
  `DrawQuadBillboard(pos, Vector2 size, color)`, `DebugRenderer.LineWidth` (set only). Sources: AgentDocs `csharp-api/Scene.md`,
  `Entity.md`; docs.luckyrobots.com API pages; DLL export names.
- **Undocumented, VERIFY in editor:** text size per unit of entity scale, the text's anchor point, which local axis the text
  faces, whether a default font is used, glyph coverage (overlay sticks to ASCII), whether DebugRenderer primitives last one
  frame (assumed: drawn every frame from `OnLateUpdate`), and `DrawCircle`'s rotation convention (unused; squares are drawn with lines).
- **Measured in editor (2026-10-12 screenshot):** a camera-child TextComponent renders and faces the camera; its anchor is
  the text's left edge, near the first line's centre; one line is ~0.96 x the entity scale; **long lines wrap at ~28
  characters** (no C# control over the width), so the overlay keeps every line to 24 (`OverlayFormatter.MaxLineChars`).
- `CameraComponent.VerticalFOV` is in **degrees** (AgentDocs). There's no viewport-size API; the overlay measures the aspect by
  projecting three points through `ToScreenSpace` (units undocumented; the `[Overlay] ToScreenSpace:` log line records them).
- Scene camera: tag `Camera`, `Primary: true`.
- **Update clocks (docs: scripting/getting-started/entity-lifecycle, 2026-10-12):** `OnUpdate`/`OnLateUpdate` run per fixed
  simulation step (Robot runner, 50 Hz); a rendered frame can contain 0, 1 or several steps. `OnFreePreUpdate`/`OnFreePostUpdate`
  run **once per rendered frame** (same signature, `float ts` = frame time) but only with the scene setting
  "Enable Free Step Update Scripts" (`FreeStepEnabled` in the .hscene; off in this project until turned on).
  **DebugRenderer is immediate-mode: drawn from a per-step callback it blinks** on frames with no step. Markers, camera
  blending and `Input.IsKeyPressed` (true for one rendered frame) therefore run from `OnFreePostUpdate`.

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
