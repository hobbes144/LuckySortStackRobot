# Milestone 9: Vision (plan Stretch A + §14), build plan

**Status (2026-10-12): decisions made (§4); V1 detector done and tested offline; V0 capture spike coded and compiled
against the real engine DLL, waiting for its first editor run.** See §7 for what was built and how to run V0.

**Status at planning time: planned 2026-10-12, not started.** Goal: the robot finds the cubes and the free space on the table with its own
camera instead of reading positions from the engine. Picks, set-asides and replanning then run on what it *sees*.

## 1. Why it's feasible now

`docs/LUCKY_API_NOTES.md` §11 used to say there is no C# pixel readback. There is, undocumented:

```csharp
// Hazel.Net.Grpc.Capture.CaptureBridge (public, decoded from Hazel-ScriptCore.dll metadata)
public static bool TryCaptureCameraRgba(ulong cameraId, uint width, uint height, out byte[] rgba, out uint w, out uint h);
public static bool TryCaptureViewportRgba(uint width, uint height, out byte[] rgba, out uint w, out uint h);
```

It's what the gRPC `CameraService` uses. **RGBA only: no depth API was found.** Unverified: cost per call, pixel row order,
whether a non-primary camera renders, and whether `cameraId` is the entity ID.

## 2. Architecture

```text
Camera_Vision (top-down) ──TryCaptureCameraRgba──► RGBA frame
        │
        ▼  Perception/ (engine-free, unit-tested on synthetic images)
  colour segmentation → connected blobs → per blob: colour, size class (top-face area), centre, yaw (blob moments, mod 90°)
        │                                  + pile inference (a blob ring around a smaller blob of another colour)
        ▼
  PerceivedWorldState : IWorldState  ──►  TaskStateMachine / SupportGraph / FreeSpace (unchanged)
                                          FreeSpace gets an occupancy grid from the same image (any cube pixel = occupied)

  StackEvaluator keeps using the engine's true positions (rule 4: the evaluator inspects physical state)
        └─► perception error (seen vs true, mm and degrees) logged per object and shown on the overlay
```

- Identity is free: one cube per colour × size in the pool, so (colour, size class) names the pool tag.
- Size classes by top-face area: Large 3600 mm², Medium 2500, Small 1600, Tiny 900, well separated.
- `UseVision` toggle on `EpisodeManager`: benchmarks compare ground truth vs vision on the same seeds.

## 3. Phases (each gated before the next)

| # | Phase | Deliverable | Gate |
|---|---|---|---|
| V0 | **Capture spike** (editor) | `Camera_Vision` entity (SceneBuilder), one capture saved as PNG, timing logged | PNG shows the mat and cubes, row order known, capture cost measured |
| V1 | **Detector** (engine-free) | `Perception/CubeDetector`: segmentation, blobs, size, yaw; pixel↔world mapping | Tests on synthetic rendered images: every cube found, position ≤ 3 mm, yaw ≤ 3° (mod 90) |
| V2 | **PerceivedWorldState** | `IWorldState` over the latest detections; occluded cubes keep their last seen pose | FakeSim episodes run end to end on perceived state (synthetic camera from FakeSim) |
| V3 | **Capture policy** | Capture only when the arm is clear of the table (e.g. at SelectTarget after retreat), not per frame | No stale or occluded reads in a 10-seed editor run; frame rate unaffected |
| V4 | **Piles** | Ring inference for smaller-on-larger; chosen equal-size strategy (§4 decision 2) | Clutter seeds (`PileCount` 2) sort with vision on |
| V5 | **Overlay + benchmark** | Seen-vs-true markers, perception error on the readout; `UseVision` benchmark | 50 seeds vision on vs off, same seeds; report success, mean/max perception error |

## 4. Decisions (made 2026-10-12)

**Answers:** 1 = **perspective** top-down camera; 2 = **(a)**: in vision mode piles only put a strictly smaller cube on a larger
one (plus: of a *different colour*, since a same-colour cube on another is one blob from above); 3 = **~1 mm/px, 512 × 900**.

Consequence of perspective (and why the detector fits a model instead of taking blob centroids): off-axis, the camera sees
the cube faces that point towards the image centre as well as the top. At the mat's corners a Large cube shows ~15 mm of
side, so a centroid would be off by ~7 mm. The detector instead fits the projected silhouette of a cube of known edge
and height (convex hull of the 8 projected corners) to the blob. Height is known: mat surface, or the sum of the edges
below in a pile. Camera intrinsics come from the entity (transform + `VerticalFOV`), not from calibration; V0 checks
that the engine renders with the same model.

Original options as planned:

1. **Camera: orthographic top-down (recommended) or perspective?** Ortho maps pixels linearly to table XZ (no intrinsics
   calibration, top-face size independent of height). Perspective looks more like a real robot but needs calibration;
   it can be done through `CameraComponent.ToScreenSpace` on known points.
2. **Equal-size piles are invisible from above** (no depth): the lower cube is hidden until the top one is removed, and
   the planner would aim one cube too low. Options: (a) in vision mode, the generator only piles strictly smaller on
   larger (lower cube always shows as a ring); (b) a second, side camera for heights; (c) descend until finger contact.
   Recommended: (a) first, (c) later as a robustness feature.
3. **Resolution**: about 1 mm/pixel over the mat (≈ 512 × 900 for ortho) to read 5 mm rings and 3° yaw. Lower if V0 shows
   capture is slow.

## 5. Risks

- Capture cost stalls the fixed-step loop: keep captures per decision, not per frame; measure in V0.
- The arm or its shadow occludes cubes: capture with the arm retreated; keep last-seen poses; lighting fixed in the scene.
- Colour thresholds vs lighting/tonemapping: calibrate thresholds from one captured frame in V0, store them in settings.
- The overlay text and DebugRenderer markers may appear in the capture if the vision camera sees them: put the vision
  camera where the overlay isn't (the overlay is parented to the main camera), and check whether DebugRenderer draws
  into camera captures.

## 6. Demo story

"The planner used to read positions from the engine. Now it looks: here's what it sees (markers), here's how far off it
is (mm on the overlay), and the benchmark with vision on vs off on the same 50 seeds." The evaluator still uses ground
truth, so perception errors show up as honest failures, not hidden ones.

## 7. Built so far (2026-10-12)

### Perception/ (engine-free, unit-tested: `PerceptionTests`, 782/782 total)

| File | What |
|---|---|
| `PinholeCamera` | Project / back-project, pixel convention (u right, v down, top-left origin), `LookingDown` |
| `VisionRig` | The camera's authored pose: (0.5, 2.0, 0) looking down, vfov 27.5°, 512×900 (SceneBuilder writes the same) |
| `RgbaImage` | Pixels, row-order fix (`FromCapture(bottomUp)`), PNG writer, debug drawing |
| `SyntheticRenderer` | Ray-casts the workspace (mat, markers, cubes, extra boxes) through the camera: the test double for the engine camera, and the "prediction" V0 compares a real frame with |
| `ColorClassifier` | Pixel → PixelClass (4 cube colours, 4 marker colours, background) by nearest reference in the HSV cone; 32³ LUT; `Calibrate` learns references from one real frame + its predicted render |
| `CubeDetector` | Clusters → silhouette model fit (pattern search on x, z, yaw; sub-pixel row coverage; pixels of cubes above don't count against the one below); size by best fit; piles by size assignment |
| `CaptureCheck` | V0's analysis of one real frame: blank?, row order, camera-model scale/offset, calibration, detection error, annotated images, report |

**V1 gate met on synthetic frames** (anti-aliased, noisy, 8 generated seeds = 98 cubes): every cube found, mean position
error 0.10 mm, max 0.64 mm, max yaw error 1.4° (gate: 3 mm, 3°). Also: Large cubes at all four mat corners, every size told
apart alone, 2- and 3-cube piles (lower cubes seen only as rings), a half-hidden cube is never reported with a wrong pose,
a same-colour stack is never split into wrong cubes, detection < 500 ms per frame. Calibration recovers ≥ 99.5 % pixel
accuracy under a different light.

### Vision/ (engine side) + scene

- `VisionSensor`: wraps `CaptureBridge.TryCaptureCameraRgba` (signature and `out` parameters checked in the DLL metadata),
  builds the `PinholeCamera` from the entity, times each capture (measurement only).
- `VisionSpike`: one capture attempt per sim step until 5 succeed, then `CaptureCheck` and output files.
- `EpisodeManager` → **Vision** group: `VisionCaptureSpike`, `VisionCameraTag`, `VisionWidth/Height`; runtime button
  **"Vision: capture + check now"** (any time, doesn't pause the task).
- `Tools/SceneBuilder` writes `LS_Camera_Vision` (non-primary). Verified offline: re-running it on the current scene only
  adds that entity.

### Running V0 (editor)

1. Close the editor, `dotnet run --project Tools/SceneBuilder -- LuckySortStack`, reopen. (Or add a camera by hand: tag
   `LS_Camera_Vision`, position (0.5, 2, 0), rotation (−90°, 0, 0), perspective vfov 27.5, not primary.)
2. `dotnet build LuckySortStack/Assets/Scripts/Client/LuckySortStack.csproj`.
3. EpisodeManager: `VisionCaptureSpike` on, `PileCount` 0 for the first run, `BenchmarkEpisodes` 1. Press Play.
4. Read the `[Vision]` log lines and open `Recordings/<time>_vision/annotated.png`. What each outcome means:

| Report says | Meaning | Next |
|---|---|---|
| `BLANK` or "No capture succeeded" | the bridge doesn't render a non-primary camera (or needs the gRPC server) | try `Primary: true` on the vision camera for one run; try the gRPC server toggle |
| Row order TOP-DOWN / BOTTOM-UP | settles that unknown; `CaptureCheck` handles both | record in LUCKY_API_NOTES §11 |
| Camera model `MISMATCH`, scale ≈ 0.9 or 1.1 | engine uses the FOV/aspect differently (e.g. capture keeps the viewport aspect) | compare `capture_raw.png` size with the request; fix the model, not the detector |
| Camera model OK, errors ≤ 3 mm | V0 + V1 gate met on real pixels | V2 |
| Cubes NOT FOUND / large misfit | colours: tonemapping, shadows, markers vs cubes | look at `labels.png`; `classifier.txt` holds the calibrated references |

Also note from the run: capture time (ms), whether DebugRenderer markers appear in the capture (turn `ShowMarkers` off if
they do), whether the arm's home pose hides any part of the mat.

### V2 design note (for the next step)

The state machine reads object poses at moments a top-down camera can't see: during centring (arm overhead), grasp
verification (cube in the hand), placement settling (arm still above the stack). Proposal: **vision supplies the
planning inputs** (where to pick, yaw, finger-clearance neighbours, support graph, free space), refreshed by a capture
whenever the arm is clear (V3), last-seen pose kept when a cube is hidden. **Contact checks** (grasp verified, placement
settled, stack drift) keep their current source, the stand-in for the gripper width / force sensing a real cell would
use there, and are labelled as such. The evaluator stays on ground truth (rule 4). Implementation: a second
`IWorldState` for verification in `TaskStateMachine` (defaults to the same one, so nothing changes with vision off).
