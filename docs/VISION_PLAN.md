# Milestone 9: Vision (plan Stretch A + §14), build plan

**Status: planned 2026-10-12, not started.** Goal: the robot finds the cubes and the free space on the table with its own
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

## 4. Decisions to make at the start of the session

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
