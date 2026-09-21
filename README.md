# Mobile Autonomous Driving Simulator

An on-device ADAS research platform for Android. The phone is mounted on the
dashboard; it watches the road through the rear camera and its own sensors,
builds a model of the world, plans a path, predicts collisions, and computes
the steering, throttle and brake an autonomous driving system **would**
command at every instant.

Those commands are drawn on the HUD and written to a log file. They are never
sent anywhere.

> **SIMULATION ONLY — no vehicle control output.**
> There is no CAN bus code, no ECU code, no OBD-II writing, and no actuator
> interface in this repository. The app requests no Bluetooth, USB or serial
> permissions, because it has nothing to talk to. The vehicle remains under
> the control of its human driver at all times. See
> [docs/SAFETY.md](docs/SAFETY.md).

Target device: **Samsung Galaxy S23** (the defaults and performance budgets
are tuned for it), Android 8.0+ / arm64.

---

## What it does

```
Camera ─► Preprocess ─► Object detection ─► Tracking ─┐
                    ├─► Segmentation ─► Drivable area ─┤
                    ├─► Lane detection ─► Road edges ──┤
                    ├─► Road markings ─► Junction ─────┤
                    ├─► Signs + lights ► Rules in force┤
                    └─► Depth ─────────► Depth fusion ─┤
                                                       ▼
IMU + GPS ─► Ego motion ──────────────────────► World model
                                                       │
Map ─► Route ─► Manoeuvre intent ──────────────────────┤
                                                       ▼
                        Local path planner ─► Collision prediction
                                                       │
                                                       ▼
                             Driving decision engine (state machine)
                                                       │
                                                       ▼
                       Simulated controller ─► Vehicle dynamics ─► HUD
```

On a live drive the HUD shows, over the camera image:

- bounding boxes with stable identities, distance, relative velocity and TTC
  (`CAR #12 / 18.4m / -4.1m/s / TTC 4.5s`);
- lane boundaries with per-line confidence, and solid/dashed classification;
- the drivable corridor as a translucent overlay;
- the planned path as a corridor with a centreline;
- traffic signs and traffic lights, with whether each one governs *our* lane;
- crossings, stop lines and speed bumps drawn as the metric patch of road
  they occupy, with distance and confidence;
- the inferred junction, as a dashed line across the road, labelled with what
  governs it and what the inference was based on;
- a hazard banner, and the decision with its reason and confidence.

Along the bottom: `SPEED`, `STEERING`, `THROTTLE`, `BRAKE`, and a steering
wheel that turns with the simulated command. Down the left, the simulated
indicators blink when the stack would be signalling, with the reason next to
them.

### Reading the road

The stack does not only avoid things — it reads the road the way a driver
does, and says what it read:

| | What it does | How |
|---|---|---|
| **Speed limit** | Reads the number off the sign, holds it for the next 800 m, and decays its confidence with distance travelled | Sign shape and colour, then digit template matching; adopted only after repeated consistent readings |
| **Pedestrian crossing** | Eases off on approach; yields outright when someone is on it or at its edge | Zebra stripes in bird's-eye view |
| **Speed bump** | Eases down to about 20 km/h before it | Transverse bands in bird's-eye view; the IMU then scores whether the jolt was really there |
| **Stop line** | Treated as evidence of a junction, never as an obligation on its own | A single solid bar across the road |
| **Red light** | Stops, then waits | Colour and position, plus whether the head governs *our* path |
| **Junction** | Slows on approach; slows more for an uncontrolled one, and more again for crossing traffic | Several weak cues combined: paint, signal heads, signs, markings that stop while the asphalt continues, road widening, vehicles crossing our path |
| **Indicating** | Signals *before* the manoeuvre, holds it through, cancels when it is done | The decision state, the route's next turn, and the planner's lateral offset |

Everything in that table carries a confidence and an explanation, and the
explanation names the evidence: *"UNCONTROLLED junction in 24 m — stop line
at 26 m"*. See [docs/ROAD_RULES.md](docs/ROAD_RULES.md) for how each one is
detected and, more usefully, where each one fails.

---

## Installing it on a phone

### Requirements

| | |
|---|---|
| Android | **8.0 (API 26)** or newer |
| Architecture | arm64 (any modern phone, including the S23) |
| Sensors | rear camera, accelerometer and gyroscope are **required**; GNSS and magnetometer are optional |
| Free space | ~120 MB for the app, plus ~2 GB per hour if you record drives with frames |

Nothing else needs installing on the phone. No root, no Termux, no separate
runtime, no companion app, and no model download. TensorFlow Lite with its
GPU/NNAPI delegates, and the EfficientDet-Lite0 detector, are both inside the
APK.

### Build and install

```bash
flutter pub get

# One APK for your own phone, straight over USB:
flutter run --release            # debug builds are several times slower

# Or produce an installable APK. --split-per-abi gives a 30 MB arm64 APK
# instead of a 74 MB fat one, because the TensorFlow Lite AAR ships native
# libraries for three architectures.
flutter build apk --release --split-per-abi
adb install build/app/outputs/flutter-apk/app-arm64-v8a-release.apk
```

Release builds are currently signed with the **debug key** so the above works
with no setup. That is fine for a research build installed over USB; a phone
installing the APK from a file manager will need "install unknown apps"
enabled for that file manager. Replace `signingConfig` in
`android/app/build.gradle.kts` before distributing it to anyone else.

### First run

1. **Grant camera access.** It is mandatory — without it there is nothing to
   perceive. Location is optional: refusing it costs absolute speed accuracy
   and navigation, and nothing else.
2. **Run the Calibration wizard.** Two minutes, once per mounting position.
   Every distance depends on it.
3. **Drive.** Object detection works out of the box: EfficientDet-Lite0 ships
   inside the APK and is selected automatically. Installing a stronger
   detector, or a depth/lane/segmentation model, is optional — see the table
   below and [docs/MODELS.md](docs/MODELS.md).

### Permissions the app requests

`CAMERA`, `ACCESS_FINE_LOCATION`, `ACCESS_COARSE_LOCATION`, `WAKE_LOCK`,
`INTERNET`, `ACCESS_NETWORK_STATE` — and nothing else. The CameraX plugin's
`RECORD_AUDIO` and external-storage permissions are explicitly removed from
the merged manifest: this app never records audio and never writes outside
its own private directory, so the list a user sees at install matches what the
app can actually do.

### AI models

One detector ships in the APK: **EfficientDet-Lite0** (COCO, 320x320, int8,
4.6 MB, Apache-2.0). It is selected automatically on first launch, so the app
detects vehicles, pedestrians, cyclists, motorcycles and traffic lights with
nothing to download.

It is deliberately the *bundleable* detector rather than the best one.
YOLOv8 and YOLO11 are stronger and the app knows how to run them, but they
are AGPL-3.0 and shipping their weights inside the APK would relicense this
whole application. Installing one at runtime is your own copy and your own
licensing decision; provenance for what is bundled is in
[assets/models/NOTICE.md](assets/models/NOTICE.md).

Every other role runs a classical fallback until you install a model. Push
`.tflite` files into the app's model directory and press refresh on the **AI
models** screen; a file whose name matches a catalogue entry — including
`efficientdet_lite0.tflite` itself — replaces the bundled one. See
[docs/MODELS.md](docs/MODELS.md) for the supported architectures, the export
commands, and the JSON sidecar format for models the catalogue does not know.

### Calibrating

Run the **Calibration** wizard once per mounting position. It takes about two
minutes and is the highest-leverage thing you can do: every distance, lane
offset and time-to-collision is computed through the camera model. With
defaults, a car at 25 m reads anywhere between 20 m and 32 m; calibrated, the
same estimate is good to about a metre.

---

## What runs with nothing installed

Out of the box — the bundled detector plus the classical stack — every stage
is functional. The table says what each one does before you install anything
more, so the limits are stated rather than discovered on the road.

| Capability | Out of the box | With a model installed |
|---|---|---|
| Object detection | EfficientDet-Lite0 at 320x320. Solid on nearby traffic and pedestrians; loses small objects past roughly 60 m, and caps at 25 boxes per frame. | YOLOv8s roughly doubles useful range and removes the box cap. |
| Lane detection | Classical IPM + matched filter. Good on clear markings, weaker at night and on worn paint. | UFLD v2 is far better at night. |
| Drivable area | Heuristic seeded region growing. Cannot distinguish asphalt from similarly-coloured pavement. | A semantic segmenter does. |
| Distance | Ground-plane geometry, class size priors and motion parallax. Lower confidence, still metric. | A depth network adds a fifth cue and sharpens occluded objects. |
| Traffic lights | Hue + aspect position. This is the intended implementation, not a fallback. | — |
| Traffic signs | Shape and colour give coarse categories; speed-limit digits by template matching. | A GTSRB classifier on detector crops. |
| Planning, decisions, vehicle simulation | Fully functional. | — |

Two honesty rules hold throughout. If you turn the detector off, or it fails
to load, the stack reports an explicitly *degraded* result rather than an
empty scene — reporting "the road is clear" because you cannot see is the most
dangerous thing a perception system can do, so autonomy confidence collapses
and the decision engine goes to `UNCERTAIN`. And if a frame fills every one of
the detector's 25 output slots with a confident box, the frame is marked
**saturated**: the boxes are real, but they are provably not all of them, and
confidence drops accordingly.

---

## Repository layout

```
lib/
  core/          safety contract, geometry, confidence, clock, profiling, Kalman
  camera/        calibration, frames, preprocessing, capture, frame scheduling
  sensors/       IMU, GPS, phone→vehicle frame, ego-motion fusion
  ai/            model descriptors, registry, inference backend, role interfaces
  perception/    object classes, detection, NMS, YOLO decode, signs, lights
  tracking/      Hungarian assignment, multi-object tracker, motion classes
  depth/         depth map, neural estimator, multi-cue fusion
  road/          bird's-eye view, lane detectors, segmenters, edges, corridor
  navigation/    route model, providers, matcher, navigation service
  world_model/   WorldState, hazards, world model builder
  planning/      planned path, lattice planner, collision prediction
  decision/      driving states, rule-based decision engine
  simulation/    simulated control, bicycle model, pure pursuit, controller
  pipeline/      pipeline config, factory, the per-frame orchestration
  recording/     schema, session recorder, session store
  replay/        replay reader, replay player and comparison
  ui/            theme, HUD overlays and panels, ten screens
  debug/         battery and thermal monitoring
android/app/src/main/kotlin/   TFLite runner with GPU/NNAPI delegates
docs/            architecture, models, safety, confidence, phases
test/            190+ tests over geometry, perception, planning and replay
```

## Documentation

- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) — how the pipeline fits together
  and why each stage is where it is.
- [docs/SAFETY.md](docs/SAFETY.md) — the simulation-only contract and how it is
  enforced.
- [docs/MODELS.md](docs/MODELS.md) — installing, exporting and describing models.
- [docs/ROAD_RULES.md](docs/ROAD_RULES.md) — speed limits, crossings, speed
  bumps, junctions and indicating: how each is read and where each fails.
- [docs/CONFIDENCE.md](docs/CONFIDENCE.md) — what every confidence number means.
- [docs/PHASES.md](docs/PHASES.md) — the fourteen development phases, each
  independently runnable.
- [docs/PERFORMANCE.md](docs/PERFORMANCE.md) — the frame budget on a Galaxy S23.

## Testing

```bash
flutter analyze
flutter test
```

The tests do not mock the algorithms. Lane detection is verified against
synthetic road scenes rendered *through the same camera model the detector
inverts*, so recovering a lane at −1.75 m from an image drawn with a lane at
−1.75 m exercises the whole IPM → filter → window → fit chain. The Hungarian
solver is checked against brute force. Depth fusion is checked against a
synthetic MiDaS-style map whose scale it has to recover. The tracker is
checked for identity stability through occlusion and for rejecting physically
impossible velocities.

## Licence and intent

This is a **research, ADAS, simulation and data-collection** tool. It is not a
driver-assistance product, it is not certified for anything, and nothing it
displays should be relied on while driving. Keep your eyes on the road.
