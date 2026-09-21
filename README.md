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
- a hazard banner, and the decision with its reason and confidence.

Along the bottom: `SPEED`, `STEERING`, `THROTTLE`, `BRAKE`, and a steering
wheel that turns with the simulated command.

---

## Running it

```bash
flutter pub get
flutter run --release          # release: the debug build is much slower
```

The app works immediately with **no model files installed** — see below for
exactly which capabilities that costs.

### Installing AI models

No neural weights ship with the app: they are large, separately licensed, and
upgradable on their own schedule. Push `.tflite` files into the app's model
directory and press refresh on the **AI models** screen. See
[docs/MODELS.md](docs/MODELS.md) for the supported architectures, the export
commands, and the JSON sidecar format for models the catalogue does not know.

### Calibrating

Run the **Calibration** wizard once per mounting position. It takes about two
minutes and is the highest-leverage thing you can do: every distance, lane
offset and time-to-collision is computed through the camera model. With
defaults, a car at 25 m reads anywhere between 20 m and 32 m; calibrated, the
same estimate is good to about a metre.

---

## What works without models

The stack degrades in a specific, stated way rather than failing or — much
worse — quietly pretending.

| Capability | With no models installed |
|---|---|
| Lane detection | Classical IPM + matched filter. Good on clear markings, weaker at night and on worn paint. |
| Drivable area | Heuristic seeded region growing. Cannot distinguish asphalt from similarly-coloured pavement. |
| Distance | Ground-plane geometry, class size priors and motion parallax. Lower confidence, still metric. |
| Traffic lights | Hue + aspect position. This is the intended implementation, not a fallback. |
| Traffic signs | Shape and colour give coarse categories; speed-limit digits by template matching. |
| Planning, decisions, vehicle simulation | Fully functional. |
| **Object detection** | **Nothing.** Vehicles, pedestrians and obstacles are **not** detected. |

That last row is why the stack reports an explicitly *degraded* result rather
than an empty scene when no detector is installed: reporting "the road is
clear" because you cannot see is the most dangerous thing a perception system
can do. Autonomy confidence collapses and the decision engine goes to
`UNCERTAIN`.

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
