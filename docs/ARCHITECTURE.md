# Architecture

## The shape of the problem

A phone on a dashboard has one camera, an IMU, a GNSS receiver, and about
40 milliseconds per frame to spend. It has no LiDAR, no radar, no wheel-speed
signal, no steering-angle signal and no second camera with a useful baseline.
Every design decision below follows from that.

## Data flow

The pipeline runs once per accepted frame, in `PerceptionPipeline.process`.
Stages are injected, so that class contains the *order* and the data flow, not
the algorithms.

```
 1  Preprocess           YUV→RGB, downscale to the inference resolution
 2  Object detection     neural, or an explicitly degraded stub
 3  Tracking             ego-compensated, Hungarian association
 4  Segmentation         neural or heuristic  (every 2nd frame)
 5  Lane detection       neural or classical
 6  Drivable area        metric corridor, trimmed around obstacles
 7  Road edges           kerbs and verges from gradient + segmentation
 8  NO_LANE_MODE         corridor from other evidence, or nothing
 9  Depth                neural (every 3rd frame), fitted to the ground plane
10  Depth fusion         five cues → one distance per track, fed back
11  Signs and lights     on detector crops
12  World model          first pass
13  Planning             sampling lattice over lateral offsets
14  Collision prediction both bodies propagated, footprints tested
15  World model          second pass, with risk-annotated tracks
16  Decision             candidate generation + priority arbitration
17  Control + dynamics   pure pursuit + PI, then the bicycle model
```

### Why the world model is built twice

The first pass gives the planner something to plan against. Collision
prediction then needs the planned path, and its results (lane relation, risk,
TTC) belong *on the tracks*. Rebuilding is cheap — no inference is involved —
and it means the published `WorldState` is internally consistent: the tracks
it contains carry the risk that was computed against the path it also
contains.

## Frames of reference

Getting this wrong is the most common source of subtle, hard-to-see bugs, so
it is fixed once and used everywhere.

- **Image**: `(u, v)` pixels, origin top-left.
- **Camera**: `x` right, `y` down, `z` along the optical axis.
- **Vehicle**: `x` right of the centreline, `y` forward. Angles positive to
  the right, matching the steering sign convention.

`CameraCalibration` owns both directions of the projection and refuses to
answer above the horizon, because a ray that never meets the ground has no
ground distance and inventing one is exactly the kind of quiet fabrication
this project avoids.

Two velocity quantities are kept distinct on every track:

- `velocityWorld` — the object's own motion over the ground. This is what the
  tracking filter estimates, because it is the quantity that stays constant
  while we move and turn.
- `relativeVelocity` — the rate of change of the object's position in the
  moving, rotating vehicle frame, derived from the above including the
  yaw-induced apparent motion term. Time-to-collision is built on this one.

Conflating them makes a parked car look like it is accelerating towards you,
and makes a car ahead at matched speed look like an imminent collision. Both
were real bugs during development; both are now tested.

## Why each stage is the way it is

### Inverse perspective before lane finding

In a perspective image a lane marking converges, varies in width with
distance, and curves on a straight road. Resampling onto a metric top-down
grid removes all three at once: a marking becomes a constant-width vertical
stripe, and lane finding becomes a matched filter plus a sliding window rather
than a pile of heuristics. The sampling table depends only on calibration, so
it is built once and reused.

### Classical *and* neural implementations of the same interface

Not a fallback for its own sake. The classical lane detector and heuristic
segmenter mean the stack is useful on a device with no model files, and they
are the reference the learned versions are compared against in replay. Each
role is behind its own interface (`ObjectDetector`, `DepthEstimator`,
`LaneDetector`, `RoadSegmenter`, …) so swapping an implementation touches one
line in `PipelineFactory`.

### Monocular depth is fused, not trusted

There are five independent cues, each strong where the others are weak:

| Cue | Strong | Weak |
|---|---|---|
| Ground-plane contact | near field | towards the horizon; occluded bases |
| Class size prior | any range, occluded bases | only as tight as the class's size spread |
| Depth network | dense, shape-aware | *relative* — no metres until fitted |
| Track history | integrates many frames | not independent of its own past |
| Motion parallax | exact, metric | only valid for stationary objects, needs real movement |

They are combined by inverse-variance weighting with outlier down-weighting,
and the agreement between them (reduced chi-squared) feeds the confidence. A
relative depth map is fitted to metric scale each frame against ground-plane
anchors, and until that fit succeeds `DepthMap.distanceAt` returns `null`
rather than a number with unknown units.

### The planner is a lattice, not a line follower

A single centre-following line can only follow the lane or not. It has no way
to express "move over slightly for the cyclist". Sampling lateral offsets and
scoring them on clearance, centring, comfort and continuity does, and it makes
the reasoning inspectable — the debug overlay can show every candidate and why
it lost.

Obstacle clearance uses the object's **swept** footprint (current position to
predicted position), not just its predicted position. Testing only the
prediction leaves the space a crossing motorcycle currently occupies looking
free, and the planner steers into it — moving *towards* a fast-crossing
vehicle. That was a real bug; the swept footprint fixes it and the test
`a motorcycle filtering across the lane is flagged` pins it down.

Traffic flowing with us squarely in our own lane is skipped by the clearance
test: a lead vehicle is a speed constraint handled by the follow controller,
not a geometric obstruction. Otherwise the road reads as impassable every time
you catch up with slower traffic.

### The decision engine generates candidates and arbitrates

Rather than a nest of if/else that is impossible to reason about, each rule
produces an independent candidate carrying its own reason and confidence, and
the highest-priority one wins. Hysteresis is one-way: escalation is immediate,
de-escalation requires a dwell time. Every decision can therefore explain
itself, which is the entire point of a tool whose output a human evaluates.

### Recording is append-only JSONL

A drive can end with the app being killed, the battery dying or the phone
overheating. An append-only text stream costs at most the last line, streams
without loading the drive into memory, and is readable by anything after an
`adb pull`. `SessionStore` recovers a footer by scanning the stream when one
is missing, so an interrupted drive is still listed with its real statistics.

Writes go through a buffer drained by a single serialised writer: an `IOSink`
cannot be written to while a `flush()` is in flight, and the periodic flush
racing with per-frame writes would have killed the recording on any busy
drive.

### Replay is the point of recording

"Re-run AI" runs the currently selected models over the recorded frames,
resetting the pipeline first so no tracks or held path leak in, feeding the
recorded IMU and GPS through a fresh ego-motion estimator rather than trusting
the recorded state, and using the calibration the drive was recorded with —
every distance in the recording depends on it. `ReplayReport` then quantifies
what changed: decision agreement, mean steering and brake deltas, and the
specific state transitions.

## Threading and backpressure

The camera must never wait for inference. `FrameScheduler` implements
latest-wins: if the pipeline is busy, an incoming frame *replaces* any frame
already waiting. A stale frame is worse than no frame when you are computing
time-to-collision.

Dropped frames are therefore expected and are counted in three categories —
busy, throttled, and superseded — so the performance screen can distinguish
"the device cannot keep up" from "you asked for 10 FPS".

## Native boundary

Kotlin owns exactly one thing: running TFLite models, with the GPU and NNAPI
delegates that only exist in the Android library. Input and output cross the
method channel as raw byte buffers (`Float32List` is passed without
per-element boxing), and are written straight into the interpreter's direct
`ByteBuffer`s. All interpretation of what the tensors *mean* stays in Dart, so
changing model architecture never requires touching Kotlin.
