# Reading the road

How the stack works out what the road is telling it, what it does about each
thing, and — the part that matters when you are evaluating it on a real
drive — where each one stops working.

Nothing here reaches a vehicle. Every response below is a number on the HUD
and a line in a log. See [SAFETY.md](SAFETY.md).

---

## Speed limits

`lib/perception/traffic_sign_recognizer.dart`,
`lib/perception/speed_limit_reader.dart`,
`lib/perception/regulatory_context_tracker.dart`

A limit sign is visible for about a second and then governs the next
kilometre, so reading it is only half the job — the other half is holding it.

1. The sign is found by shape and colour (a red-rimmed white circle), or by a
   classifier model if one is installed.
2. The number is read by template matching on the binarised interior. This
   produces a **separate** confidence from the sign detection, and it is much
   lower: recognising a round red-rimmed sign at 60 m is easy, telling "80"
   from "30" at 60 m is not.
3. A limit is adopted only after **three consistent readings** at 70 %+ value
   confidence. A jump of more than 50 km/h from the current limit needs twice
   that evidence, because a large jump from one sign is more often a misread
   than a real change.
4. Once adopted it persists for **800 m of odometer**, not for a time, and its
   confidence decays with distance travelled — the further from the sign, the
   more likely an unobserved change has been passed.
5. School zones clamp it to 30 km/h and road works to 50, on top of the posted
   value.

A limit only constrains the decision engine once `hasSpeedLimit` is true
(confidence above 0.6). A sign we are 20 % sure said "50" is a reason to say
we do not know the limit, not a reason to brake. A limit from map data is
trusted on its own provenance.

**Where it fails:** unusual sign designs, signs on gantries that apply to a
different lane, electronic variable-message signs, and anything at night
beyond headlight range. Digit reading is the weak link, and it says so.

---

## Road-surface markings

`lib/road/road_marking_detector.dart`, `lib/road/road_marking_tracker.dart`

Crossings, stop lines and speed bumps are found in the **bird's-eye
projection** rather than in the camera image, because in a perspective image
all three look alike and change with distance, while in bird's-eye space each
has a distinct, distance-invariant signature. The discriminator is the *axis
of alternation*, which follows from how the markings are painted:

| Marking | Signature in bird's-eye space |
|---|---|
| Stop line | One solid bar across the road: a single bright run along the forward axis, no alternation across it |
| Speed bump | Bands painted *across* the road, repeated along travel: alternation on the **forward** axis, short period |
| Crossing | Bars painted *along* travel, repeated across the road: alternation on the **lateral** axis, sustained over a metre or more |

Each row of the grid is thresholded by **Otsu**, per row rather than globally.
Two reasons, and both bite in practice: brightness falls off with distance, so
one global threshold either misses far markings or invents near ones; and a
row crossing a stop line is *mostly paint*, so any "brighter than this row's
average" rule sits above the paint itself and finds nothing. A row only counts
when Otsu's two classes are genuinely far apart, which is what stops clean
asphalt from reading as a marking.

Detections are then accumulated by the tracker, which associates them across
frames by **ground position**, carrying each one towards the vehicle using
the ego motion. A marking must be seen in three frames at 35 %+ confidence
before anything acts on it. A marking not re-seen decays — unless it lies
beyond the range the grid could actually search this frame, because not
seeing something out of sight says nothing about whether it is there.

### Range

Range differs sharply between the two patterns, and it is a property of the
camera, not of the code. A crossing's stripes repeat *across* the image, where
resolution is good. A hump's bands repeat *into* the vanishing point: a whole
two-metre hump subtends about two image rows at 22 m on a 640x360 frame, so
the period is not blurred, it is gone.

| | 640x360 | 960x540 | 1280x720 |
|---|---|---|---|
| Crossing | 25 m+ | 25 m+ | 25 m+ |
| Speed bump | ~14 m | ~18 m | ~22 m |

Raising the inference resolution is the only thing that extends the speed-bump
figure. Measured on synthetic scenes rendered through the same camera model
the detector inverts; see `test/road/road_marking_detector_test.dart`.

### Scoring its own claims

When the vehicle crosses a speed bump the stack committed to, the IMU's
vertical acceleration either shows the jolt or it does not. The Developer
screen reports **predicted / felt** for the session. Nothing reads that
number — it exists so the detector can be judged rather than believed.

**Where it fails:** an unpainted hump is invisible. Worn paint at night is
unreliable. Wet asphalt reflecting a street lamp can produce a bright
transverse band. A crossing on a side street is detected and, if it does not
lie across our path, ignored.

### Paint is not the end of the road

An appearance-based segmenter sees a zebra crossing as a bright band across
the carriageway and stops classifying road there. Left alone, the corridor
would end at every crossing and the planner would report the path blocked —
braking for a piece of paint. So `DrivableAreaBuilder` **bridges** an
interruption of up to 6 m when road resumes beyond it, at reduced confidence.
A genuine end of the road has nothing beyond it, so nothing is bridged to.

---

## Traffic lights

`lib/perception/traffic_light_recognizer.dart`

Colour from hue and vertical position within the head; relevance — whether
this signal governs *our* path — from lateral position relative to the
planned path and the corridor. A light is only *actionable* when relevance
and colour are both confident and the colour has been stable for three
frames. A red light that governs a side road is detected, labelled, and not
acted on.

A green light clears a pending stop-line or give-way obligation, because at a
signalised junction the signal supersedes the sign.

---

## Junctions

`lib/road/intersection_detector.dart`

There is no "intersection" class in any detector, and there does not need to
be. A junction announces itself in several ways at once, and the combination
is far more reliable than any one of them:

| Cue | Weight |
|---|---|
| Stop line ahead | 0.55 × its confidence |
| Crossing ahead | 0.40 × its confidence |
| A signal head resolved as ours | 0.70 × its relevance |
| Stop sign in force | 0.65 |
| Give-way sign in force | 0.60 |
| Lane markings end while the asphalt runs on | 0.35 |
| Drivable corridor flares outwards | 0.30 |

Combined as a **noisy-OR**, which has the property this needs: several
independent weak cues reinforce each other, and no amount of any one of them
reaches certainty. Below 35 % nothing is reported at all — a junction
announced every few hundred metres on a straight road would train the driver
to ignore the panel.

The lane-markings cue is deliberately *proportional* (the road must run on
more than 1.6× as far, and at least 12 m further). Lane fits routinely run a
little shorter than the segmented surface on an ordinary road, and a fixed
"8 m shorter" rule calls that a junction on every straight.

The junction's distance is the **nearest** contributing cue: braking for the
far edge of a junction we are already entering would be too late.

### Approach speed

| Control | Approach |
|---|---|
| Signalised | the posted limit — the signal is not what limits us |
| Give way | 5.5 m/s |
| Stop | 3.0 m/s |
| **Uncontrolled** | 7.0 m/s |
| any of the above, with traffic crossing our path | 4.5 m/s |

The uncontrolled row is the important one. No signal does not mean the
junction is ours — it means we do not know who has priority, and the correct
response to not knowing is to arrive slowly enough to react. Equally, a green
light does not license speed past a car already crossing; crossing traffic
tightens the figure whatever the control says.

The caution *onset* is computed backwards from a comfortable deceleration, so
it starts further out the faster we are going, rather than at a fixed
distance.

**Where it fails:** roundabouts are treated as junctions rather than
modelled. Slip roads and filter lanes can read as a junction. A junction with
no paint, no signs and no visible crossing traffic — a rural crossroads — may
not reach the threshold at all, and the stack will not pretend otherwise.

---

## Indicating

`lib/simulation/turn_signal.dart`, `lib/simulation/turn_signal_planner.dart`

Indicating is not a by-product of steering. It has to happen *before* the
manoeuvre, which makes it a decision in its own right rather than something
that falls out of the controller. Three sources feed it, in priority order:

1. **Hazards** — a simulated emergency stop, or stopping in a live lane.
2. **The decision state** — `TURN_LEFT`, `TURN_RIGHT`,
   `LANE_CHANGE_SIMULATION`, `OBSTACLE_AVOIDANCE_SIMULATION`.
3. **The route** — a turn coming up, announced 3 s ahead but never less than
   30 m, which is what matters at low speed in town.
4. **The planner** — a lateral offset beyond 0.45 m means we are visibly
   moving over.

Two behaviours separate this from a fault light: a **minimum on-time** of
1.5 s, so a one-frame wobble in the planner does not blink the lamp; and
**asymmetric thresholds** — 0.45 m to start indicating, 0.15 m to stop — so
it stays lit until the manoeuvre is genuinely finished rather than cancelling
halfway through.

The blink phase is derived from the frame timestamp rather than from a UI
animation, so the HUD, the recording and a replay of that recording all show
the same lamp at the same moment.

---

## What reaches the decision engine

Each of the above becomes a candidate with a reason and a confidence, and the
highest-priority candidate wins. The reasons name their evidence, because a
recording is only reviewable if it says *why*:

```
SLOW_DOWN   Speed bump in 18 m, easing to 20 km/h                    72 %
SLOW_DOWN   UNCONTROLLED junction in 24 m — stop line at 26 m        61 %
SLOW_DOWN   SIGNALLED junction in 22 m, traffic crossing — signal
            head ahead (GREEN)                                       68 %
PEDESTRIAN_YIELD  2 people at the crossing 16 m ahead                84 %
SLOW_DOWN   64 km/h in a 50 km/h limit                               77 %
```

And each becomes a `Hazard` on the world model, which is what the HUD banner,
the side panel and the overlay all read from. A hazard carries its distance,
so the same fact appears consistently in all three.
