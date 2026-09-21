# Field testing

What the synthetic suite settles, what it cannot, and what to drive.

Synthetic scenes are good at one thing a real drive is bad at: producing the
*same* difficult scene twice, so a change in behaviour can be attributed to a
change in the code. They are bad at everything else. Nothing below is a
substitute for driving.

---

## What the repository already checks

`test/robustness/adverse_conditions_test.dart` renders each condition through
the same camera model the perception code inverts, and asserts the thing that
actually matters: **when the stack stops coping, it says so.** A detector that
returns a confident lane from a rain-blurred image is far more dangerous than
one that returns nothing.

| Condition | What is simulated | What is asserted |
|---|---|---|
| Night | Headlight cone projected onto the ground; illumination scales *contrast*, not just brightness; noise that does not dim with the scene | Mean luma crosses the stack's own night threshold; lane range collapses to roughly the beam; confidence falls; lane width stays correct |
| Rain | Contrast haze, lens streaks, specular patches on wet tarmac | No confident wrong lane; heavier rain costs more confidence; reflections never read as a crossing |
| Glare | Saturated blob just above the horizon with bloom | No confident wrong lane; a blown-out frame is reported unusable rather than analysed |
| Curves | Curvature up to 1/167 m | Tracked with correct lane width; range shrinks rather than correctness |
| Worn paint | Marking luma reduced towards the asphalt | Confidence falls; near-invisible paint gives up rather than guessing |
| Traffic | Occluders placed in metres on the road plane | One boundary hidden → single-edge mode with the learned width; both hidden → `NO_LANE_MODE` |
| Stacked | Night + rain together | Worse than either alone, and the number admits it |

Two findings came out of writing those tests, and both were real:

* Lane confidence did not depend on image contrast at all. A lane traced from
  markings 25 luma above a noisy night image scored identically to one from
  markings 150 luma above daylight tarmac. There is now a signal-strength term
  in the boundary confidence.
* Specular reflections on wet tarmac read as a pedestrian crossing at 56 %,
  and a low sun read as a speed bump. Both are fixed — a stripe-consistency
  test that a real zebra passes and scattered reflections do not, and a
  saturation guard that reports a blown-out frame as unusable instead of
  reporting "no markings", which would have been a lie.

---

## What has to be driven

Everything below is invisible to a renderer. Record each one (**Recorded
drives** → the session is written as JSONL with frames), because a recording
can be re-run through a changed stack afterwards and a memory cannot.

### 1. Distance accuracy

Open **Distance accuracy** and drive twenty minutes of ordinary road with
parked cars. No props needed — the screen explains the method. Note the scale,
the per-band errors and the diagnosis. **Do this first**, because every other
measurement is downstream of it: if distances are 10 % long, so is every TTC,
every gap and every braking point.

Repeat after any change to the mount.

### 2. Thermal behaviour

The one that decides whether the app is usable at all.

* Mount as you actually would — windscreen, sun on it, no fan.
* Drive an hour, watching **Performance → Thermal governor**.
* Record: time to first step-down, the level it settles at, the sustained
  frame rate, and battery percent per hour.
* Then repeat with EfficientDet-Lite2 selected instead of Lite0, and with
  MiDaS installed, and compare. That is the only way to find out what the
  accuracy options actually cost on your phone.

### 3. Night

* Unlit rural road, lit urban road, and a tunnel entrance and exit.
* Watch for: lane range collapsing to the headlights (expected), auto-exposure
  hunting between the beam and the dark (a known weakness — try
  `lockExposureWhileDriving`), and oncoming headlights saturating the frame.
* The thing to check is not whether it works. It is whether the confidence
  numbers fall when it stops working.

### 4. Rain

* Light rain, heavy rain, and spray behind a lorry.
* Watch for: the wiper crossing the frame, droplets on the lens holding focus,
  and standing water mirroring street lights.
* Specifically confirm the stripe-consistency fix on a genuinely wet night
  road. It is the one fix in this repository tuned against simulated
  reflections rather than real ones.

### 5. Glare

* Drive west at sunset, then east at sunrise.
* Confirm the saturation guard: the marking detector should report the frame
  unusable, and the HUD should show it, rather than quietly reporting a clear
  road.

### 6. Worn and absent markings

* A resurfaced road with ghost markings from the old layout — the case most
  likely to produce a confident wrong lane.
* An unmarked residential street, to exercise `NO_LANE_MODE`.
* Roadworks with temporary yellow markings over white ones.

### 7. Traffic

* Stop-start queue, a lorry alongside covering a boundary, and a motorcycle
  filtering between lanes.
* Watch track identity: an ID that changes as a vehicle passes behind another
  breaks the velocity estimate and therefore the TTC.

### 8. Junctions

* Signalled crossroads, unsignalled crossroads, a roundabout, and a junction
  with a signal head for the cross street visible at the same time.
* That last one is the test for `SignalRelevanceResolver`. Confirm the cross
  street's red does not become ours.

### 9. The alerts themselves

The hardest thing to test synthetically, because the failure is behavioural
rather than technical: an alert policy is wrong when a driver starts ignoring
it, and no unit test can see that.

* Drive an hour in ordinary traffic with **Drive mode** and audio on.
* Count how many times it spoke. If you stopped noticing, it said too much.
* Note any alert that arrived too late to act on — that is the one number
  that matters, and it is not in any log.
* Confirm the screen and the speaker never disagreed: they are driven by the
  same policy, so a difference is a bug.

### 10. Speed limits

* A road where the limit changes, and one with a gantry sign applying to a
  different lane.
* Confirm the limit persists for the full 800 m and that its confidence
  decays with distance from the sign.

---

## What to record for each run

The app writes most of this itself; the rest is worth a note.

| | Where it comes from |
|---|---|
| FPS (camera and pipeline), latency, per-stage timings | Recorded per cycle; **Performance** |
| Thermal status and headroom, battery, drain per hour | Recorded per sample; **Performance → Device** |
| Governor level and why it chose it | Recorded per cycle |
| Distance scale and per-band error | **Distance accuracy** → export CSV |
| Weather, light, road type, traffic | Your notes |
| Phone model, mount, ambient temperature | Your notes |

A result without the conditions beside it is not a result. "12 FPS" means
completely different things at `NONE` and at `SEVERE`, which is why the
governor's decision is recorded alongside the frame rate rather than left to
be inferred.

---

## The honest summary

This repository contains no verified real-world accuracy figures, and will
not until someone drives it and publishes theirs. What it contains instead is
the instrumentation to produce them, and a test suite that fails when the
stack becomes confident about something it should not be.
