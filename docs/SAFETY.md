# Safety architecture

## The contract

This project computes what an autonomous driving system would do. It does not
do it.

```dart
// lib/core/safety.dart
static const bool simulationOnly = true;
```

That constant has no setter, no build flavour that changes it, and no
environment variable that overrides it. `SafetyMode.assertSimulationOnly()` is
a real runtime check rather than an `assert`, so the invariant holds in
release builds too, and it is called from two places: application startup, and
the constructor of every `SimulatedControlCommand`.

## What is absent, and why that is verifiable

The strongest guarantee is not a flag — it is that the code to do the
dangerous thing does not exist.

| Interface | Present? | How you can check |
|---|---|---|
| CAN bus | No | No CAN library in `pubspec.yaml`; no `can`, `j1939` or `isotp` symbol anywhere in `lib/` or `android/`. |
| ECU / OBD-II write | No | No OBD library; no ELM327 or similar command strings. |
| Bluetooth | No | `AndroidManifest.xml` requests no Bluetooth permission. An app that cannot open a Bluetooth socket cannot reach an OBD dongle. |
| USB / serial | No | No USB host permission, no `usb-device` intent filter. |
| Actuator output | No | `SimulatedControlCommand` has exactly two consumers: the HUD painter and the JSONL recorder. |

The manifest states this in a comment at the point where such permissions
would otherwise be declared, so the omission is visibly deliberate rather than
an oversight.

## How control values flow

```
DrivingDecision ─► SimulatedVehicleController ─► SimulatedControlCommand
                                                          │
                                        ┌─────────────────┴─────────────────┐
                                        ▼                                   ▼
                                 ControlStrip                      SessionRecorder
                             (pixels on a screen)                (a line in a file)
```

There is no third arrow. `SimulatedControlCommand` is also fed to
`KinematicBicycleModel` — but that integrates a *simulated* vehicle state used
only to draw where the car would have gone, and it writes to nothing.

Every recorded control line carries `"mode": "SIMULATION_ONLY=true"`, and
every session header carries the same tag plus the banner text. A dataset
produced by this app can be audited for provenance without reading the code
that produced it.

## Naming as a guard rail

Four decision states end in `Simulation`:

- `LANE_CHANGE_SIMULATION`
- `OBSTACLE_AVOIDANCE_SIMULATION`
- `EMERGENCY_BRAKE_SIMULATION`

The names are in the type system, in the HUD, and in every recorded file. A
future change that tried to make one of these actuate something would have to
first rename it, which is a much more visible act than adding a call.

## Behavioural safety within the simulation

Separately from *not driving the car*, the simulated stack is built to fail in
the right direction, because a research tool that models unsafe behaviour
teaches the wrong lesson:

- **Missing perception is reported, never assumed benign.** With no detector
  installed, `DetectionResult.noModel` is explicitly degraded; autonomy
  confidence collapses and the decision engine goes to `UNCERTAIN`. It never
  reports an empty road.
- **Uncertainty only ever increases caution.** Collision assessment inflates
  an object's footprint by how poorly it is localised, and cut-in detection
  deliberately uses *nominal* geometry — an earlier version let a growing
  uncertainty cone make an object look as though it had always been in our
  path, so a less certain observation produced a *lower* risk. That is fixed
  and tested.
- **Distance to a vulnerable road user is biased near.** When the depth cues
  disagree about a pedestrian, the fused estimate is pulled towards the
  nearest plausible reading. The error cost is not symmetric, so the estimator
  is not either. It is not applied to vehicles, where a pessimistic range
  would cause constant false braking in traffic.
- **No path means no steering proposal.** When the road model drops out, the
  HUD's steering readout goes to `—` and the wheel greys out, rather than
  holding the last value. "I do not know" looks different from "straight
  ahead".
- **Absence of lane markings never authorises an invented path.**
  `NoLaneCorridorEstimator` returns `null` unless there is positive evidence —
  drivable area, road edges, the paths other vehicles are actually taking, or
  recent lane geometry. A confident-looking path with nothing behind it is the
  most dangerous output such a system could produce.
- **`UNCERTAIN` never commands acceleration.** If the stack does not know what
  is ahead, the only defensible proposal is to ease off.
- **Escalation is immediate, de-escalation is not.** Moving to a more urgent
  state happens on the frame it is warranted. Leaving emergency braking
  requires the situation to have been resolved for a dwell period, so a
  single dropped detection does not release the brake.

## What this tool is for

Research, ADAS experimentation, simulation and data collection. It is not a
driver-assistance product, it is not certified for anything, and it must not
be relied on while driving. It is a passenger that takes notes.
