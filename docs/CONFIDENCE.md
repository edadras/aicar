# The confidence system

Nothing the AI produces is treated as ground truth. Every perception output
carries a confidence, those confidences propagate, and when the aggregate
falls far enough the stack says so and stops proposing actions.

This is not decoration. It is the mechanism by which a monocular system on a
phone can be honest about what it does and does not know.

## Per-stage confidences

| Stage | What the number is built from |
|---|---|
| Detection | The detector's own score, aggregated per frame. An empty *non-degraded* result scores high — "the road ahead is clear" is a legitimate, confident observation. An empty *degraded* result scores zero. |
| Lane boundary | Fit residual, number of supporting points, longitudinal extent, bird's-eye coverage; multiplied by 0.8 when the camera is uncalibrated. |
| Lane model | For two boundaries, weighted towards the weaker one. NO_LANE_MODE is capped well below a marked lane. |
| Drivable area | Mean per-sample segmentation confidence × a range factor. A corridor reaching 8 m ahead tells the planner very little however confident each sample is. |
| Depth map | Reduced chi-squared of the ground-plane fit. Falls off near the top of the frame, where inverse depth saturates, and at the borders. Zero until fitted. |
| Fused distance | Relative precision, cue agreement (reduced chi-squared), and how many *independent* cues contributed. |
| Track | Detector score blended over time; velocity confidence from the filter covariance × track age. A one-frame-old track has no usable velocity at all. |
| Lane relation | The object's localisation confidence and the road model's, whichever is worse. A lane relation derived from a guessed lane is a guess. |
| Path | The road model's confidence, discounted by how constrained the path was. Threading a 0.5 m gap is a much weaker proposal than running down an empty lane. |
| Decision | Evidence behind the winning candidate: detector score × distance confidence × lane-relation confidence. |
| Sign value | Template match score × agreement across repeated observations. Repetition matters far more than any one frame's score. |
| Light colour | Hue concentration and aspect position, combined as independent evidence when they agree and collapsed when they disagree. |

## Combination rules

Two rules, used deliberately:

- **Independent evidence** (noisy-OR): `1 − (1−a)(1−b)`. Two *different* cues
  supporting the same hypothesis. Hue and lamp position agreeing on "red".
- **Conjunctive**: `a × b`. A result only as good as its weakest input. A
  distance needing both a detection *and* a depth map.

Using the wrong one is a real error: a conjunctive combination of ten cues
approaches zero regardless of how good they are, and a noisy-OR of a chain of
dependencies approaches one regardless of how bad they are.

## The autonomy roll-up

`AutonomyConfidence.compute` takes five subsystem scores with weights
reflecting what control actually depends on:

```
perception 0.28   lanes 0.22   planning 0.20   depth 0.18   egoMotion 0.12
```

The weighted mean alone would hide a single catastrophic subsystem, so the
result is pulled towards the weakest input:

```dart
overall = min(weighted, 0.5 * weighted + 0.5 * worst)
```

The stack is only as trustworthy as the thing it is worst at.

Below `ConfidenceThresholds.autonomyFloor` (0.40) the HUD shows
**AUTONOMY CONFIDENCE LOW**, naming the weakest subsystem, and the decision
engine produces `UNCERTAIN` — which never commands acceleration.

Lighting is applied here rather than inside each stage, so the reason is
visible in one place: below 55 mean luminance (night) the camera-derived
scores are scaled down, and above 235 (blown-out highlights entering a tunnel
or against a low sun) they are scaled by 0.7.

## Thresholds

```dart
usable        = 0.45   // below this, a result may not influence control
high          = 0.75   // above this, act without corroboration
autonomyFloor = 0.40   // below this, the whole stack declares itself unsure
laneUsable    = 0.50   // a lane must clear this to generate a path
detectionFloor= 0.30   // detections below this are dropped before tracking
```

## Why uncertainty must never reduce apparent risk

This deserves its own note because getting it backwards is easy and was a real
bug in this codebase.

Collision assessment inflates an object's footprint by how poorly it is
localised — more doubt, bigger box, earlier warning. But cut-in detection
originally used that same inflated gap to decide whether an object *started*
outside our path. A large uncertainty cone made a doubtful object appear to
have always been in our path, so it never counted as entering it, and a less
certain observation produced a **lower** risk than a confident one.

Cut-in detection now uses nominal geometry; inflation only affects TTC and the
reported closest approach, both of which are monotonic in the safe direction.
The test `a poorly-localised object yields a more cautious assessment` asserts
the invariant directly.

## Asymmetric costs

Where the cost of an error is not symmetric, the estimator is not either:

- When depth cues disagree about a **vulnerable road user**, the fused
  distance is pulled towards the nearest plausible reading. Not applied to
  vehicles, where a pessimistic range would cause constant false braking.
- Vulnerable road users escalate one TTC threshold earlier, because the
  consequence is worse and their next move is less predictable.
- A speed limit needs three consistent readings at 70 % confidence before it
  influences anything. Acting on a misread limit is worse than not reading it.
- A traffic light needs three stable frames, 60 % colour confidence and 55 %
  relevance confidence. A single red frame among greens is a reflection, and
  acting on it would produce phantom braking mid-junction.
