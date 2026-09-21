# Performance on a Galaxy S23

## The budget

At a 20 FPS inference target the whole pipeline has **50 ms** per frame. The
camera runs at 30 FPS independently and never waits.

Indicative costs with YOLOv8n on the GPU delegate, MiDaS-small on the GPU, and
SegFormer-B0 on the GPU, at the 512×288 inference resolution:

| Stage | Typical | Runs |
|---|---|---|
| Preprocess (YUV→RGB, downscale) | 4–6 ms | every frame |
| Object detection | 10–14 ms | every frame |
| Tracking | 1–3 ms | every frame |
| Lane detection (classical) | 5–8 ms | every frame |
| Segmentation | 18–26 ms | every 2nd frame |
| Depth | 22–34 ms | every 3rd frame |
| Depth fusion | 1–2 ms | every frame |
| Traffic signs (on crops) | 1–3 ms | every 2nd frame |
| Traffic lights | 1–2 ms | every frame |
| World model (×2) | < 1 ms | every frame |
| Planning | 2–4 ms | every frame |
| Collision prediction | 1–3 ms | every frame |
| Decision + control + dynamics | < 1 ms | every frame |

Amortised, that lands around 45–60 ms per cycle: roughly **15–20 FPS**
sustained, against the 10–20 FPS target.

Measure it yourself on the **Performance** screen, which shows per-stage
average and p95 against your configured budget. The p95 is the number that
explains dropped frames; the average is the one that explains the frame rate.

## Cadence, not just resolution

Segmentation and depth are the two most expensive stages *and* the two that
change most slowly — the road does not move between frames. Running them every
second and third cycle roughly doubles the achievable detection rate with no
meaningful loss in the corridor or the distances, because both are consumed as
slowly-varying context rather than as per-frame measurements.

`StageCadence` controls this; the toggles on the Performance screen let you
measure exactly what each stage costs by turning it off and watching the rate.

## The real limit is heat

A phone clamped to a dashboard in sunlight, running the GPU flat out, will
reach `THERMAL_STATUS_MODERATE` within about fifteen minutes and be clocked
down from there. Once that happens the frame rate falls regardless of any
setting, and a measurement taken then says nothing about the code.

The Performance screen samples `PowerManager.getCurrentThermalStatus` and says
so explicitly when throttling begins. **Record the thermal state alongside any
performance measurement**: 12 FPS at "normal" and 12 FPS at "severe" mean
completely different things.

Practical mitigations, in order of effect:

1. Lower the **target inference FPS**. Capping below what the device *can* do
   is the single most effective thermal control, and above about 15 FPS the
   extra frames add little for a driving HUD.
2. Drop the **inference resolution** one step. Costs some detection range on
   small, distant objects.
3. Increase the segmentation and depth **cadence**.
4. Physically: a vented cradle, out of direct sun, and charging *off* while
   driving if you do not need the range — charging adds meaningful heat.

## Battery

Camera, GPU and GNSS together draw roughly 15–25 % per hour on an S23 with the
screen at moderate brightness. The Performance screen extrapolates the
observed drain, which is more useful than any general figure because it
reflects your actual settings and conditions.

## Storage

- Structured data only: a few MB per hour.
- Data + frames at stride 2 (≈7 images/s, 70 % JPEG, 512×288): roughly
  **1.5–2.5 GB per hour**.

Image encoding runs on its own pump with a bounded queue. Under sustained load
an *image* may be dropped — the structured record still describes the frame —
because stalling perception to wait for the flash would be worse. The
Performance screen reports dropped images separately from dropped frames.

Settings has a prune control; recordings are deleted oldest-first, since
running out of space mid-drive is the worst possible time to find out.

## Where the remaining headroom is

If you need more, in rough order of payoff:

- **Run the pipeline in a background isolate.** Preprocessing currently runs
  on the main isolate because the platform's image-stream callback delivers
  there. Moving the conversion behind a `TransferableTypedData` handoff (the
  packing helper already exists in `camera_service.dart`) would return several
  milliseconds per frame to the UI thread.
- **Native preprocessing.** YUV→RGB and the bilinear resize are the largest
  pure-CPU costs. A NEON implementation behind the same interface would
  roughly halve them.
- **Quantise the detector** and move it to NNAPI. Significantly cooler, at a
  real cost in small-object recall — worth measuring on your own footage with
  the replay comparison rather than assuming.
