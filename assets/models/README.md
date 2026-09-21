# Bundled models

## What ships in the APK

| File | Role | Input | Size | Licence | |
|---|---|---|---|---|---|
| `efficientdet_lite0.tflite` | Object detection | 320×320 uint8 | 4.6 MB | Apache-2.0 | default |
| `efficientdet_lite2.tflite` | Object detection | 448×448 uint8 | 7.6 MB | Apache-2.0 | accuracy option |

Lite2 is the better detector and is **not** the default: 448×448 is roughly
twice the compute of 320×320, and on a phone clamped to a windscreen that
difference shows up as thermal throttling rather than as better detections.
Choose it deliberately on the AI models screen and watch the thermal readout.

Those are the only models bundled. It is enough for the app to detect vehicles,
pedestrians, cyclists, motorcycles and traffic lights the first time it is
launched, with nothing to download. Every other role (depth, lane, road
segmentation, sign classification) runs its classical fallback until a model
is installed — see `docs/MODELS.md`.

Models too large to bundle are offered as an **in-app download** instead:
MiDaS v2.1 small is 63 MB, which would more than triple the APK for a
capability most drives do not need. The AI models screen shows the size and
the licence, asks before fetching anything, and verifies the SHA-256 before
installing it.

## Why EfficientDet-Lite0 and not YOLO

YOLOv8 and YOLO11 are better detectors, and the app knows how to run them.
They are also **AGPL-3.0**, and redistributing their weights inside the APK
would put this whole application under the AGPL. EfficientDet-Lite0 is
Apache-2.0, so it can travel with the binary without imposing anything on the
people who build it.

Installing a YOLO export at runtime is a different act — it is the user's own
copy, and their own licensing decision. The app supports it fully; see the
**AI models** screen.

Provenance and full licence text: `NOTICE.md` in this directory.

## Adding another bundled model

Put the `.tflite` file here, add a `ModelDescriptor` to `lib/ai/model_catalog.dart`
with `isBundledAsset: true` and `assetOrFilePath` set to the asset key
(`assets/models/<file>.tflite`), and record its provenance and licence in
`NOTICE.md`. `ModelRegistry.refresh()` picks it up automatically; a model the
user installs with the same id shadows it.

Gradle is configured not to compress `.tflite` files (`androidResources {
noCompress }` in `android/app/build.gradle.kts`) because a compressed asset
cannot be memory-mapped, and the interpreter would otherwise have to copy the
whole model into the heap.
