# Installing and describing AI models

## What already ships

One model is compiled into the APK:

| File | Role | Input | Size | Licence |
|---|---|---|---|---|
| `assets/models/efficientdet_lite0.tflite` | Object detection | 320x320 uint8 | 4.6 MB | Apache-2.0 |

`ModelRegistry` surfaces it as an installed model with no file behind it, and
auto-selects it, so detection works on first launch. It cannot be deleted;
select a different model for the role instead, or install a file with the same
name to replace it.

Why this one and not YOLOv8, which is a better detector: Ultralytics releases
YOLOv8 and YOLO11 under **AGPL-3.0**, and redistributing their weights inside
the APK would put this entire application under the AGPL. EfficientDet-Lite0
is Apache-2.0. Installing a YOLO export yourself is a different act under
different terms, and is fully supported. Provenance and licence text for
what is bundled: [`assets/models/NOTICE.md`](../assets/models/NOTICE.md).

Everything else is discovered at runtime. Model files are large, separately
licensed and upgradable on their own schedule, and bundling them all would tie
a model version to an app version for no benefit.

## Where files go

Open **AI models** in the app; the model directory path is shown and can be
copied. It is typically:

```
/data/data/com.aicar.aicar/files/models/
```

Push files there and press refresh:

```bash
adb push yolov8n_float16.tflite /sdcard/Download/
# then move it with a file manager, or for a debuggable build:
adb shell run-as com.aicar.aicar mkdir -p files/models
adb shell "cat /sdcard/Download/yolov8n_float16.tflite | run-as com.aicar.aicar tee files/models/yolov8n_float16.tflite > /dev/null"
```

A file whose **name** matches an entry in the catalogue is configured
automatically. Anything else needs a sidecar (below).

An installed file **shadows** a bundled model of the same id — push a newer
`efficientdet_lite0.tflite` and it is the one that runs. When a role has
exactly one installed file, it is selected automatically; when it has several,
the choice is yours and the bundled model keeps running until you make it.

## Known models

| Role | File name the catalogue expects | Notes |
|---|---|---|
| Object detection | `efficientdet_lite0.tflite` | **Bundled.** Apache-2.0, int8 via NNAPI, 25 boxes/frame. |
| Object detection | `yolov8n_float16.tflite` | Recommended default. ~10 ms on the S23 GPU at 640². AGPL-3.0. |
| Object detection | `yolov8s_float16.tflite` | Better on small, distant objects; ~2.5× the cost. AGPL-3.0. |
| Object detection | `yolo11n_float16.tflite` | |
| Object detection | `ssd_mobilenet_v2_int8.tflite` | Lowest power via NNAPI; misses small objects. |
| Depth | `midas_v21_small_256.tflite` | Relative inverse depth. |
| Depth | `depth_anything_v2_small.tflite` | Much better geometry, ~35 ms. |
| Lanes | `ufld_v2_culane.tflite` | Row-anchor; far better at night than the classical detector. |
| Segmentation | `segformer_b0_driving.tflite` | Drives the drivable-area overlay and NO_LANE_MODE. |
| Sign classification | `gtsrb_classifier_48.tflite` | Runs on detector crops, ~1 ms each. |

## Exporting compatible files

### YOLOv8 / YOLO11 (Ultralytics)

```bash
pip install ultralytics
yolo export model=yolov8n.pt format=tflite half=True imgsz=640
# produces yolov8n_saved_model/yolov8n_float16.tflite
```

The exported head is `[1, 84, 8400]`: four box values then 80 class scores,
with **no objectness channel**. The decoder resolves channels-first versus
channels-last from the known channel count rather than assuming, because
exports disagree and guessing wrong produces garbage boxes instead of an
error.

### MiDaS

```bash
# from the MiDaS repo
python -m tf.make_onnx_model   # then convert to .tflite, or use the
                               # published midas_v21_small_256.tflite
```

Output is **relative inverse depth**. It carries no metres at all until
`DepthFusion.fitToGroundPlane` fits it against road-surface anchors each
frame; `DepthMap.distanceAt` returns `null` before that fit succeeds.

### Ultra-Fast-Lane-Detection v2

Export to a `[1, griddingNum+1, rowAnchors, lanes]` tensor. The extra grid
cell is the "no lane in this row" class. The decoder takes the *expectation*
over the grid distribution rather than the argmax, which buys roughly a
lane-marking width of precision at 30 m.

## Sidecar format

For a model the catalogue does not know, put `<name>.json` beside
`<name>.tflite`:

```json
{
  "id": "my-detector-v3",
  "name": "My detector v3 (driving classes, 512)",
  "role": "objectDetection",
  "path": "my_detector_v3.tflite",
  "inputWidth": 512,
  "inputHeight": 512,
  "inputChannels": 3,
  "outputFormat": "yoloV8",
  "labels": ["pedestrian", "car", "truck", "bus", "motorcycle",
             "bicycle", "traffic cone", "barrier", "road debris"],
  "inputMean": [0, 0, 0],
  "inputStd": [255, 255, 255],
  "channelsFirst": false,
  "quantized": false,
  "delegate": "gpu",
  "numThreads": 4,
  "scoreThreshold": 0.35,
  "iouThreshold": 0.45,
  "labelVocabulary": "driving"
}
```

### Fields

| Field | Meaning |
|---|---|
| `role` | One of `objectDetection`, `depthEstimation`, `laneDetection`, `roadSegmentation`, `trafficSignDetection`, `trafficSignClassification`, `trafficLightClassification`. |
| `outputFormat` | `yoloV8`, `yoloV5`, `ssdMobileNet`, `relativeDepthMap`, `metricDepthMap`, `semanticSegmentation`, `laneRowAnchor`, `classification`. |
| `inputMean` / `inputStd` | Applied per channel as `(x - mean) / std`. The defaults give plain `0..1`. ImageNet normalisation is `[123.675, 116.28, 103.53]` / `[58.395, 57.12, 57.375]`. |
| `channelsFirst` | `true` for NCHW models. |
| `delegate` | `gpu` (best for float on Snapdragon), `nnapi` (best for fully-quantised int8), `cpu`. A delegate that fails to compile falls back to CPU automatically. |
| `inputFit` | `letterbox` or `stretch`. How the frame is fitted to the input tensor, and therefore how boxes are mapped back. Derived from `outputFormat` when omitted — YOLO letterboxes, SSD and EfficientDet stretch, because that is what the TF Object Detection API's `fixed_shape_resizer` does. Set it only for an unusual export. Getting it wrong does not throw; it shifts and squashes every box. |
| `labelVocabulary` | `coco` or `driving`. Selects the table that maps the model's labels onto this project's `ObjectClass` set. A label with no entry is *dropped*, not forced into `unknown`, so a COCO detector does not fill the world model with sofas. |
| `extra` | Format-specific: `griddingNum`, `rowAnchorCount`, `laneCount`, `rowAnchorStart`, `rowAnchorEnd`, `outputWidth`, `outputHeight`, `depthScale`, `depthOffset`. |

## SSD and EfficientDet heads

The TFLite Object Detection API export produces four tensors — boxes,
classes, scores, count — with boxes as `(ymin, xmin, ymax, xmax)` normalised
to the **input tensor**, not the camera frame. Two things follow.

First, the fit matters: these heads are trained with `fixed_shape_resizer`,
which stretches, so `inputFit` is `stretch` and the normalised boxes are the
frame's coordinates directly. Letterboxing one and decoding it as if it had
been stretched displaces every box by the padding — a bug that looks exactly
like a poorly-trained model.

Second, the class indices come from the **90-entry** COCO label map, which
keeps the gaps in the original category ids. Indexing the 80-entry list with
a 90-map index silently mislabels everything after the first gap: a
motorcycle becomes an airplane. `coco90Labels` in `lib/ai/model_catalog.dart`
is a byte-for-byte copy of the bundled model's own embedded `labelmap.txt`.
Its `???` placeholders have no `ObjectClassMapping` entry, so detections on
them are dropped.

The postprocess op writes a fixed number of slots — 25 for
EfficientDet-Lite0. When every slot comes back and even the weakest box
clears the score threshold, the scene was truncated: the result is flagged
`isSaturated`, which lowers frame confidence without pretending the detections
we did get are worthless. A dense intersection is exactly where a silently
truncated object list does the most damage.

## Adding a new class vocabulary

If your detector uses labels neither table covers, add an entry to
`ObjectClassMapping` in `lib/perception/object_class.dart`. That is the only
place that needs to change — every downstream stage works in terms of
`ObjectClass`, which carries the physical size prior, the plausible speed
bound and the safety margins that the depth fusion, the planner and the
collision predictor all read.

## Delegate selection on a Galaxy S23

- **Float models → `gpu`.** The Adreno 740 delegate is roughly 4× a
  multi-threaded CPU run for a detector of this size.
- **Fully int8-quantised models → `nnapi`,** which routes to the Hexagon DSP:
  the lowest power draw and the coolest option for a long drive.
- **Anything else → `cpu`** with XNNPACK, which is the automatic fallback.

The **AI models** screen lists the delegates the device actually accepted.
