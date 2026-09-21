# Installing and describing AI models

No neural weights ship with this app. They are large, separately licensed and
upgradable on their own schedule, and bundling them would tie a model version
to an app version for no benefit. Instead the app discovers model files at
runtime.

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

## Known models

| Role | File name the catalogue expects | Notes |
|---|---|---|
| Object detection | `yolov8n_float16.tflite` | Recommended default. ~10 ms on the S23 GPU at 640². |
| Object detection | `yolov8s_float16.tflite` | Better on small, distant objects; ~2.5× the cost. |
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
| `labelVocabulary` | `coco` or `driving`. Selects the table that maps the model's labels onto this project's `ObjectClass` set. A label with no entry is *dropped*, not forced into `unknown`, so a COCO detector does not fill the world model with sofas. |
| `extra` | Format-specific: `griddingNum`, `rowAnchorCount`, `laneCount`, `rowAnchorStart`, `rowAnchorEnd`, `outputWidth`, `outputHeight`, `depthScale`, `depthOffset`. |

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
