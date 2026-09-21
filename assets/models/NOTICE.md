# Third-party model notices

This directory contains pre-trained model weights that are **not** covered by
this project's own licence. Each entry below records where the file came from
and the terms it is redistributed under.

---

## EfficientDet-Lite0 (object detection)

* **File:** `efficientdet_lite0.tflite`
* **SHA-256:** `2e04c53bfeac0ac2a30c057c7e2a777594ce39baaac35a92f74fb1e8c4fc4e0b`
* **Size:** 4,563,519 bytes
* **Source:** TensorFlow Hub —
  `https://tfhub.dev/tensorflow/lite-model/efficientdet/lite0/detection/metadata/1`
* **Copyright:** Copyright 2020 The TensorFlow Authors. All Rights Reserved.
* **Licence:** Apache License, Version 2.0
  (`https://www.apache.org/licenses/LICENSE-2.0`)
* **Architecture:** EfficientDet-Lite0, from *EfficientDet: Scalable and
  Efficient Object Detection* (Tan, Pang & Le, 2020).
* **Training data:** COCO 2017, released by its authors under
  CC BY 4.0 (images are subject to their own Flickr terms).

The file is redistributed unmodified. It carries TFLite Model Metadata,
including its own `labelmap.txt`; `coco90Labels` in
`lib/ai/model_catalog.dart` is a byte-for-byte copy of that label map, kept in
Dart so the decoder does not have to parse the metadata at runtime.

### Apache-2.0 notice

```
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
```

---

## EfficientDet-Lite2 (object detection)

* **File:** `efficientdet_lite2.tflite`
* **SHA-256:** `6fd32c84ab1eb0f7e7f3a7a20a20d7df1530daa8378728f7c79571096286bd52`
* **Size:** 7,557,887 bytes
* **Source:** TensorFlow Hub —
  `https://tfhub.dev/tensorflow/lite-model/efficientdet/lite2/detection/metadata/1`
* **Copyright:** Copyright 2020 The TensorFlow Authors. All Rights Reserved.
* **Licence:** Apache License, Version 2.0

Same architecture, head and label map as Lite0, at 448x448 instead of
320x320. Shipped as the accuracy option; not the default, because the extra
compute is also extra heat.

---

## Models the app can run but does not bundle

Listed here so the licensing position is unambiguous for anyone building a
distribution of this app. The terms below are those of each project's
**official** weights; a re-trained or third-party export may differ, so check
the one you actually install.

| Model | Licence | Bundleable? |
|---|---|---|
| YOLOv8n / YOLOv8s / YOLO11n (Ultralytics) | AGPL-3.0 | **No** — would relicense the whole app |
| SSD MobileNet V2 (TF OD API) | Apache-2.0 | Yes |
| MiDaS v2.1 small (Intel ISL) | MIT | Yes, but 63 MB — offered as an in-app download instead |
| Depth Anything V2 small | Apache-2.0 | Yes |
| Ultra-Fast-Lane-Detection v2 | MIT | Yes |
| SegFormer-B0 (NVIDIA) | NVIDIA Source Code Licence — non-commercial | **No** for a commercial build |

None of these ship in the APK. They are installed at runtime by the user, from
their own copy, under whatever terms they have accepted.
