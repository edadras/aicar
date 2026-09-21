import 'dart:math' as math;
import 'dart:typed_data';

import '../ai/model_descriptor.dart';
import '../camera/image_preprocessing.dart';
import '../core/geometry.dart';
import 'detection.dart';
import 'object_class.dart';

/// Raw box straight out of a detector head, in letterboxed input pixels.
class RawBox {
  const RawBox(this.cx, this.cy, this.w, this.h, this.score, this.classIndex);
  final double cx;
  final double cy;
  final double w;
  final double h;
  final double score;
  final int classIndex;
}

/// Decodes the output tensors of the common single-stage detector heads into
/// normalised [Detection]s.
///
/// Kept as pure functions over typed data so it can be unit-tested with
/// synthetic tensors — which is the only way to verify a decoder without a
/// device and a real model.
class YoloDecoder {
  const YoloDecoder._();

  /// YOLOv8/YOLO11 head: `[1, 4 + numClasses, numAnchors]` (channels-first,
  /// the usual TFLite export) or `[1, numAnchors, 4 + numClasses]`.
  ///
  /// There is no objectness channel; the class score *is* the confidence.
  static List<RawBox> decodeV8({
    required Float32List output,
    required List<int> shape,
    required int numClasses,
    required double scoreThreshold,
  }) {
    final (int anchors, int channels, bool channelsFirst) =
        _resolveLayout(shape, numClasses + 4);
    if (anchors <= 0) return const <RawBox>[];

    final List<RawBox> boxes = <RawBox>[];
    for (int a = 0; a < anchors; a++) {
      double bestScore = 0;
      int bestClass = -1;
      for (int c = 0; c < numClasses; c++) {
        final double s = channelsFirst
            ? output[(4 + c) * anchors + a]
            : output[a * channels + 4 + c];
        if (s > bestScore) {
          bestScore = s;
          bestClass = c;
        }
      }
      if (bestClass < 0 || bestScore < scoreThreshold) continue;

      final double cx = channelsFirst ? output[a] : output[a * channels];
      final double cy =
          channelsFirst ? output[anchors + a] : output[a * channels + 1];
      final double w =
          channelsFirst ? output[2 * anchors + a] : output[a * channels + 2];
      final double h =
          channelsFirst ? output[3 * anchors + a] : output[a * channels + 3];
      boxes.add(RawBox(cx, cy, w, h, bestScore, bestClass));
    }
    return boxes;
  }

  /// YOLOv5/v7 head: `[1, numAnchors, 5 + numClasses]` with an objectness
  /// channel that multiplies the class score.
  static List<RawBox> decodeV5({
    required Float32List output,
    required List<int> shape,
    required int numClasses,
    required double scoreThreshold,
  }) {
    final (int anchors, int channels, bool channelsFirst) =
        _resolveLayout(shape, numClasses + 5);
    if (anchors <= 0) return const <RawBox>[];

    final List<RawBox> boxes = <RawBox>[];
    for (int a = 0; a < anchors; a++) {
      final double objectness =
          channelsFirst ? output[4 * anchors + a] : output[a * channels + 4];
      if (objectness < scoreThreshold * 0.5) continue;

      double bestScore = 0;
      int bestClass = -1;
      for (int c = 0; c < numClasses; c++) {
        final double s = channelsFirst
            ? output[(5 + c) * anchors + a]
            : output[a * channels + 5 + c];
        if (s > bestScore) {
          bestScore = s;
          bestClass = c;
        }
      }
      final double score = bestScore * objectness;
      if (bestClass < 0 || score < scoreThreshold) continue;

      final double cx = channelsFirst ? output[a] : output[a * channels];
      final double cy =
          channelsFirst ? output[anchors + a] : output[a * channels + 1];
      final double w =
          channelsFirst ? output[2 * anchors + a] : output[a * channels + 2];
      final double h =
          channelsFirst ? output[3 * anchors + a] : output[a * channels + 3];
      boxes.add(RawBox(cx, cy, w, h, score, bestClass));
    }
    return boxes;
  }

  /// Work out whether the tensor is `[1, C, N]` or `[1, N, C]`.
  ///
  /// Exports disagree about this, and guessing wrong silently produces
  /// garbage boxes rather than an error — so the layout is resolved from the
  /// known channel count rather than assumed.
  static (int, int, bool) _resolveLayout(List<int> shape, int expectedChannels) {
    final List<int> dims =
        shape.where((int d) => d > 1).toList(growable: false);
    if (dims.length < 2) return (0, 0, false);
    final int d0 = dims[dims.length - 2];
    final int d1 = dims[dims.length - 1];

    if (d0 == expectedChannels) return (d1, d0, true);
    if (d1 == expectedChannels) return (d0, d1, false);
    // Fall back to "the smaller dimension is the channel dimension", which is
    // true for every real detector (classes << anchors).
    return d0 < d1 ? (d1, d0, true) : (d0, d1, false);
  }

  /// Convert raw boxes into normalised detections in the original frame.
  ///
  /// Handles both pixel-unit and already-normalised model outputs, and undoes
  /// the letterbox padding.
  static List<Detection> toDetections({
    required List<RawBox> raw,
    required LetterboxResult letterbox,
    required int originalWidth,
    required int originalHeight,
    required List<String> labels,
    required ObjectClassMapping mapping,
    required int frameId,
    required int timestampMicros,
  }) {
    if (raw.isEmpty) return const <Detection>[];

    // Some exports emit 0..1, others 0..inputSide. Decide once, from the data.
    double maxCoord = 0;
    for (final RawBox b in raw) {
      maxCoord = math.max(maxCoord, math.max(b.cx, b.cy));
    }
    final double unitScale =
        maxCoord <= 1.5 ? letterbox.side.toDouble() : 1.0;

    final List<Detection> out = <Detection>[];
    for (final RawBox b in raw) {
      final double cx = b.cx * unitScale;
      final double cy = b.cy * unitScale;
      final double w = b.w * unitScale;
      final double h = b.h * unitScale;

      // Letterboxed square pixels -> original image pixels.
      final double left = (cx - w / 2 - letterbox.padX) / letterbox.scale;
      final double top = (cy - h / 2 - letterbox.padY) / letterbox.scale;
      final double right = (cx + w / 2 - letterbox.padX) / letterbox.scale;
      final double bottom = (cy + h / 2 - letterbox.padY) / letterbox.scale;

      final String label = b.classIndex >= 0 && b.classIndex < labels.length
          ? labels[b.classIndex]
          : 'class_${b.classIndex}';
      final ObjectClass? cls = mapping.map(label);
      if (cls == null) continue; // not relevant to driving

      out.add(Detection(
        objectClass: cls,
        box: BoundingBox.fromPixels(
          left: left,
          top: top,
          right: right,
          bottom: bottom,
          imageWidth: originalWidth,
          imageHeight: originalHeight,
        ).clampToUnit(),
        score: clampDouble(b.score, 0, 1),
        frameId: frameId,
        timestampMicros: timestampMicros,
        rawLabel: label,
      ));
    }
    return out;
  }

  /// TFLite Object Detection API head (SSD MobileNet, EfficientDet-Lite):
  /// four parallel tensors — boxes `[1, N, 4]` as (ymin, xmin, ymax, xmax),
  /// classes `[1, N]`, scores `[1, N]`, count `[1]`.
  ///
  /// The boxes are normalised to the **input tensor**, not to the original
  /// frame. When the frame was letterboxed to reach that tensor, the padding
  /// has to be undone or every box is shifted and squashed — which does not
  /// throw, it just looks like a badly trained model. [letterbox] carries the
  /// transform that was applied; pass `null` when the frame was stretched
  /// straight to the input size, which is what these heads actually expect.
  static List<Detection> decodeSsd({
    required Float32List boxes,
    required Float32List classes,
    required Float32List scores,
    required int count,
    required List<String> labels,
    required ObjectClassMapping mapping,
    required double scoreThreshold,
    required int frameId,
    required int timestampMicros,
    LetterboxResult? letterbox,
    int originalWidth = 0,
    int originalHeight = 0,
  }) {
    final List<Detection> out = <Detection>[];
    final int n = math.min(count, scores.length);
    for (int i = 0; i < n; i++) {
      final double score = scores[i];
      if (score < scoreThreshold) continue;
      final int classIndex = classes[i].round();
      final String label = classIndex >= 0 && classIndex < labels.length
          ? labels[classIndex]
          : 'class_$classIndex';
      final ObjectClass? cls = mapping.map(label);
      if (cls == null) continue;

      double ymin = boxes[i * 4];
      double xmin = boxes[i * 4 + 1];
      double ymax = boxes[i * 4 + 2];
      double xmax = boxes[i * 4 + 3];

      if (letterbox != null && originalWidth > 0 && originalHeight > 0) {
        final (double l, double t) = letterbox.toOriginalNormalized(
            xmin, ymin, originalWidth, originalHeight);
        final (double r, double b) = letterbox.toOriginalNormalized(
            xmax, ymax, originalWidth, originalHeight);
        xmin = l;
        ymin = t;
        xmax = r;
        ymax = b;
      }

      out.add(Detection(
        objectClass: cls,
        box: BoundingBox.fromLTRB(xmin, ymin, xmax, ymax).clampToUnit(),
        score: clampDouble(score, 0, 1),
        frameId: frameId,
        timestampMicros: timestampMicros,
        rawLabel: label,
      ));
    }
    return out;
  }

  /// Whether an SSD-style head ran out of output slots on this frame.
  ///
  /// The postprocess op writes a fixed number of slots and reports how many
  /// it filled, so "filled them all" is only suspicious when the weakest box
  /// it returned still cleared [scoreThreshold]: that means the ranking was
  /// cut off mid-way through boxes we would have kept, not that the tail was
  /// zero-score padding.
  static bool isSsdSaturated({
    required Float32List scores,
    required int count,
    required double scoreThreshold,
  }) {
    final int capacity = scores.length;
    if (capacity == 0 || count < capacity) return false;
    for (int i = 0; i < capacity; i++) {
      if (scores[i] < scoreThreshold) return false;
    }
    return true;
  }

  /// Pick the class-label mapping named by a descriptor.
  static ObjectClassMapping mappingFor(ModelDescriptor d) =>
      switch (d.labelVocabulary) {
        'driving' || 'bdd100k' => ObjectClassMapping.driving,
        _ => ObjectClassMapping.coco,
      };
}
