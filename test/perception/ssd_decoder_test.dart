import 'dart:typed_data';

import 'package:aicar/ai/model_catalog.dart';
import 'package:aicar/ai/model_descriptor.dart';
import 'package:aicar/camera/image_preprocessing.dart';
import 'package:aicar/perception/detection.dart';
import 'package:aicar/perception/object_class.dart';
import 'package:aicar/perception/yolo_decoder.dart';
import 'package:flutter_test/flutter_test.dart';

/// The bundled detector's head emits boxes normalised to its **input
/// tensor**, so the fit applied to the frame has to be undone in exactly the
/// way it was applied. These tests pin that down, because getting it wrong
/// does not throw — every box just lands somewhere slightly wrong, which is
/// indistinguishable from a badly-trained model.
void main() {
  /// One detection's worth of SSD tensors: boxes are (ymin, xmin, ymax, xmax).
  ({Float32List boxes, Float32List classes, Float32List scores}) one({
    required double ymin,
    required double xmin,
    required double ymax,
    required double xmax,
    int classIndex = 2, // 'car' in both the 80- and 90-entry label maps
    double score = 0.9,
  }) =>
      (
        boxes: Float32List.fromList(<double>[ymin, xmin, ymax, xmax]),
        classes: Float32List.fromList(<double>[classIndex.toDouble()]),
        scores: Float32List.fromList(<double>[score]),
      );

  List<Detection> decode(
    ({Float32List boxes, Float32List classes, Float32List scores}) t, {
    LetterboxResult? letterbox,
    int originalWidth = 0,
    int originalHeight = 0,
    List<String> labels = coco90Labels,
  }) =>
      YoloDecoder.decodeSsd(
        boxes: t.boxes,
        classes: t.classes,
        scores: t.scores,
        count: 1,
        labels: labels,
        mapping: ObjectClassMapping.coco,
        scoreThreshold: 0.4,
        frameId: 1,
        timestampMicros: 0,
        letterbox: letterbox,
        originalWidth: originalWidth,
        originalHeight: originalHeight,
      );

  group('decodeSsd box mapping', () {
    test('a stretched frame passes normalised boxes straight through', () {
      // No letterbox: the input tensor covers the whole frame, so tensor
      // coordinates already are frame coordinates.
      final List<Detection> out = decode(
        one(ymin: 0.25, xmin: 0.10, ymax: 0.75, xmax: 0.40),
      );

      expect(out, hasLength(1));
      expect(out.single.box.left, closeTo(0.10, 1e-6));
      expect(out.single.box.top, closeTo(0.25, 1e-6));
      expect(out.single.box.right, closeTo(0.40, 1e-6));
      expect(out.single.box.bottom, closeTo(0.75, 1e-6));
      expect(out.single.objectClass, ObjectClass.car);
    });

    test('a letterboxed frame has its padding removed', () {
      // A 640x360 frame letterboxed into 320x320: scale 0.5, so the content
      // occupies 320x180 with 70px of padding above and below.
      final LetterboxResult lb = LetterboxResult(
        bytes: Uint8List(0),
        side: 320,
        scale: 0.5,
        padX: 0,
        padY: 70,
      );

      // A box covering the full content area of the letterboxed square must
      // come back as the full frame.
      final double contentTop = 70 / 320;
      final double contentBottom = (70 + 180) / 320;
      final List<Detection> out = decode(
        one(ymin: contentTop, xmin: 0, ymax: contentBottom, xmax: 1),
        letterbox: lb,
        originalWidth: 640,
        originalHeight: 360,
      );

      expect(out, hasLength(1));
      expect(out.single.box.left, closeTo(0, 1e-6));
      expect(out.single.box.top, closeTo(0, 1e-6));
      expect(out.single.box.right, closeTo(1, 1e-6));
      expect(out.single.box.bottom, closeTo(1, 1e-6));
    });

    test('decoding a letterboxed frame as stretched misplaces the box', () {
      // The regression this guards: the same tensor decoded without the
      // letterbox is wrong, and wrong in a way that looks plausible.
      final LetterboxResult lb = LetterboxResult(
        bytes: Uint8List(0),
        side: 320,
        scale: 0.5,
        padX: 0,
        padY: 70,
      );
      final ({Float32List boxes, Float32List classes, Float32List scores}) t =
          one(ymin: 70 / 320, xmin: 0, ymax: 250 / 320, xmax: 1);

      final List<Detection> correct = decode(t,
          letterbox: lb, originalWidth: 640, originalHeight: 360);
      final List<Detection> naive = decode(one(
        ymin: 70 / 320,
        xmin: 0,
        ymax: 250 / 320,
        xmax: 1,
      ));

      expect((correct.single.box.top - naive.single.box.top).abs(),
          greaterThan(0.2));
    });

    test('a class index past the label list is dropped, not guessed', () {
      final List<Detection> out = decode(
        one(ymin: 0.2, xmin: 0.2, ymax: 0.4, xmax: 0.4, classIndex: 500),
      );
      expect(out, isEmpty);
    });

    test('the 90-entry map keeps indices aligned past COCO\'s gaps', () {
      // Index 12 is 'stop sign' in the 90-map and 'parking meter' in the
      // 80-map. Using the wrong one here is the mislabelling bug.
      expect(coco90Labels[12], 'stop sign');
      expect(cocoLabels[12], 'parking meter');
      expect(coco90Labels, hasLength(90));
      expect(cocoLabels, hasLength(80));

      for (final int i in <int>[0, 2, 5, 7, 9]) {
        expect(coco90Labels[i], cocoLabels[i],
            reason: 'the driving classes sit before the first gap');
      }
    });
  });

  group('saturation', () {
    Float32List scores(List<double> v) => Float32List.fromList(v);

    test('a partly-filled output is not saturated', () {
      expect(
        YoloDecoder.isSsdSaturated(
          scores: scores(<double>[0.9, 0.8, 0, 0]),
          count: 2,
          scoreThreshold: 0.4,
        ),
        isFalse,
      );
    });

    test('a full output padded with weak boxes is not saturated', () {
      // The postprocess op reports every slot, but the tail is padding.
      expect(
        YoloDecoder.isSsdSaturated(
          scores: scores(<double>[0.9, 0.8, 0.05, 0.01]),
          count: 4,
          scoreThreshold: 0.4,
        ),
        isFalse,
      );
    });

    test('a full output of confident boxes is saturated', () {
      expect(
        YoloDecoder.isSsdSaturated(
          scores: scores(<double>[0.9, 0.8, 0.7, 0.6]),
          count: 4,
          scoreThreshold: 0.4,
        ),
        isTrue,
      );
    });

    test('a saturated frame scores lower than a complete one', () {
      final List<Detection> dets = decode(
        one(ymin: 0.25, xmin: 0.10, ymax: 0.75, xmax: 0.40),
      );
      DetectionResult result({required bool saturated}) => DetectionResult(
            detections: dets,
            frameId: 1,
            timestampMicros: 0,
            inferenceMicros: 1000,
            modelName: 'test',
            isSaturated: saturated,
          );

      expect(result(saturated: true).frameConfidence,
          lessThan(result(saturated: false).frameConfidence));
      // Incomplete is not the same as blind: a saturated frame still carries
      // real detections and must not collapse to zero.
      expect(result(saturated: true).frameConfidence, greaterThan(0));
    });
  });

  group('bundled detector descriptor', () {
    test('is configured the way its export actually behaves', () {
      const ModelDescriptor d = ModelCatalog.efficientDetLite0;
      expect(d.isBundledAsset, isTrue);
      expect(d.assetOrFilePath, ModelCatalog.bundledDetectorAsset);
      expect(d.outputFormat, ModelOutputFormat.ssdMobileNet);
      // The TF Object Detection API resizes with fixed_shape_resizer, which
      // is a plain stretch. Letterboxing this head shifts every box.
      expect(d.inputFit, InputFit.stretch);
      expect(d.quantized, isTrue);
      expect(d.labels, same(coco90Labels));
      expect(d.inputWidth, 320);
      expect(d.inputHeight, 320);
    });

    test('every bundled model is a detector with a real asset behind it', () {
      final List<ModelDescriptor> bundled = ModelCatalog.all.values
          .where((ModelDescriptor d) => d.isBundledAsset)
          .toList();
      expect(
        bundled.map((ModelDescriptor d) => d.id),
        containsAll(<String>[
          ModelCatalog.efficientDetLite0Id,
          ModelCatalog.efficientDetLite2Id,
        ]),
      );
      for (final ModelDescriptor d in bundled) {
        expect(d.assetOrFilePath, startsWith('assets/models/'));
        expect(d.licence, isNotNull,
            reason: 'anything shipped in the APK must state its licence');
        expect(d.sizeBytes, isNotNull);
      }
    });

    test('Lite2 is the accuracy option, not the default', () {
      const ModelDescriptor d = ModelCatalog.efficientDetLite2;
      expect(d.inputWidth, 448);
      expect(d.outputFormat, ModelOutputFormat.ssdMobileNet);
      expect(d.inputFit, InputFit.stretch);
      expect(d.labels, same(coco90Labels));
      // Bigger input, so it must not be what a fresh install picks.
      expect(d.inputWidth,
          greaterThan(ModelCatalog.efficientDetLite0.inputWidth));
    });

    test('a downloadable model states its size, licence and checksum', () {
      for (final ModelDescriptor d in ModelCatalog.all.values) {
        if (!d.isDownloadable) continue;
        expect(d.sizeBytes, isNotNull,
            reason: '${d.id}: the user must be told what it costs');
        expect(d.licence, isNotNull, reason: d.id);
        expect(d.downloadSha256, isNotNull,
            reason: '${d.id}: unverified weights produce confident nonsense');
        expect(d.downloadUrl, startsWith('https://'), reason: d.id);
      }
    });

    test('YOLO heads still letterbox', () {
      expect(ModelCatalog.yolov8n.inputFit, InputFit.letterbox);
      expect(ModelCatalog.yolo11n.inputFit, InputFit.letterbox);
      expect(ModelCatalog.ssdMobileNet.inputFit, InputFit.stretch);
    });

    test('inputFit survives a JSON round trip', () {
      final ModelDescriptor back = ModelDescriptor.fromJson(
          ModelCatalog.efficientDetLite0.toJson());
      expect(back.inputFit, InputFit.stretch);
      final ModelDescriptor yolo =
          ModelDescriptor.fromJson(ModelCatalog.yolov8n.toJson());
      expect(yolo.inputFit, InputFit.letterbox);
    });
  });
}
