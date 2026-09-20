import 'dart:math' as math;

import '../core/geometry.dart';
import 'detection.dart';
import 'object_class.dart';

/// Non-maximum suppression variants used to clean up raw detector output.
class NonMaximumSuppression {
  const NonMaximumSuppression._();

  /// Classic greedy NMS, applied per class.
  ///
  /// Per-class (rather than class-agnostic) matters on the road: a motorcycle
  /// overlapping the car it is filtering past is two real objects, and
  /// suppressing one of them would hide exactly the hazard this project cares
  /// most about.
  static List<Detection> apply(
    List<Detection> detections, {
    double iouThreshold = 0.45,
    int maxDetections = 100,
  }) {
    if (detections.length <= 1) return detections;

    final Map<ObjectClass, List<Detection>> byClass =
        <ObjectClass, List<Detection>>{};
    for (final Detection d in detections) {
      byClass.putIfAbsent(d.objectClass, () => <Detection>[]).add(d);
    }

    final List<Detection> kept = <Detection>[];
    for (final List<Detection> group in byClass.values) {
      group.sort((Detection a, Detection b) => b.score.compareTo(a.score));
      final List<bool> suppressed = List<bool>.filled(group.length, false);
      for (int i = 0; i < group.length; i++) {
        if (suppressed[i]) continue;
        kept.add(group[i]);
        for (int j = i + 1; j < group.length; j++) {
          if (suppressed[j]) continue;
          if (group[i].box.iou(group[j].box) > iouThreshold) {
            suppressed[j] = true;
          }
        }
      }
    }

    kept.sort((Detection a, Detection b) => b.score.compareTo(a.score));
    return kept.length > maxDetections
        ? kept.sublist(0, maxDetections)
        : kept;
  }

  /// Soft-NMS with a Gaussian penalty.
  ///
  /// Preferred in dense traffic: instead of deleting an overlapping box it
  /// decays its score, so a car genuinely occluded by the one in front keeps a
  /// reduced-but-nonzero detection and the tracker can hold its identity
  /// through the occlusion.
  static List<Detection> softNms(
    List<Detection> detections, {
    double sigma = 0.5,
    double scoreThreshold = 0.15,
    int maxDetections = 100,
  }) {
    if (detections.length <= 1) return detections;

    final Map<ObjectClass, List<Detection>> byClass =
        <ObjectClass, List<Detection>>{};
    for (final Detection d in detections) {
      byClass.putIfAbsent(d.objectClass, () => <Detection>[]).add(d);
    }

    final List<Detection> out = <Detection>[];
    for (final List<Detection> group in byClass.values) {
      final List<Detection> pool = List<Detection>.from(group);
      while (pool.isNotEmpty) {
        pool.sort((Detection a, Detection b) => b.score.compareTo(a.score));
        final Detection best = pool.removeAt(0);
        out.add(best);
        for (int i = 0; i < pool.length; i++) {
          final double iou = best.box.iou(pool[i].box);
          if (iou <= 0) continue;
          final double decay = math.exp(-(iou * iou) / sigma);
          pool[i] = pool[i].copyWith(score: pool[i].score * decay);
        }
        pool.removeWhere((Detection d) => d.score < scoreThreshold);
      }
    }

    out.sort((Detection a, Detection b) => b.score.compareTo(a.score));
    return out.length > maxDetections ? out.sublist(0, maxDetections) : out;
  }

  /// Drop boxes that are geometrically impossible for their class.
  ///
  /// Cheap, high-yield sanity filter: a "person" wider than they are tall, or
  /// a car occupying most of the frame while claiming to be 80 m away, is a
  /// detector artefact and would otherwise trigger a phantom emergency brake.
  static List<Detection> filterImplausible(
    List<Detection> detections, {
    double minBoxArea = 1e-5,
    double maxBoxArea = 0.9,
  }) {
    return detections.where((Detection d) {
      final BoundingBox b = d.box;
      if (b.width <= 0 || b.height <= 0) return false;
      if (b.area < minBoxArea || b.area > maxBoxArea) return false;

      final double ar = b.aspectRatio;
      return switch (d.objectClass) {
        // Upright classes: much wider than tall means a bad box.
        ObjectClass.person => ar < 1.4,
        ObjectClass.trafficCone => ar < 1.5,
        ObjectClass.trafficLight => ar < 1.6,
        // Vehicles seen from any angle stay within a broad but finite range.
        ObjectClass.car ||
        ObjectClass.van ||
        ObjectClass.truck ||
        ObjectClass.bus =>
          ar > 0.3 && ar < 6.0,
        ObjectClass.motorcycle || ObjectClass.bicycle => ar > 0.2 && ar < 3.0,
        _ => true,
      };
    }).toList();
  }

  /// Merge boxes of *different* classes that are essentially the same object.
  ///
  /// Detectors routinely fire `car` and `truck` on the same van. Keeping both
  /// would double-count the obstacle and halve the apparent gap.
  static List<Detection> mergeCrossClassDuplicates(
    List<Detection> detections, {
    double iouThreshold = 0.75,
  }) {
    final List<Detection> sorted = List<Detection>.from(detections)
      ..sort((Detection a, Detection b) => b.score.compareTo(a.score));
    final List<Detection> kept = <Detection>[];
    for (final Detection d in sorted) {
      bool duplicate = false;
      for (final Detection k in kept) {
        if (!_sameKind(d.objectClass, k.objectClass)) continue;
        if (k.box.iou(d.box) > iouThreshold) {
          duplicate = true;
          break;
        }
      }
      if (!duplicate) kept.add(d);
    }
    return kept;
  }

  static bool _sameKind(ObjectClass a, ObjectClass b) {
    if (a == b) return true;
    return a.isVehicle && b.isVehicle;
  }
}

/// Clamp every box to the image and drop degenerate ones.
List<Detection> sanitizeDetections(List<Detection> detections) {
  final List<Detection> out = <Detection>[];
  for (final Detection d in detections) {
    final BoundingBox b = d.box.clampToUnit();
    if (b.width <= 1e-4 || b.height <= 1e-4) continue;
    out.add(d.copyWith(
      box: b,
      score: clampDouble(d.score, 0, 1),
    ));
  }
  return out;
}
