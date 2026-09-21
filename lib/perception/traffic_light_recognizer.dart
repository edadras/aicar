import 'dart:math' as math;

import '../ai/interfaces/traffic_light_detector.dart';
import '../ai/model_descriptor.dart';
import '../camera/camera_calibration.dart';
import '../camera/camera_frame.dart';
import '../core/confidence.dart';
import '../core/geometry.dart';
import 'detection.dart';
import 'object_class.dart';
import 'signal_relevance.dart';
import 'traffic_light.dart';

/// Classifies the illuminated aspect of a detected traffic light, and decides
/// whether that light governs the ego vehicle.
///
/// Colour classification is done classically rather than with a network, and
/// deliberately so: the signal is a saturated point light source against a
/// dark housing, which is close to the ideal case for colour analysis, and a
/// classifier that can explain itself ("hue 8°, in the top third of the
/// housing") is far easier to trust and debug than one that cannot.
///
/// Two cues are combined, because either alone fails in a common situation:
///  * **Hue** of the brightest saturated blob. Fails when the sun is directly
///    behind the signal, washing every lamp towards white.
///  * **Vertical position** of that blob within the housing. Fails for
///    single-aspect and horizontal signals.
/// Agreement between them is what produces a confident answer; disagreement
/// correctly produces a low-confidence one.
class TrafficLightRecognizer extends TrafficLightDetector {
  TrafficLightRecognizer({
    this.minBoxHeightPixels = 8,
    this.saturationThreshold = 0.28,
    this.valueThreshold = 0.42,
  });

  final int minBoxHeightPixels;
  final double saturationThreshold;
  final double valueThreshold;

  /// Per-light state so a colour must be stable before it is acted on.
  final Map<int, _LightMemory> _memory = <int, _LightMemory>{};
  int _nextId = 1;

  @override
  String get modelId => 'classical-traffic-light';

  @override
  String get displayName => 'Traffic light recogniser (hue + aspect position)';

  @override
  ModelRole get role => ModelRole.trafficLightClassification;

  @override
  ModelDescriptor? get descriptor => null;

  @override
  bool get isReady => true;

  @override
  String? get unavailableReason => null;

  @override
  Future<void> load() async {}

  @override
  Future<void> close() async {}

  /// Planned-path geometry used to judge relevance. Updated by the pipeline
  /// each frame before [detectLights] runs.
  Polynomial? egoPathCenterline;
  double egoLaneHalfWidth = 1.75;

  /// Distance to the junction the stack believes is ahead, and how sure it
  /// is. Set by the pipeline from the intersection estimate.
  ///
  /// This is the single biggest improvement available to relevance. A signal
  /// head only governs an approach to a junction, so knowing where the
  /// junction is turns "how far off my path is it?" — which a head mounted
  /// on a gantry over the cross street can easily pass — into "does it belong
  /// to the junction I am approaching?", which it cannot.
  double? junctionDistanceMeters;
  double junctionConfidence = 0;

  static const SignalRelevanceResolver _relevanceResolver =
      SignalRelevanceResolver();

  @override
  Future<List<TrafficLight>> detectLights(
    CameraFrame frame, {
    List<Detection> candidates = const <Detection>[],
  }) async {
    if (frame.format != PixelFormat.rgb888) {
      // Colour is the whole method; a luminance-only frame cannot support it.
      return const <TrafficLight>[];
    }

    final List<TrafficLight> out = <TrafficLight>[];
    final Set<int> seen = <int>{};

    for (final Detection d in candidates) {
      if (d.objectClass != ObjectClass.trafficLight) continue;
      final double boxHeightPx = d.box.height * frame.height;
      if (boxHeightPx < minBoxHeightPixels) continue;

      final _AspectAnalysis? analysis = _analyseHousing(frame, d.box);
      if (analysis == null) continue;

      final int id = _matchOrCreateId(d.box);
      seen.add(id);
      final _LightMemory memory =
          _memory.putIfAbsent(id, () => _LightMemory(box: d.box));

      final (TrafficLightColor color, double colorConfidence) =
          _resolveColor(analysis);

      memory.update(box: d.box, color: color);

      final (double? distance, double? lateral) =
          _geometry(frame.calibration, d.box);

      final (TrafficLightRelevance relevance, double relevanceConfidence) =
          _resolveRelevance(
        lateralOffset: lateral,
        distance: distance,
        boxHeightPx: boxHeightPx,
        aspectRatio: analysis.aspectRatio,
        memory: memory,
      );

      out.add(TrafficLight(
        id: id,
        color: color,
        arrow: analysis.arrow,
        box: d.box,
        confidence: Confidence(d.score, source: 'detector'),
        colorConfidence: Confidence(colorConfidence, source: 'hue+position'),
        relevance: relevance,
        relevanceConfidence:
            Confidence(relevanceConfidence, source: 'path-geometry'),
        frameId: frame.id,
        timestampMicros: frame.timestampMicros,
        distanceMeters: distance,
        lateralOffsetMeters: lateral,
        observationCount: memory.observations,
        stableColorFrames: memory.stableFrames,
      ));
    }

    _memory.removeWhere((int id, _LightMemory m) => !seen.contains(id));
    return out;
  }

  /// Find the illuminated lamp inside the housing and describe it.
  _AspectAnalysis? _analyseHousing(CameraFrame frame, BoundingBox box) {
    final int x0 = (box.left * frame.width).floor().clamp(0, frame.width - 1);
    final int x1 = (box.right * frame.width).ceil().clamp(x0 + 1, frame.width);
    final int y0 = (box.top * frame.height).floor().clamp(0, frame.height - 1);
    final int y1 =
        (box.bottom * frame.height).ceil().clamp(y0 + 1, frame.height);

    final int w = x1 - x0;
    final int h = y1 - y0;
    if (w < 2 || h < 4) return null;

    double bestScore = 0;
    double sumHue = 0;
    double sumWeight = 0;
    double sumY = 0;
    double sumX = 0;
    int litPixels = 0;
    // Circular mean accumulators: hue is an angle, so averaging 350° and 10°
    // arithmetically would give 180° — cyan — which is nonsense.
    double hueCos = 0;
    double hueSin = 0;

    for (int y = y0; y < y1; y++) {
      for (int x = x0; x < x1; x++) {
        final (int r, int g, int b) = frame.pixelAt(x, y);
        final (double hue, double sat, double val) = _rgbToHsv(r, g, b);
        if (val < valueThreshold) continue;
        if (sat < saturationThreshold) continue;

        // Weight by how "lamp-like" the pixel is: bright and saturated.
        final double weight = val * sat;
        final double radians = degToRad(hue);
        hueCos += weight * math.cos(radians);
        hueSin += weight * math.sin(radians);
        sumHue += weight * hue;
        sumWeight += weight;
        sumY += weight * (y - y0) / h;
        sumX += weight * (x - x0) / w;
        litPixels++;
        if (weight > bestScore) bestScore = weight;
      }
    }

    // A housing with almost no lit pixels is off, occluded, or a false
    // positive — all of which mean "do not claim a colour".
    final int minLitPixels = math.max(2, (w * h) ~/ 40);
    if (litPixels < minLitPixels || sumWeight <= 0) return null;

    final double meanHue =
        (radToDeg(math.atan2(hueSin, hueCos)) + 360) % 360;
    final double relativeY = sumY / sumWeight;
    final double relativeX = sumX / sumWeight;

    // Hue concentration: how tightly the lit pixels agree on a colour. A
    // washed-out or mixed blob has low concentration and must not be trusted.
    final double concentration =
        math.sqrt(hueCos * hueCos + hueSin * hueSin) / sumWeight;

    return _AspectAnalysis(
      hueDegrees: meanHue,
      hueConcentration: concentration,
      relativeY: relativeY,
      relativeX: relativeX,
      litFraction: litPixels / (w * h),
      aspectRatio: w / h,
      arrow: _inferArrow(w, h, relativeX, sumHue / sumWeight),
    );
  }

  /// Combine hue and lamp position into a colour and a confidence.
  (TrafficLightColor, double) _resolveColor(_AspectAnalysis a) {
    // --- hue vote ---
    final (TrafficLightColor hueColor, double hueScore) = _colorFromHue(a);

    // --- position vote ---
    // Only meaningful for a vertical housing; a wide one is horizontal or a
    // single aspect, where position says nothing.
    final bool verticalHousing = a.aspectRatio < 0.62;
    TrafficLightColor positionColor = TrafficLightColor.unknown;
    double positionScore = 0;
    if (verticalHousing) {
      if (a.relativeY < 0.38) {
        positionColor = TrafficLightColor.red;
        positionScore = 0.7;
      } else if (a.relativeY > 0.62) {
        positionColor = TrafficLightColor.green;
        positionScore = 0.7;
      } else {
        positionColor = TrafficLightColor.yellow;
        positionScore = 0.55;
      }
    }

    if (hueColor == TrafficLightColor.unknown) {
      // Hue unusable (sun glare). Position alone is weak evidence and is
      // reported as such rather than upgraded.
      if (positionColor == TrafficLightColor.unknown) {
        return (TrafficLightColor.unknown, 0.1);
      }
      return (positionColor, positionScore * 0.5);
    }

    if (positionColor == TrafficLightColor.unknown) {
      return (hueColor, hueScore * 0.8);
    }

    if (hueColor == positionColor) {
      // Independent agreement: combine as independent evidence.
      final double combined = 1 - (1 - hueScore) * (1 - positionScore);
      return (hueColor, clampDouble(combined, 0, 0.98));
    }

    // Disagreement. Hue is the stronger cue when it is concentrated, but the
    // result must carry the doubt rather than hide it.
    if (a.hueConcentration > 0.75) {
      return (hueColor, hueScore * 0.55);
    }
    return (TrafficLightColor.unknown, 0.2);
  }

  (TrafficLightColor, double) _colorFromHue(_AspectAnalysis a) {
    if (a.hueConcentration < 0.45) {
      return (TrafficLightColor.unknown, 0);
    }
    final double h = a.hueDegrees;
    final double quality = clampDouble((a.hueConcentration - 0.45) / 0.45, 0, 1);

    // Red wraps around 0°, amber sits near 40°, green signals are noticeably
    // blue-shifted (they are specified as "blue-green" in most standards).
    if (h >= 340 || h <= 14) {
      return (TrafficLightColor.red, 0.55 + 0.4 * quality);
    }
    if (h > 14 && h < 30) {
      // Between red and amber: genuinely ambiguous on a small, blurred lamp.
      return (TrafficLightColor.red, 0.35 + 0.2 * quality);
    }
    if (h >= 30 && h <= 62) {
      return (TrafficLightColor.yellow, 0.5 + 0.4 * quality);
    }
    if (h > 90 && h < 190) {
      return (TrafficLightColor.green, 0.55 + 0.4 * quality);
    }
    return (TrafficLightColor.unknown, 0);
  }

  /// Arrow aspects are wider than they are tall relative to a round lamp and
  /// sit off-centre in the housing. This is a weak inference and only
  /// distinguishes the obvious cases.
  TrafficLightArrow _inferArrow(
    int w,
    int h,
    double relativeX,
    double meanHue,
  ) {
    if (w < 6) return TrafficLightArrow.none;
    if (relativeX < 0.33) return TrafficLightArrow.left;
    if (relativeX > 0.67) return TrafficLightArrow.right;
    return TrafficLightArrow.none;
  }

  /// Distance and lateral offset of the signal head, from the calibration.
  ///
  /// A traffic light is not on the ground, so the ground-plane projection does
  /// not apply. Its typical mounting height is the usable prior.
  (double?, double?) _geometry(CameraCalibration cal, BoundingBox box) {
    final double boxHeightPx = box.height * cal.imageHeight;
    final double? distance = cal.distanceFromApparentHeight(
      boxHeightPixels: boxHeightPx,
      realHeightMeters: ObjectClass.trafficLight.sizePrior.height,
    );
    if (distance == null || distance < 3 || distance > 160) {
      return (null, null);
    }
    final double bearing =
        (box.centerX * cal.imageWidth - cal.cx) / cal.fx;
    return (distance, bearing * distance + cal.lateralOffsetMeters);
  }

  /// Does this signal govern us?
  ///
  /// Delegated to [SignalRelevanceResolver], which is where the reasoning
  /// lives and where it can be tested without rendering a traffic light.
  (TrafficLightRelevance, double) _resolveRelevance({
    required double? lateralOffset,
    required double? distance,
    required double boxHeightPx,
    required double aspectRatio,
    required _LightMemory memory,
  }) {
    if (lateralOffset == null || distance == null) {
      return (TrafficLightRelevance.unknown, 0.2);
    }

    final double pathLateral = egoPathCenterline?.evaluate(distance) ?? 0.0;
    final RelevanceVerdict verdict = _relevanceResolver.resolve(
      RelevanceEvidence(
        lateralOffsetMeters: lateralOffset,
        distanceMeters: distance,
        pathLateralAtDistance: pathLateral,
        egoLaneHalfWidth: egoLaneHalfWidth,
        boxHeightPixels: boxHeightPx,
        aspectRatio: aspectRatio,
        lateralDriftPerMetre: memory.lateralDriftPerMetre(
          offsetFromPath: lateralOffset - pathLateral,
          distance: distance,
        ),
        junctionDistanceMeters: junctionDistanceMeters,
        junctionConfidence: junctionConfidence,
      ),
    );

    return switch (verdict.governsEgo) {
      true => (TrafficLightRelevance.egoPath, verdict.confidence),
      false => (TrafficLightRelevance.otherPath, verdict.confidence),
      null => (TrafficLightRelevance.unknown, verdict.confidence),
    };
  }

  /// Track signal heads across frames by box overlap so that colour stability
  /// can be measured.
  int _matchOrCreateId(BoundingBox box) {
    int bestId = -1;
    double bestIou = 0.25;
    for (final MapEntry<int, _LightMemory> e in _memory.entries) {
      final double iou = e.value.box.iou(box);
      if (iou > bestIou) {
        bestIou = iou;
        bestId = e.key;
      }
    }
    return bestId >= 0 ? bestId : _nextId++;
  }

  void reset() {
    _memory.clear();
  }

  /// RGB to HSV. Hue in degrees, saturation and value in 0..1.
  static (double, double, double) _rgbToHsv(int r, int g, int b) {
    final double rf = r / 255;
    final double gf = g / 255;
    final double bf = b / 255;
    final double maxC = math.max(rf, math.max(gf, bf));
    final double minC = math.min(rf, math.min(gf, bf));
    final double delta = maxC - minC;

    double hue;
    if (delta < 1e-6) {
      hue = 0;
    } else if (maxC == rf) {
      hue = 60 * (((gf - bf) / delta) % 6);
    } else if (maxC == gf) {
      hue = 60 * ((bf - rf) / delta + 2);
    } else {
      hue = 60 * ((rf - gf) / delta + 4);
    }
    if (hue < 0) hue += 360;

    final double saturation = maxC < 1e-6 ? 0 : delta / maxC;
    return (hue, saturation, maxC);
  }
}

class _AspectAnalysis {
  const _AspectAnalysis({
    required this.hueDegrees,
    required this.hueConcentration,
    required this.relativeY,
    required this.relativeX,
    required this.litFraction,
    required this.aspectRatio,
    required this.arrow,
  });

  final double hueDegrees;
  final double hueConcentration;
  final double relativeY;
  final double relativeX;
  final double litFraction;
  final double aspectRatio;
  final TrafficLightArrow arrow;
}

class _LightMemory {
  _LightMemory({required this.box});

  BoundingBox box;
  TrafficLightColor lastColor = TrafficLightColor.unknown;
  int observations = 0;
  int stableFrames = 0;

  double? _lastOffsetFromPath;
  double? _lastDistance;

  /// How fast this head is sliding across our path, per metre we close on it.
  ///
  /// Zero for a head that governs us — it stays put over our lane as we
  /// approach. Large for one over the cross street, which sweeps sideways.
  /// Returns null until there is a previous observation far enough back to
  /// divide by.
  double? lateralDriftPerMetre({
    required double offsetFromPath,
    required double distance,
  }) {
    final double? priorOffset = _lastOffsetFromPath;
    final double? priorDistance = _lastDistance;
    _lastOffsetFromPath = offsetFromPath;
    _lastDistance = distance;

    if (priorOffset == null || priorDistance == null) return null;
    final double closed = priorDistance - distance;
    // Only meaningful while actually closing on it.
    if (closed < 1.0) return null;
    return (offsetFromPath - priorOffset) / closed;
  }

  void update({required BoundingBox box, required TrafficLightColor color}) {
    this.box = box;
    observations++;
    if (color == lastColor) {
      stableFrames++;
    } else {
      stableFrames = 1;
      lastColor = color;
    }
  }
}
