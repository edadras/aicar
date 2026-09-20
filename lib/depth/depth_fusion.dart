import 'dart:math' as math;

import '../camera/camera_calibration.dart';
import '../core/confidence.dart';
import '../core/geometry.dart';
import '../perception/object_class.dart';
import '../sensors/ego_motion.dart';
import '../tracking/object_track.dart';
import 'depth_map.dart';

/// One independent distance estimate with its uncertainty.
///
/// Keeping the cues separate (rather than blending them ad hoc) is what makes
/// the fused confidence meaningful: two cues that agree tighten the estimate,
/// two that disagree widen it, and a missing cue simply drops out.
class DepthCue {
  const DepthCue({
    required this.distanceMeters,
    required this.sigmaMeters,
    required this.source,
  });

  final double distanceMeters;

  /// One standard deviation, metres. This is where each cue's known weakness
  /// is encoded — the ground-plane cue's sigma explodes near the horizon, the
  /// size cue's scales with the class's size spread, and so on.
  final double sigmaMeters;

  final String source;

  double get precision => sigmaMeters <= 1e-6 ? 0 : 1 / (sigmaMeters * sigmaMeters);

  @override
  String toString() =>
      '$source: ${distanceMeters.toStringAsFixed(1)}±${sigmaMeters.toStringAsFixed(1)}m';
}

/// Result of fusing all available cues for one object.
class FusedDistance {
  const FusedDistance({
    required this.distanceMeters,
    required this.sigmaMeters,
    required this.confidence,
    required this.cues,
    required this.agreement,
  });

  final double distanceMeters;
  final double sigmaMeters;
  final Confidence confidence;
  final List<DepthCue> cues;

  /// 0..1 measure of how well the independent cues agreed. Low agreement with
  /// a tight fused sigma would be a lie, so it feeds directly into
  /// [confidence].
  final double agreement;

  String get debugLabel => cues.map((DepthCue c) => c.toString()).join(' | ');
}

/// Combines every available monocular distance cue into one estimate.
///
/// The Galaxy S23 has no depth sensor, so distance is inferred. Five
/// independent cues are available, each strong exactly where the others are
/// weak:
///
///  1. **Ground-plane geometry** — where the object's contact patch meets the
///     road. Excellent in the near field, degrades towards the horizon.
///  2. **Apparent size** — the class's physical size prior against its pixel
///     size. Works at any range and through occlusion of the base, but only
///     as tight as the class's size spread.
///  3. **Depth network** — dense and shape-aware, but relative, so it must
///     first be fitted to metric scale using the ground plane as reference.
///  4. **Track history** — the tracker's own filtered range, which integrates
///     many past frames.
///  5. **Motion parallax** — for a stationary object, the change in range must
///     equal our own displacement. This is a direct, independent metric
///     measurement and is the strongest cue available for parked vehicles and
///     roadside furniture.
///
/// Fusion is inverse-variance weighted, with a robustness step that
/// down-weights an outlier cue instead of letting it drag the estimate.
class DepthFusion {
  const DepthFusion({
    this.boxPixelError = 3.0,
    this.minSigmaMeters = 0.4,
    this.groundAnchorCount = 24,
  });

  /// Assumed bounding-box edge error in pixels. Detector boxes are not exact,
  /// and this is what propagates that into a distance uncertainty.
  final double boxPixelError;

  final double minSigmaMeters;

  /// How many road-surface points to sample when fitting a relative depth map
  /// to metric scale.
  final int groundAnchorCount;

  /// Fit a relative depth map to metres using the road surface as reference.
  ///
  /// The road is the one part of the scene whose metric distance we know
  /// independently, from the calibration alone. Sampling the network's output
  /// along the centre of the carriageway therefore gives exactly the
  /// (raw, metric) pairs needed to recover the missing affine transform.
  DepthMap fitToGroundPlane({
    required DepthMap depth,
    required CameraCalibration calibration,
    double minRange = 6,
    double maxRange = 45,
  }) {
    // Deliberately gated on hasData, not isUsable: an unfitted relative map
    // is exactly the input this method exists to consume, and it reports
    // zero confidence until the fit succeeds.
    if (!depth.hasData || depth.scale == DepthScale.metric) return depth;

    final List<DepthMetricAnchor> anchors = <DepthMetricAnchor>[];
    final double step = (maxRange - minRange) / groundAnchorCount;

    for (int i = 0; i < groundAnchorCount; i++) {
      final double range = minRange + i * step;
      // Sample three lateral positions so a single obstruction (a car in our
      // lane) cannot corrupt the whole fit.
      for (final double lateral in <double>[-1.4, 0.0, 1.4]) {
        final PixelPoint? p =
            calibration.projectGroundToImage(Vec2(lateral, range));
        if (p == null) continue;
        final double nx = p.u / calibration.imageWidth;
        final double ny = p.v / calibration.imageHeight;
        if (nx < 0.02 || nx > 0.98 || ny < 0.02 || ny > 0.98) continue;

        final double raw = depth.rawAt(nx, ny);
        if (!raw.isFinite) continue;

        anchors.add(DepthMetricAnchor(
          rawValue: raw,
          metricDistance: range,
          // Near-field anchors are geometrically far more reliable.
          weight: calibration.groundDepthConfidenceAtRow(p.v),
          source: 'ground-plane',
        ));
      }
    }

    return depth.fitToMetric(anchors);
  }

  /// Fuse all cues for one tracked object.
  FusedDistance fuse({
    required ObjectTrack track,
    required CameraCalibration calibration,
    required DepthMap depth,
    required EgoMotionState egoMotion,
    double? previousDistanceMeters,
    double? previousTimestampSeconds,
    double? currentTimestampSeconds,
  }) {
    final List<DepthCue> cues = <DepthCue>[];

    final DepthCue? ground = _groundPlaneCue(track, calibration);
    if (ground != null) cues.add(ground);

    final DepthCue? size = _sizeCue(track, calibration);
    if (size != null) cues.add(size);

    final DepthCue? network = _networkCue(track, depth);
    if (network != null) cues.add(network);

    final DepthCue? history = _historyCue(track);
    if (history != null) cues.add(history);

    final DepthCue? parallax = _parallaxCue(
      track: track,
      egoMotion: egoMotion,
      previousDistanceMeters: previousDistanceMeters,
      previousTimestampSeconds: previousTimestampSeconds,
      currentTimestampSeconds: currentTimestampSeconds,
    );
    if (parallax != null) cues.add(parallax);

    if (cues.isEmpty) {
      return FusedDistance(
        distanceMeters: track.estimatedDistanceMeters,
        sigmaMeters: 999,
        confidence: Confidence(0, source: 'no-cue'),
        cues: const <DepthCue>[],
        agreement: 0,
      );
    }

    return _combine(cues, track.objectClass);
  }

  // --- Individual cues ----------------------------------------------------

  /// Where the object touches the road.
  DepthCue? _groundPlaneCue(ObjectTrack track, CameraCalibration cal) {
    final double bottomRow = track.box.bottom * cal.imageHeight;
    if (bottomRow <= cal.horizonY + 2) return null;

    final Vec2? ground = cal.projectToGround(
      PixelPoint(track.box.centerX * cal.imageWidth, bottomRow),
    );
    if (ground == null || ground.y < 0.5 || ground.y > 200) return null;

    // Propagate the box-edge pixel error into a range error using the local
    // ground resolution. Near the horizon this grows without bound, which is
    // exactly the behaviour we want the fusion to see.
    final double? resolution = cal.groundResolutionAtRow(bottomRow);
    double sigma = resolution == null
        ? ground.length * 0.3
        : resolution * boxPixelError;

    // An uncalibrated mounting makes every metric claim softer.
    if (!cal.isCalibrated) sigma *= 1.6;

    return DepthCue(
      distanceMeters: ground.length,
      sigmaMeters: math.max(minSigmaMeters, sigma),
      source: 'ground',
    );
  }

  /// Apparent size against the class's physical size prior.
  DepthCue? _sizeCue(ObjectTrack track, CameraCalibration cal) {
    final ObjectClass cls = track.objectClass;
    if (cls == ObjectClass.unknown) return null;

    final double heightPx = track.box.height * cal.imageHeight;
    final double widthPx = track.box.width * cal.imageWidth;
    if (heightPx < 4) return null;

    final PhysicalSizePrior prior = cls.sizePrior;
    final double? fromHeight = cal.distanceFromApparentHeight(
      boxHeightPixels: heightPx,
      realHeightMeters: prior.height,
    );
    if (fromHeight == null) return null;

    // Height is the more stable dimension for road users: a car's apparent
    // width swings with viewing angle, its height barely does.
    double distance = fromHeight;
    double sizeSpread = cls.sizePriorSpread;

    // For vehicles seen nearly head-on or tail-on, width corroborates height.
    final double? fromWidth = cal.distanceFromApparentWidth(
      boxWidthPixels: widthPx,
      realWidthMeters: prior.width,
    );
    if (fromWidth != null && cls.isVehicle) {
      final double ratio = fromWidth / fromHeight;
      if (ratio > 0.75 && ratio < 1.33) {
        distance = (fromHeight + fromWidth) / 2;
        sizeSpread *= 0.8; // two agreeing dimensions
      }
    }

    // Uncertainty has two parts: the prior's spread (multiplicative) and the
    // box-edge pixel error (which matters most for small, distant objects).
    final double priorSigma = distance * sizeSpread;
    final double pixelSigma = distance * (boxPixelError / heightPx);
    final double sigma = math.sqrt(
      priorSigma * priorSigma + pixelSigma * pixelSigma,
    );

    return DepthCue(
      distanceMeters: distance,
      sigmaMeters: math.max(minSigmaMeters, sigma),
      source: 'size',
    );
  }

  /// Median depth inside the box, converted to metres.
  DepthCue? _networkCue(ObjectTrack track, DepthMap depth) {
    if (!depth.isUsable) return null;
    final double? distance = depth.distanceInBox(track.box);
    if (distance == null || distance <= 0.5 || distance > 250) return null;

    final double mapConfidence =
        depth.confidenceAt(track.box.centerX, track.box.centerY);
    if (mapConfidence <= 0.05) return null;

    // Monocular depth error grows roughly with distance, and inversely with
    // how much the map as a whole is trusted.
    final double relativeError = 0.12 / math.max(mapConfidence, 0.1);
    final double sigma = distance * relativeError;

    return DepthCue(
      distanceMeters: distance,
      sigmaMeters: math.max(minSigmaMeters, sigma),
      source: 'depth-net',
    );
  }

  /// The tracker's own filtered range, which already integrates past frames.
  DepthCue? _historyCue(ObjectTrack track) {
    if (track.age < 3) return null;
    final double distance = track.estimatedDistanceMeters;
    if (distance <= 0.5) return null;

    // Confidence grows with age but saturates: an old track is not a laser
    // rangefinder, it is many noisy measurements of the same noisy cue.
    final double ageFactor = clampDouble(track.age / 12.0, 0, 1);
    final double sigma = distance * (0.22 - 0.10 * ageFactor);

    return DepthCue(
      distanceMeters: distance,
      sigmaMeters: math.max(minSigmaMeters, sigma),
      source: 'history',
    );
  }

  /// Motion parallax against known ego displacement.
  ///
  /// For a stationary object the range must shrink by exactly the distance we
  /// travelled. Inverting that gives a *metric* range from the apparent size
  /// change alone, with no size prior and no ground contact needed:
  ///
  ///   d_now = Δego / (1 - h_prev/h_now)
  ///
  /// It is only valid when the object really is stationary and we really did
  /// move, so both are checked before the cue is offered.
  DepthCue? _parallaxCue({
    required ObjectTrack track,
    required EgoMotionState egoMotion,
    double? previousDistanceMeters,
    double? previousTimestampSeconds,
    double? currentTimestampSeconds,
  }) {
    if (previousDistanceMeters == null ||
        previousTimestampSeconds == null ||
        currentTimestampSeconds == null) {
      return null;
    }
    if (track.direction != MotionDirection.stationary) return null;
    if (egoMotion.speedConfidence < 0.4) return null;

    final double dt = currentTimestampSeconds - previousTimestampSeconds;
    if (dt <= 0.02 || dt > 1.5) return null;

    final double egoDisplacement = egoMotion.speedMps * dt;
    // Too little movement and the ratio below is dominated by box noise.
    if (egoDisplacement < 0.6) return null;

    final List<TrackObservation> history = track.history.toList();
    if (history.length < 2) return null;
    final TrackObservation prev = history[history.length - 2];
    final TrackObservation now = history.last;

    final double prevHeight = prev.box.height;
    final double nowHeight = now.box.height;
    if (prevHeight <= 1e-4 || nowHeight <= 1e-4) return null;

    final double ratio = prevHeight / nowHeight;
    final double denominator = 1 - ratio;
    // Near-zero denominator means the apparent size barely changed, so the
    // estimate is numerically meaningless however tempting the formula looks.
    if (denominator.abs() < 0.02) return null;

    final double distance = egoDisplacement / denominator;
    if (distance <= 1.0 || distance > 200) return null;

    // Error propagation: sensitivity to the height ratio blows up as the
    // denominator shrinks, which correctly makes this cue weak at long range.
    final double ratioSigma = boxPixelError / (nowHeight * 720) * 2;
    final double sigma =
        (distance * ratioSigma / denominator.abs()).abs() + 0.5;

    return DepthCue(
      distanceMeters: distance,
      sigmaMeters: math.max(minSigmaMeters, sigma),
      source: 'parallax',
    );
  }

  // --- Combination --------------------------------------------------------

  FusedDistance _combine(List<DepthCue> cues, ObjectClass objectClass) {
    // Robustness pass: a cue far from the weighted median is down-weighted
    // rather than trusted. A single bad depth-map median should not move the
    // estimate by 20 m.
    final List<DepthCue> sorted = List<DepthCue>.from(cues)
      ..sort((DepthCue a, DepthCue b) =>
          a.distanceMeters.compareTo(b.distanceMeters));
    final double median = sorted[sorted.length ~/ 2].distanceMeters;

    final List<double> weights = <double>[];
    for (final DepthCue c in cues) {
      double w = c.precision;
      final double deviation = (c.distanceMeters - median).abs();
      final double tolerance = math.max(2.0, median * 0.35);
      if (deviation > tolerance) {
        // Smooth, not binary: a cue that is somewhat off still contributes.
        w *= math.exp(-(deviation - tolerance) / tolerance);
      }
      weights.add(w);
    }

    double weightSum = 0;
    double weighted = 0;
    for (int i = 0; i < cues.length; i++) {
      weightSum += weights[i];
      weighted += weights[i] * cues[i].distanceMeters;
    }
    if (weightSum <= 0) {
      return FusedDistance(
        distanceMeters: median,
        sigmaMeters: 50,
        confidence: Confidence(0.05, source: 'degenerate'),
        cues: cues,
        agreement: 0,
      );
    }

    final double distance = weighted / weightSum;
    final double sigma = math.max(minSigmaMeters, math.sqrt(1 / weightSum));

    // Agreement: spread of the cues relative to what their own sigmas claim.
    double agreement = 1.0;
    if (cues.length >= 2) {
      double chiSquared = 0;
      for (final DepthCue c in cues) {
        final double z =
            (c.distanceMeters - distance) / math.max(c.sigmaMeters, 0.1);
        chiSquared += z * z;
      }
      final double reduced = chiSquared / (cues.length - 1);
      // Reduced chi-squared near 1 means the cues agree within their stated
      // uncertainties; much larger means at least one of them is wrong.
      agreement = clampDouble(1.0 / (1.0 + math.max(0, reduced - 1) / 2), 0, 1);
    } else {
      // A single cue cannot be cross-checked, so it never scores full marks.
      agreement = 0.5;
    }

    // Final confidence: precision relative to the distance, modulated by
    // agreement and by how many independent cues contributed.
    final double relativePrecision =
        clampDouble(1.0 - sigma / math.max(distance * 0.35, 1.5), 0, 1);
    final double diversity = clampDouble((cues.length - 1) / 3.0, 0, 1);

    final double confidence = clampDouble(
      0.45 * relativePrecision + 0.35 * agreement + 0.20 * diversity,
      0,
      1,
    );

    // Asymmetric error cost. Over-estimating the distance to a pedestrian is
    // far worse than under-estimating it, so when the cues disagree about a
    // vulnerable road user the fused value is pulled towards the nearest
    // plausible reading rather than the statistical centre. This biases the
    // estimate deliberately, and only for the classes where the asymmetry is
    // real — it is not applied to vehicles, where a pessimistic range would
    // cause constant false braking in traffic.
    double reported = distance;
    if (objectClass.isVulnerable && agreement < 0.75) {
      final double nearest = sorted.first.distanceMeters;
      final double pull = clampDouble(1.0 - agreement, 0, 1) * 0.5;
      reported = distance + (nearest - distance) * pull;
    }

    return FusedDistance(
      distanceMeters: reported,
      sigmaMeters: sigma,
      confidence: Confidence(
        confidence,
        source: cues.map((DepthCue c) => c.source).join('+'),
      ),
      cues: cues,
      agreement: agreement,
    );
  }
}
