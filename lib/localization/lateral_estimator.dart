import 'dart:math' as math;

import '../core/confidence.dart';
import '../core/geometry.dart';
import '../navigation/route.dart';
import '../road/lane.dart';
import '../sensors/ego_motion.dart';
import 'lateral_state.dart';

/// Fuses camera, IMU and map matching into where we are across the road.
///
/// Each source answers a different part of the question, and none of them
/// answers it alone:
///
///  * The **camera** measures the offset within the lane directly, to a few
///    centimetres, and is the only source that does. It also drops out —
///    worn paint, a crossing, glare, a lorry alongside — and when it does,
///    the offset does not stop existing.
///  * The **IMU** bridges those dropouts. Lateral displacement is the
///    integral of `speed x sin(heading error)`, and heading error is the
///    integral of yaw rate, so bridging is a double integration: the
///    uncertainty grows as the square of the gap and the bridge is worth
///    about a second, not a minute. That is encoded here rather than left as
///    a comment — the confidence decays quadratically and the estimate is
///    discarded outright past [maxBridgeSeconds].
///  * **Map matching** supplies the road, the bearing to cross-check the
///    heading against, and (when the map has it) how many lanes there are.
///    It does not supply which lane we are in.
///
/// And **GPS supplies none of it**. A phone's fix is good to a handful of
/// metres at best, which is wider than a lane; deciding lane membership from
/// it would be confident and wrong roughly half the time. So
/// [LateralState.laneIndexFromEdge] stays null unless something actually
/// establishes it, and the normal, honest answer to "which lane am I in?" is
/// that we do not know.
class LateralEstimator {
  LateralEstimator({
    this.maxBridgeSeconds = 2.0,
    this.drivingSide = DrivingSide.right,
    this.headingAgreementDegrees = 25,
  });

  /// How long the IMU may carry the estimate without a camera observation.
  ///
  /// Two seconds at 14 m/s is 28 m of road. Beyond that the double
  /// integration has accumulated more error than a lane is wide, and an
  /// estimate wider than a lane is not an estimate.
  final double maxBridgeSeconds;

  final DrivingSide drivingSide;

  /// How far the fused heading may differ from the matched road's bearing
  /// before the match is treated as suspect.
  final double headingAgreementDegrees;

  LateralState _state = LateralState.unknownState;

  /// Confidence at the moment of the last real observation.
  ///
  /// The decay is applied to *this*, not to the running value. Applying it to
  /// the running value would compound it once per frame, so at 20 FPS a
  /// two-second bridge would decay forty times over and collapse to zero in a
  /// fraction of a second — a bridge that does not bridge.
  double _observedConfidence = 0;

  double _secondsSinceObservation = 0;
  double _bridgedOffset = 0;
  double _headingErrorRadians = 0;

  LateralState get state => _state;

  void reset() {
    _state = LateralState.unknownState;
    _observedConfidence = 0;
    _secondsSinceObservation = 0;
    _bridgedOffset = 0;
    _headingErrorRadians = 0;
  }

  LateralState update({
    required LaneDetectionResult lanes,
    required EgoMotionState ego,
    required double dtSeconds,
    RouteProgress? route,
  }) {
    final bool? headingAgrees = _headingAgreement(ego, route);

    final bool cameraUsable = lanes.overallConfidence >= 0.45 &&
        lanes.mode != LaneMode.none &&
        lanes.mode != LaneMode.noLane;

    if (cameraUsable) {
      _secondsSinceObservation = 0;
      _observedConfidence = lanes.overallConfidence;
      _bridgedOffset = lanes.egoLateralOffsetMeters;
      _headingErrorRadians = lanes.egoHeadingErrorRadians;

      final (int?, int?, double) lane = _inferLane(lanes);
      _state = LateralState(
        offsetInLaneMeters: lanes.egoLateralOffsetMeters,
        offsetConfidence: Confidence(
          lanes.overallConfidence,
          source: 'lane observation',
        ),
        source: lanes.mode == LaneMode.bothBoundaries
            ? LateralSource.laneObservation
            : LateralSource.singleBoundary,
        laneWidthMeters: lanes.laneWidthMeters,
        laneIndexFromEdge: lane.$1,
        laneCount: lane.$2,
        laneIndexConfidence: lane.$3,
        headingAgreesWithRoad: headingAgrees,
      );
      return _state;
    }

    // --- No observation this frame: bridge on the IMU -------------------
    _secondsSinceObservation += dtSeconds;
    if (_secondsSinceObservation > maxBridgeSeconds ||
        !_state.isUsable) {
      _state = LateralState(
        offsetInLaneMeters: 0,
        offsetConfidence: Confidence.zero,
        source: LateralSource.none,
        secondsSinceObservation: _secondsSinceObservation,
        headingAgreesWithRoad: headingAgrees,
      );
      return _state;
    }

    // Heading error integrates yaw rate; lateral offset integrates the
    // lateral component of velocity.
    _headingErrorRadians += ego.yawRateRadPerS * dtSeconds;
    _bridgedOffset += ego.speedMps * math.sin(_headingErrorRadians) * dtSeconds;

    // Quadratic decay: the error in a double integration grows with the
    // square of the elapsed time, so the confidence must fall the same way.
    final double t = _secondsSinceObservation / maxBridgeSeconds;
    final double decay = clampDouble(1 - t * t, 0, 1);

    _state = LateralState(
      offsetInLaneMeters: _bridgedOffset,
      offsetConfidence: Confidence(
        _observedConfidence * decay,
        source: 'dead reckoning',
      ),
      source: LateralSource.deadReckoned,
      laneWidthMeters: _state.laneWidthMeters,
      // Lane membership is not carried across a dropout. Whether we changed
      // lane is exactly what we could not see.
      laneIndexFromEdge: null,
      laneCount: _state.laneCount,
      laneIndexConfidence: 0,
      secondsSinceObservation: _secondsSinceObservation,
      headingAgreesWithRoad: headingAgrees,
    );
    return _state;
  }

  /// Does the fused heading match the road the map matched us to?
  ///
  /// Null when there is no match to check against. This is a validity test
  /// on the map match, not a position: if we are pointing 60° away from the
  /// road we were snapped to, the snap is wrong and nothing derived from it
  /// — lane count, speed limit, upcoming manoeuvre — should be believed.
  bool? _headingAgreement(EgoMotionState ego, RouteProgress? route) {
    if (route == null || route.matchQuality < 0.2) return null;
    if (ego.headingConfidence < 0.3) return null;
    double delta = (ego.headingDegrees - route.roadBearingDegrees).abs() % 360;
    if (delta > 180) delta = 360 - delta;
    // A road can be driven in either direction, and the polyline has only
    // one; 180° apart is agreement, not disagreement.
    final double folded = math.min(delta, 180 - delta);
    return folded <= headingAgreementDegrees;
  }

  /// Try to work out which lane we are in from the boundary types.
  ///
  /// Returns `(index from the outer edge, lane count, confidence)`.
  ///
  /// This succeeds rarely and is meant to. The one genuinely reliable
  /// signature is a physical edge — a kerb or the end of the asphalt — on the
  /// outer side, which puts us in the outermost lane. Everything else is a
  /// guess, and a guess about lane membership is worse than an admission:
  /// the decisions built on it, such as which signal governs us or which
  /// exit we can take, are ones you would far rather see refused than
  /// answered wrongly.
  ///
  /// The lane **count** is not available at all in this build. OSM-derived
  /// routing gives geometry, road names and sometimes a speed limit, but not
  /// a per-direction lane count, and without it "lane 1 of ?" is not worth
  /// reporting. A lane-level map would plug in here and is the only thing
  /// that would make this question routinely answerable.
  (int?, int?, double) _inferLane(LaneDetectionResult lanes) {
    LaneBoundary? outer;
    for (final LaneBoundary b in lanes.boundaries) {
      if (!b.position.isEgoBoundary) continue;
      final bool isRight = b.position == LanePosition.egoRight;
      if (isRight == drivingSide.outerEdgeIsRight) outer = b;
    }

    if (outer != null &&
        (outer.lineType == LineType.curb ||
            outer.lineType == LineType.roadEdge) &&
        outer.confidence.value > 0.5) {
      return (0, null, clampDouble(outer.confidence.value, 0, 0.85));
    }

    return (null, null, 0);
  }
}
