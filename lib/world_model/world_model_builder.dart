import 'dart:math' as math;

import '../camera/camera_calibration.dart';
import '../core/confidence.dart';
import '../core/geometry.dart';
import '../depth/depth_map.dart';
import '../navigation/route.dart';
import '../perception/detection.dart';
import '../perception/traffic_light.dart';
import '../perception/traffic_sign.dart';
import '../road/intersection_detector.dart';
import '../road/lane.dart';
import '../road/road_marking.dart';
import '../road/no_lane_corridor.dart';
import '../road/road_segmentation.dart';
import '../sensors/ego_motion.dart';
import '../tracking/object_track.dart';
import 'hazard.dart';
import 'world_state.dart';

/// Assembles the per-frame outputs of every perception stage into one
/// [WorldState], and derives the two things that are properties of the *whole*
/// scene rather than of any one stage: the hazard list and the autonomy
/// confidence.
///
/// Keeping this separate from the pipeline means the fusion rules can be
/// tested against hand-built inputs, with no camera, no models and no isolate.
class WorldModelBuilder {
  const WorldModelBuilder({
    this.laneDepartureThresholdMeters = 0.45,
    this.nightLuminanceThreshold = 55,
  });

  /// Lateral offset from the lane centre beyond which a lane-departure
  /// warning is raised.
  final double laneDepartureThresholdMeters;

  final double nightLuminanceThreshold;

  WorldState build({
    required int frameId,
    required int timestampMicros,
    required EgoMotionState ego,
    required CameraCalibration calibration,
    required LaneDetectionResult lanes,
    required DrivableArea drivableArea,
    required List<RoadEdge> roadEdges,
    required List<ObjectTrack> tracks,
    required List<TrafficSign> signs,
    required List<TrafficLight> lights,
    required RegulatoryContext regulatory,
    List<RoadMarking> roadMarkings = const <RoadMarking>[],
    IntersectionEstimate? intersection,
    CorridorEstimate? corridor,
    RouteProgress? routeProgress,
    DepthMap? depth,
    RoadSegmentation? segmentation,
    DetectionResult? detections,
    double ambientLuminance = 128,
    double planningConfidence = 0,
    List<String> degradedSubsystems = const <String>[],
  }) {
    final AutonomyConfidence autonomy = _computeAutonomy(
      detections: detections,
      lanes: lanes,
      corridor: corridor,
      depth: depth,
      segmentation: segmentation,
      ego: ego,
      planningConfidence: planningConfidence,
      ambientLuminance: ambientLuminance,
      calibration: calibration,
    );

    final List<Hazard> hazards = _collectHazards(
      tracks: tracks,
      lanes: lanes,
      lights: lights,
      regulatory: regulatory,
      roadMarkings: roadMarkings,
      intersection: intersection,
      ego: ego,
      drivableArea: drivableArea,
      autonomy: autonomy,
      ambientLuminance: ambientLuminance,
      degradedSubsystems: degradedSubsystems,
    );

    return WorldState(
      frameId: frameId,
      timestampMicros: timestampMicros,
      ego: ego,
      calibration: calibration,
      lanes: lanes,
      drivableArea: drivableArea,
      roadEdges: roadEdges,
      corridor: corridor,
      tracks: tracks,
      trafficSigns: signs,
      trafficLights: lights,
      regulatory: regulatory,
      roadMarkings: roadMarkings,
      intersection: intersection,
      routeProgress: routeProgress,
      depth: depth,
      segmentation: segmentation,
      hazards: hazards,
      autonomy: autonomy,
      ambientLuminance: ambientLuminance,
      degradedSubsystems: degradedSubsystems,
    );
  }

  // --- Autonomy confidence ------------------------------------------------

  AutonomyConfidence _computeAutonomy({
    required DetectionResult? detections,
    required LaneDetectionResult lanes,
    required CorridorEstimate? corridor,
    required DepthMap? depth,
    required RoadSegmentation? segmentation,
    required EgoMotionState ego,
    required double planningConfidence,
    required double ambientLuminance,
    required CameraCalibration calibration,
  }) {
    // Perception: how much we trust what we can see.
    //
    // Object detection and surface segmentation are different capabilities
    // and are not interchangeable. Knowing where the road is does not
    // compensate for not knowing what is on it — the dangerous unknowns are
    // the objects. So a degraded detector pins perception at zero rather
    // than being averaged upwards by a healthy segmenter, and otherwise the
    // detector dominates the blend.
    final bool detectorDegraded = detections == null || detections.isDegraded;
    double perception;
    if (detectorDegraded) {
      perception = 0;
    } else {
      final double detectionScore = detections.frameConfidence;
      final double segmentationScore =
          segmentation != null && segmentation.isUsable
              ? segmentation.overallConfidence
              : detectionScore;
      perception = 0.75 * detectionScore + 0.25 * segmentationScore;
    }

    // Road model: lanes if usable, otherwise the corridor, which is inherently
    // weaker and is scored as such.
    final double laneScore = math.max(
      lanes.overallConfidence,
      (corridor?.confidence ?? 0) * 0.8,
    );

    // Depth: a fitted map plus the geometric fallback. Even with no depth
    // model, ground-plane geometry gives a usable-but-limited distance, so
    // this floors above zero — but only when the camera is calibrated.
    double depthScore = depth?.globalConfidence ?? 0;
    final double geometricFloor = calibration.isCalibrated ? 0.45 : 0.28;
    depthScore = math.max(depthScore, geometricFloor);

    final double egoScore = ego.overallConfidence;

    // Night and low light degrade everything the camera does; applying it here
    // rather than inside each stage keeps the reason visible in one place.
    double lightingFactor = 1.0;
    if (ambientLuminance < nightLuminanceThreshold) {
      lightingFactor = clampDouble(
        0.55 + 0.45 * (ambientLuminance / nightLuminanceThreshold),
        0.5,
        1.0,
      );
    } else if (ambientLuminance > 235) {
      // Blown-out highlights (low sun, tunnel exit) are just as bad.
      lightingFactor = 0.7;
    }

    return AutonomyConfidence.compute(
      perception: perception * lightingFactor,
      lanes: laneScore * lightingFactor,
      depth: depthScore,
      egoMotion: egoScore,
      planning: planningConfidence,
    );
  }

  // --- Hazards ------------------------------------------------------------

  List<Hazard> _collectHazards({
    required List<ObjectTrack> tracks,
    required LaneDetectionResult lanes,
    required List<TrafficLight> lights,
    required RegulatoryContext regulatory,
    required List<RoadMarking> roadMarkings,
    required IntersectionEstimate? intersection,
    required EgoMotionState ego,
    required DrivableArea drivableArea,
    required AutonomyConfidence autonomy,
    required double ambientLuminance,
    required List<String> degradedSubsystems,
  }) {
    final List<Hazard> hazards = <Hazard>[];

    for (final ObjectTrack t in tracks) {
      final Hazard? h = Hazard.fromTrack(t);
      if (h != null) hazards.add(h);

      // Motorcycles get their own warning even below the generic risk
      // threshold: their lateral dynamics make them the case most likely to
      // surprise both the system and the driver.
      if (t.isMotorcycleCrossing &&
          (h == null || h.type != HazardType.motorcycleCrossing)) {
        hazards.add(Hazard(
          type: HazardType.motorcycleCrossing,
          severity: HazardSeverity.warning,
          description: '${t.displayLabel} crossing at '
              '${t.lateralSpeedMps.abs().toStringAsFixed(1)} m/s laterally, '
              '${t.estimatedDistanceMeters.toStringAsFixed(1)} m ahead',
          confidence: t.confidence.value,
          relatedTrackId: t.id,
          position: t.position,
          distanceMeters: t.estimatedDistanceMeters,
          timeToCollisionSeconds: t.timeToCollisionSeconds,
        ));
      }
    }

    // Lane departure: only meaningful when the lane model is trustworthy and
    // we are actually moving.
    if (lanes.mode == LaneMode.bothBoundaries &&
        lanes.overallConfidence > 0.55 &&
        ego.speedMps > 4 &&
        lanes.egoLateralOffsetMeters.abs() > laneDepartureThresholdMeters) {
      hazards.add(Hazard(
        type: HazardType.laneDeparture,
        severity: lanes.egoLateralOffsetMeters.abs() > 0.9
            ? HazardSeverity.warning
            : HazardSeverity.caution,
        description: 'Drifting '
            '${lanes.egoLateralOffsetMeters.abs().toStringAsFixed(2)} m '
            '${lanes.egoLateralOffsetMeters > 0 ? 'right' : 'left'} of the '
            'lane centre',
        confidence: lanes.overallConfidence,
      ));
    }

    final TrafficLight? red = lights
        .cast<TrafficLight?>()
        .firstWhere(
          (TrafficLight? l) => l!.isActionable && l.color.requiresStop,
          orElse: () => null,
        );
    if (red != null) {
      hazards.add(Hazard(
        type: HazardType.redLight,
        severity: HazardSeverity.warning,
        description: 'Red light '
            '${red.distanceMeters == null ? 'ahead' : 'at '
                '${red.distanceMeters!.toStringAsFixed(0)} m'}',
        confidence: red.colorConfidence.value * red.relevanceConfidence.value,
        distanceMeters: red.distanceMeters,
      ));
    }

    if (regulatory.pendingStop) {
      hazards.add(const Hazard(
        type: HazardType.stopSign,
        severity: HazardSeverity.warning,
        description: 'Stop sign ahead',
        confidence: 0.8,
      ));
    }

    // Paint on the road. These are announcements, not emergencies, so they
    // stay at caution unless someone is standing on the crossing.
    for (final RoadMarking m in roadMarkings) {
      if (m.farEdgeMeters < 0) continue;
      if (m.distanceMeters > m.type.approachMeters) continue;

      switch (m.type) {
        case RoadMarkingType.crosswalk:
          final bool occupied = tracks.any((ObjectTrack t) =>
              t.objectClass.isVulnerable &&
              t.position.y > m.distanceMeters - 4 &&
              t.position.y < m.farEdgeMeters + 4 &&
              t.position.x.abs() < m.widthMeters / 2 + 2.5);
          hazards.add(Hazard(
            type: HazardType.crosswalkAhead,
            severity:
                occupied ? HazardSeverity.warning : HazardSeverity.caution,
            description: occupied
                ? 'Someone is at the crossing '
                    '${m.distanceMeters.toStringAsFixed(0)} m ahead'
                : 'Pedestrian crossing '
                    '${m.distanceMeters.toStringAsFixed(0)} m ahead',
            confidence: m.confidence.value,
            distanceMeters: m.distanceMeters,
          ));
        case RoadMarkingType.speedBump:
          hazards.add(Hazard(
            type: HazardType.speedBumpAhead,
            severity: HazardSeverity.caution,
            description: 'Speed bump '
                '${m.distanceMeters.toStringAsFixed(0)} m ahead'
                '${m.confirmedByMotion ? ' (confirmed)' : ''}',
            confidence: m.confidence.value,
            distanceMeters: m.distanceMeters,
          ));
        case RoadMarkingType.stopLine:
          // On its own a stop line is evidence of a junction, which the
          // junction hazard below already reports. Announcing both would
          // say the same thing twice.
          break;
      }
    }

    if (intersection != null) {
      final bool crossing = intersection.hasCrossingTraffic;
      hazards.add(Hazard(
        type: crossing
            ? HazardType.crossingTraffic
            : HazardType.intersectionAhead,
        severity: crossing
            ? HazardSeverity.warning
            : (intersection.control.requiresYield
                ? HazardSeverity.caution
                : HazardSeverity.info),
        description: '${intersection.control.label} junction '
            '${intersection.distanceMeters.toStringAsFixed(0)} m ahead: '
            '${intersection.evidence.join('; ')}',
        confidence: intersection.confidence.value,
        distanceMeters: intersection.distanceMeters,
      ));
    }

    final int? limit = regulatory.speedLimitKph;
    if (limit != null &&
        regulatory.hasSpeedLimit &&
        ego.speedKph > limit + 5 &&
        ego.speedConfidence > 0.5) {
      hazards.add(Hazard(
        type: HazardType.speedLimitExceeded,
        severity: ego.speedKph > limit + 20
            ? HazardSeverity.warning
            : HazardSeverity.caution,
        description: '${ego.speedKph.toStringAsFixed(0)} km/h in a '
            '$limit km/h limit',
        confidence: regulatory.speedLimitConfidence,
      ));
    }

    if (drivableArea.maxRangeMeters > 0 &&
        drivableArea.maxRangeMeters < 12 &&
        ego.speedMps > 5) {
      hazards.add(Hazard(
        type: HazardType.roadEnds,
        severity: HazardSeverity.warning,
        description: 'Drivable area ends '
            '${drivableArea.maxRangeMeters.toStringAsFixed(0)} m ahead',
        confidence: drivableArea.confidence,
        distanceMeters: drivableArea.maxRangeMeters,
      ));
    }

    if (ambientLuminance < nightLuminanceThreshold * 0.6) {
      hazards.add(Hazard(
        type: HazardType.lowVisibility,
        severity: HazardSeverity.caution,
        description: 'Low light: perception range and confidence reduced',
        confidence: 0.8,
      ));
    }

    if (autonomy.isLow) {
      hazards.add(Hazard(
        type: HazardType.lowAutonomyConfidence,
        severity: HazardSeverity.warning,
        description: 'Overall confidence '
            '${(autonomy.overall * 100).round()}% '
            '(weakest: ${autonomy.weakestSubsystem})',
        confidence: 0.9,
      ));
    }

    if (degradedSubsystems.isNotEmpty) {
      hazards.add(Hazard(
        type: HazardType.perceptionDegraded,
        severity: HazardSeverity.caution,
        description: degradedSubsystems.join(', '),
        confidence: 0.85,
      ));
    }

    // Most severe first so the HUD banner and the decision engine agree on
    // what "the" hazard is.
    hazards.sort((Hazard a, Hazard b) {
      final int bySeverity = b.severity.level.compareTo(a.severity.level);
      if (bySeverity != 0) return bySeverity;
      final double ad = a.distanceMeters ?? double.infinity;
      final double bd = b.distanceMeters ?? double.infinity;
      return ad.compareTo(bd);
    });

    return hazards;
  }
}
