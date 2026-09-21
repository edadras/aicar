import 'dart:math' as math;

import '../camera/camera_calibration.dart';
import '../core/confidence.dart';
import '../core/geometry.dart';
import '../depth/depth_map.dart';
import '../navigation/maneuver.dart';
import '../navigation/route.dart';
import '../perception/object_class.dart';
import '../perception/traffic_light.dart';
import '../perception/traffic_sign.dart';
import '../road/lane.dart';
import '../road/no_lane_corridor.dart';
import '../road/road_segmentation.dart';
import '../sensors/ego_motion.dart';
import '../tracking/object_track.dart';
import 'hazard.dart';

/// Everything the stack believes about the world at one instant.
///
/// This is the single integration point: every perception module writes into
/// it, and every decision module reads only from it. Planning, collision
/// prediction and the decision engine never touch a camera frame, a tensor or
/// a detector — which is what makes them deterministic, replayable and
/// testable without a device.
class WorldState {
  const WorldState({
    required this.frameId,
    required this.timestampMicros,
    required this.ego,
    required this.calibration,
    required this.lanes,
    required this.drivableArea,
    required this.roadEdges,
    required this.tracks,
    required this.trafficSigns,
    required this.trafficLights,
    required this.regulatory,
    required this.hazards,
    required this.autonomy,
    this.corridor,
    this.routeProgress,
    this.depth,
    this.segmentation,
    this.ambientLuminance = 128,
    this.degradedSubsystems = const <String>[],
  });

  /// An empty world, used before the first frame and on a hard reset.
  factory WorldState.initial() => WorldState(
        frameId: -1,
        timestampMicros: 0,
        ego: EgoMotionState.unknown,
        calibration: CameraCalibration.galaxyS23Default(),
        lanes: LaneDetectionResult.empty(frameId: -1, timestampMicros: 0),
        drivableArea:
            DrivableArea.empty(frameId: -1, timestampMicros: 0),
        roadEdges: const <RoadEdge>[],
        tracks: const <ObjectTrack>[],
        trafficSigns: const <TrafficSign>[],
        trafficLights: const <TrafficLight>[],
        regulatory: const RegulatoryContext(),
        hazards: const <Hazard>[],
        autonomy: AutonomyConfidence.unknown,
        degradedSubsystems: const <String>['not started'],
      );

  final int frameId;
  final int timestampMicros;

  // --- Ego ----------------------------------------------------------------

  final EgoMotionState ego;
  final CameraCalibration calibration;

  // --- Road ---------------------------------------------------------------

  final LaneDetectionResult lanes;
  final DrivableArea drivableArea;
  final List<RoadEdge> roadEdges;

  /// The NO_LANE_MODE corridor, present only when lanes were not usable *and*
  /// there was enough other evidence to justify one.
  final CorridorEstimate? corridor;

  final RoadSegmentation? segmentation;
  final DepthMap? depth;

  // --- Dynamic objects ----------------------------------------------------

  final List<ObjectTrack> tracks;

  // --- Infrastructure -----------------------------------------------------

  final List<TrafficSign> trafficSigns;
  final List<TrafficLight> trafficLights;
  final RegulatoryContext regulatory;

  // --- Route --------------------------------------------------------------

  final RouteProgress? routeProgress;

  // --- Assessment ---------------------------------------------------------

  final List<Hazard> hazards;
  final AutonomyConfidence autonomy;

  /// Mean scene luminance, used to detect night/tunnel conditions.
  final double ambientLuminance;

  /// Names of subsystems that reported a degraded result this frame.
  final List<String> degradedSubsystems;

  double get timestampSeconds => timestampMicros / 1e6;

  // --- Convenience views --------------------------------------------------

  /// Vulnerable road users, which several decision branches treat specially.
  List<ObjectTrack> get pedestrians => tracks
      .where((ObjectTrack t) => t.objectClass == ObjectClass.person)
      .toList();

  List<ObjectTrack> get motorcycles => tracks
      .where((ObjectTrack t) => t.objectClass == ObjectClass.motorcycle)
      .toList();

  List<ObjectTrack> get cyclists => tracks
      .where((ObjectTrack t) => t.objectClass == ObjectClass.bicycle)
      .toList();

  List<ObjectTrack> get vehicles =>
      tracks.where((ObjectTrack t) => t.objectClass.isVehicle).toList();

  List<ObjectTrack> get vulnerableRoadUsers =>
      tracks.where((ObjectTrack t) => t.objectClass.isVulnerable).toList();

  /// Objects in our own lane or crossing it, ordered nearest first.
  List<ObjectTrack> get objectsInPath {
    final List<ObjectTrack> inPath = tracks
        .where((ObjectTrack t) =>
            t.laneRelation.blocksEgoPath && t.position.y > 0)
        .toList()
      ..sort((ObjectTrack a, ObjectTrack b) =>
          a.position.y.compareTo(b.position.y));
    return inPath;
  }

  /// The vehicle we are following, if any.
  ObjectTrack? get leadVehicle {
    ObjectTrack? lead;
    for (final ObjectTrack t in tracks) {
      if (!t.objectClass.isVehicle) continue;
      if (t.laneRelation != LaneRelation.egoLane) continue;
      if (t.position.y <= 0) continue;
      if (lead == null || t.position.y < lead.position.y) lead = t;
    }
    return lead;
  }

  /// The most severe hazard, which is what the HUD banner shows.
  Hazard? get primaryHazard {
    Hazard? worst;
    for (final Hazard h in hazards) {
      if (worst == null || h.severity > worst.severity) worst = h;
    }
    return worst;
  }

  HazardSeverity get worstHazardSeverity =>
      primaryHazard?.severity ?? HazardSeverity.info;

  /// Signal that currently governs us, if one has been confidently resolved.
  TrafficLight? get governingTrafficLight {
    TrafficLight? best;
    for (final TrafficLight l in trafficLights) {
      if (!l.isActionable) continue;
      if (best == null ||
          (l.distanceMeters ?? 1e9) < (best.distanceMeters ?? 1e9)) {
        best = l;
      }
    }
    return best;
  }

  /// Posted limit currently in force, from signs, falling back to the map.
  int? get effectiveSpeedLimitKph =>
      regulatory.speedLimitKph ?? routeProgress?.mapSpeedLimitKph;

  ManeuverIntent get navigationIntent =>
      routeProgress?.intent ?? ManeuverIntent.unknown;

  bool get isNight => ambientLuminance < 55;

  /// The lane/corridor model in use, as a single centreline. Prefers detected
  /// lanes, then the NO_LANE_MODE corridor, then nothing — and "nothing" is a
  /// legitimate, meaningful answer.
  Polynomial? get referenceCenterline {
    if (lanes.overallConfidence >= ConfidenceThresholds.laneUsable) {
      final Polynomial? centre = lanes.centerline;
      if (centre != null) return centre;
    }
    return corridor?.centerline;
  }

  /// How far ahead the road model is actually supported by evidence.
  double get roadModelRangeMeters {
    final double laneRange = lanes.usableRangeMeters;
    final double corridorRange = corridor?.maxRangeMeters ?? 0;
    final double areaRange = drivableArea.maxRangeMeters;
    return math.max(laneRange, math.max(corridorRange, areaRange));
  }

  WorldState copyWith({
    List<ObjectTrack>? tracks,
    List<Hazard>? hazards,
    AutonomyConfidence? autonomy,
    RouteProgress? routeProgress,
    CorridorEstimate? corridor,
    List<String>? degradedSubsystems,
  }) =>
      WorldState(
        frameId: frameId,
        timestampMicros: timestampMicros,
        ego: ego,
        calibration: calibration,
        lanes: lanes,
        drivableArea: drivableArea,
        roadEdges: roadEdges,
        tracks: tracks ?? this.tracks,
        trafficSigns: trafficSigns,
        trafficLights: trafficLights,
        regulatory: regulatory,
        hazards: hazards ?? this.hazards,
        autonomy: autonomy ?? this.autonomy,
        corridor: corridor ?? this.corridor,
        routeProgress: routeProgress ?? this.routeProgress,
        depth: depth,
        segmentation: segmentation,
        ambientLuminance: ambientLuminance,
        degradedSubsystems: degradedSubsystems ?? this.degradedSubsystems,
      );

  Map<String, dynamic> toJson() => <String, dynamic>{
        'frameId': frameId,
        'ts': timestampMicros,
        'ego': ego.toJson(),
        'lanes': lanes.toJson(),
        'drivableArea': drivableArea.toJson(),
        'roadEdges': <Map<String, dynamic>>[
          for (final RoadEdge e in roadEdges) e.toJson(),
        ],
        if (corridor != null) 'corridor': corridor!.toJson(),
        'tracks': <Map<String, dynamic>>[
          for (final ObjectTrack t in tracks) t.toJson(),
        ],
        'signs': <Map<String, dynamic>>[
          for (final TrafficSign s in trafficSigns) s.toJson(),
        ],
        'lights': <Map<String, dynamic>>[
          for (final TrafficLight l in trafficLights) l.toJson(),
        ],
        'regulatory': regulatory.toJson(),
        'hazards': <Map<String, dynamic>>[
          for (final Hazard h in hazards) h.toJson(),
        ],
        'autonomy': autonomy.toJson(),
        if (routeProgress != null) 'route': routeProgress!.toJson(),
        if (depth != null) 'depth': depth!.toJson(),
        if (segmentation != null) 'segmentation': segmentation!.toJson(),
        'ambientLuminance': ambientLuminance.round(),
        if (degradedSubsystems.isNotEmpty) 'degraded': degradedSubsystems,
      };

  /// Reconstruct a world model from a recording, for "recorded results"
  /// replay.
  ///
  /// Only the parts that were serialised come back: the dense depth map and
  /// segmentation grid are deliberately *not* written to the stream (they
  /// would dominate its size and are recoverable by re-running the models
  /// over the recorded frames), so a replayed world has them as null. Every
  /// number the HUD showed at the time is present.
  static WorldState fromRecordedJson(
    Map<String, dynamic> j, {
    required CameraCalibration calibration,
  }) {
    final int frameId = (j['frameId'] as num?)?.toInt() ?? 0;
    final int ts = (j['ts'] as num?)?.toInt() ?? 0;

    return WorldState(
      frameId: frameId,
      timestampMicros: ts,
      ego: j['ego'] == null
          ? EgoMotionState.unknown
          : EgoMotionState.fromJson(j['ego'] as Map<String, dynamic>),
      calibration: calibration,
      lanes: j['lanes'] == null
          ? LaneDetectionResult.empty(frameId: frameId, timestampMicros: ts)
          : LaneDetectionResult.fromJson(
              j['lanes'] as Map<String, dynamic>,
              frameId: frameId,
              timestampMicros: ts,
            ),
      drivableArea: j['drivableArea'] == null
          ? DrivableArea.empty(frameId: frameId, timestampMicros: ts)
          : DrivableArea.fromJson(
              j['drivableArea'] as Map<String, dynamic>,
              frameId: frameId,
              timestampMicros: ts,
            ),
      roadEdges: <RoadEdge>[
        for (final dynamic e
            in (j['roadEdges'] as List<dynamic>? ?? const <dynamic>[]))
          RoadEdge.fromJson(e as Map<String, dynamic>),
      ],
      tracks: <ObjectTrack>[
        for (final dynamic t
            in (j['tracks'] as List<dynamic>? ?? const <dynamic>[]))
          ObjectTrack.fromJson(t as Map<String, dynamic>, frameId: frameId),
      ],
      trafficSigns: <TrafficSign>[
        for (final dynamic s
            in (j['signs'] as List<dynamic>? ?? const <dynamic>[]))
          TrafficSign.fromJson(
            s as Map<String, dynamic>,
            frameId: frameId,
            timestampMicros: ts,
          ),
      ],
      trafficLights: <TrafficLight>[
        for (final dynamic l
            in (j['lights'] as List<dynamic>? ?? const <dynamic>[]))
          TrafficLight.fromJson(
            l as Map<String, dynamic>,
            frameId: frameId,
            timestampMicros: ts,
          ),
      ],
      regulatory: _regulatoryFromJson(
          j['regulatory'] as Map<String, dynamic>?),
      hazards: <Hazard>[
        for (final dynamic h
            in (j['hazards'] as List<dynamic>? ?? const <dynamic>[]))
          Hazard.fromJson(h as Map<String, dynamic>),
      ],
      autonomy: _autonomyFromJson(j['autonomy'] as Map<String, dynamic>?),
      ambientLuminance:
          (j['ambientLuminance'] as num?)?.toDouble() ?? 128,
      degradedSubsystems: <String>[
        for (final dynamic d
            in (j['degraded'] as List<dynamic>? ?? const <dynamic>[]))
          '$d',
      ],
    );
  }

  static RegulatoryContext _regulatoryFromJson(Map<String, dynamic>? j) {
    if (j == null) return const RegulatoryContext();
    return RegulatoryContext(
      speedLimitKph: (j['speedLimit'] as num?)?.toInt(),
      speedLimitConfidence: (j['speedLimitConf'] as num?)?.toDouble() ?? 0,
      overtakingProhibited: j['noOvertaking'] as bool? ?? false,
      inSchoolZone: j['schoolZone'] as bool? ?? false,
      inRoadWorks: j['roadWorks'] as bool? ?? false,
      pendingStop: j['pendingStop'] as bool? ?? false,
      pendingGiveWay: j['pendingGiveWay'] as bool? ?? false,
    );
  }

  static AutonomyConfidence _autonomyFromJson(Map<String, dynamic>? j) {
    if (j == null) return AutonomyConfidence.unknown;
    return AutonomyConfidence(
      perception: (j['perception'] as num?)?.toDouble() ?? 0,
      lanes: (j['lanes'] as num?)?.toDouble() ?? 0,
      depth: (j['depth'] as num?)?.toDouble() ?? 0,
      egoMotion: (j['egoMotion'] as num?)?.toDouble() ?? 0,
      planning: (j['planning'] as num?)?.toDouble() ?? 0,
      overall: (j['overall'] as num?)?.toDouble() ?? 0,
      weakestSubsystem: j['weakest'] as String? ?? 'unknown',
    );
  }

  @override
  String toString() => 'WorldState(#$frameId, ${tracks.length} tracks, '
      '${hazards.length} hazards, ${lanes.mode.badge}, $autonomy)';
}
