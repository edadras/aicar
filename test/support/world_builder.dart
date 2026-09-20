import 'package:aicar/camera/camera_calibration.dart';
import 'package:aicar/core/confidence.dart';
import 'package:aicar/core/geometry.dart';
import 'package:aicar/perception/object_class.dart';
import 'package:aicar/perception/traffic_light.dart';
import 'package:aicar/perception/traffic_sign.dart';
import 'package:aicar/road/lane.dart';
import 'package:aicar/road/road_segmentation.dart';
import 'package:aicar/sensors/ego_motion.dart';
import 'package:aicar/tracking/object_track.dart';
import 'package:aicar/world_model/hazard.dart';
import 'package:aicar/world_model/world_state.dart';

const CameraCalibration testCalibration = CameraCalibration(
  imageWidth: 640,
  imageHeight: 360,
  horizontalFovDegrees: 73.7,
  cameraHeightMeters: 1.2,
  pitchDegrees: 2.5,
  rollDegrees: 0,
  yawDegrees: 0,
  lateralOffsetMeters: 0,
  longitudinalOffsetMeters: 2.0,
  isCalibrated: true,
);

EgoMotionState testEgo({
  double speedMps = 13.9,
  double yawRate = 0,
  int ts = 0,
  double speedConfidence = 0.9,
}) =>
    EgoMotionState(
      speedMps: speedMps,
      headingDegrees: 0,
      yawRateRadPerS: yawRate,
      longitudinalAccelMps2: 0,
      lateralAccelMps2: 0,
      position: null,
      timestampMicros: ts,
      speedConfidence: speedConfidence,
      headingConfidence: 0.9,
      isStationary: speedMps < 0.35,
    );

/// A straight, confidently detected 3.5 m lane centred on the vehicle.
LaneDetectionResult straightLanes({
  double width = 3.5,
  double offset = 0,
  double curvature = 0,
  double confidence = 0.85,
  double range = 40,
  int frameId = 1,
  int ts = 0,
}) {
  LaneBoundary side(LanePosition position, double lateral) => LaneBoundary(
        position: position,
        curve: Polynomial(<double>[lateral + offset, 0, curvature]),
        confidence: Confidence(confidence),
        lineType: LineType.solid,
        color: LineColor.white,
        minRangeMeters: 4,
        maxRangeMeters: range,
        supportPointCount: 20,
      );

  return LaneDetectionResult(
    boundaries: <LaneBoundary>[
      side(LanePosition.egoLeft, -width / 2),
      side(LanePosition.egoRight, width / 2),
    ],
    mode: LaneMode.bothBoundaries,
    frameId: frameId,
    timestampMicros: ts,
    laneWidthMeters: width,
    laneWidthConfidence: 0.8,
    egoLateralOffsetMeters: -offset,
    egoHeadingErrorRadians: 0,
    modelName: 'test',
  );
}

/// A uniform drivable corridor of [halfWidth] metres either side.
DrivableArea straightCorridor({
  double halfWidth = 4.0,
  double range = 50,
  double confidence = 0.8,
  int frameId = 1,
  int ts = 0,
}) =>
    DrivableArea(
      samples: <DrivableSample>[
        for (double d = 3; d <= range; d += 2)
          DrivableSample(
            distanceAhead: d,
            leftEdge: -halfWidth,
            rightEdge: halfWidth,
            confidence: confidence,
          ),
      ],
      confidence: confidence,
      frameId: frameId,
      timestampMicros: ts,
      source: 'test',
    );

ObjectTrack testTrack({
  required int id,
  required ObjectClass objectClass,
  required Vec2 position,
  Vec2 worldVelocity = const Vec2.zero(),
  Vec2? relativeVelocity,
  double egoSpeed = 13.9,
  MotionDirection direction = MotionDirection.parallel,
  LaneRelation laneRelation = LaneRelation.egoLane,
  CollisionRisk risk = CollisionRisk.low,
  double? ttc,
  double confidence = 0.9,
  double distanceConfidence = 0.8,
  int age = 10,
  bool confirmed = true,
}) {
  final Vec2 relative = relativeVelocity ??
      Vec2(worldVelocity.x, worldVelocity.y - egoSpeed);
  return ObjectTrack(
    id: id,
    objectClass: objectClass,
    confidence: Confidence(confidence),
    box: BoundingBox(
      left: 0.4,
      top: 0.4,
      width: 0.1,
      height: 0.12,
    ),
    estimatedDistanceMeters: position.length,
    distanceConfidence: Confidence(distanceConfidence),
    position: position,
    velocityWorld: worldVelocity,
    relativeVelocity: relative,
    velocityConfidence: 0.8,
    direction: direction,
    laneRelation: laneRelation,
    laneRelationConfidence: 0.8,
    collisionRisk: risk,
    timeToCollisionSeconds: ttc,
    firstSeenMicros: 0,
    lastSeenMicros: 500000,
    frameId: 10,
    age: age,
    isConfirmed: confirmed,
  );
}

WorldState testWorld({
  int frameId = 1,
  int timestampMicros = 0,
  EgoMotionState? ego,
  LaneDetectionResult? lanes,
  DrivableArea? drivableArea,
  List<ObjectTrack> tracks = const <ObjectTrack>[],
  List<TrafficSign> signs = const <TrafficSign>[],
  List<TrafficLight> lights = const <TrafficLight>[],
  RegulatoryContext regulatory = const RegulatoryContext(),
  List<Hazard> hazards = const <Hazard>[],
  AutonomyConfidence? autonomy,
  List<String> degraded = const <String>[],
  double ambientLuminance = 140,
}) =>
    WorldState(
      frameId: frameId,
      timestampMicros: timestampMicros,
      ego: ego ?? testEgo(ts: timestampMicros),
      calibration: testCalibration,
      lanes: lanes ?? straightLanes(frameId: frameId, ts: timestampMicros),
      drivableArea: drivableArea ??
          straightCorridor(frameId: frameId, ts: timestampMicros),
      roadEdges: const <RoadEdge>[],
      tracks: tracks,
      trafficSigns: signs,
      trafficLights: lights,
      regulatory: regulatory,
      hazards: hazards,
      autonomy: autonomy ??
          AutonomyConfidence.compute(
            perception: 0.85,
            lanes: 0.85,
            depth: 0.7,
            egoMotion: 0.9,
            planning: 0.8,
          ),
      ambientLuminance: ambientLuminance,
      degradedSubsystems: degraded,
    );
