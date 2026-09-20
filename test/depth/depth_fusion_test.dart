import 'dart:typed_data';

import 'package:aicar/camera/camera_calibration.dart';
import 'package:aicar/core/confidence.dart';
import 'package:aicar/core/geometry.dart';
import 'package:aicar/core/ring_buffer.dart';
import 'package:aicar/depth/depth_fusion.dart';
import 'package:aicar/depth/depth_map.dart';
import 'package:aicar/perception/object_class.dart';
import 'package:aicar/sensors/ego_motion.dart';
import 'package:aicar/tracking/object_track.dart';
import 'package:flutter_test/flutter_test.dart';

const CameraCalibration calibration = CameraCalibration(
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

/// Build a track whose bounding box is exactly what the camera would see for
/// an object of [objectClass] standing [range] metres ahead.
ObjectTrack trackAt({
  required ObjectClass objectClass,
  required double range,
  int age = 8,
  int id = 1,
  MotionDirection direction = MotionDirection.approaching,
  List<BoundingBox> historyBoxes = const <BoundingBox>[],
}) {
  final PixelPoint contact =
      calibration.projectGroundToImage(Vec2(0, range))!;
  final PhysicalSizePrior prior = objectClass.sizePrior;
  final double heightPx = calibration.fy * prior.height / range;
  final double widthPx = calibration.fx * prior.width / range;

  final BoundingBox box = BoundingBox.fromPixels(
    left: contact.u - widthPx / 2,
    top: contact.v - heightPx,
    right: contact.u + widthPx / 2,
    bottom: contact.v,
    imageWidth: calibration.imageWidth,
    imageHeight: calibration.imageHeight,
  );

  final RingBuffer<TrackObservation> history =
      RingBuffer<TrackObservation>(16);
  for (int i = 0; i < historyBoxes.length; i++) {
    history.add(TrackObservation(
      timestampMicros: i * 50000,
      box: historyBoxes[i],
      position: Vec2(0, range),
      distanceMeters: range,
      score: 0.9,
    ));
  }

  return ObjectTrack(
    id: id,
    objectClass: objectClass,
    confidence: Confidence(0.9),
    box: box,
    estimatedDistanceMeters: range,
    distanceConfidence: Confidence(0.7),
    position: Vec2(0, range),
    velocityWorld: const Vec2.zero(),
    relativeVelocity: const Vec2.zero(),
    direction: direction,
    laneRelation: LaneRelation.egoLane,
    collisionRisk: CollisionRisk.low,
    timeToCollisionSeconds: null,
    firstSeenMicros: 0,
    lastSeenMicros: 400000,
    frameId: 8,
    age: age,
    isConfirmed: true,
    history: history,
  );
}

EgoMotionState ego({double speed = 0, double speedConfidence = 0.9}) =>
    EgoMotionState(
      speedMps: speed,
      headingDegrees: 0,
      yawRateRadPerS: 0,
      longitudinalAccelMps2: 0,
      lateralAccelMps2: 0,
      position: null,
      timestampMicros: 400000,
      speedConfidence: speedConfidence,
      headingConfidence: 0.9,
    );

/// A synthetic inverse-depth map for a flat road, as a MiDaS-style model
/// would produce: value proportional to 1/distance with an arbitrary scale
/// and offset that the fitter has to recover.
DepthMap syntheticRelativeDepth({
  double gain = 40.0,
  double offset = 0.05,
  int width = 128,
  int height = 72,
  Map<BoundingBox, double>? objects,
}) {
  final Float32List values = Float32List(width * height);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      final double u = (x + 0.5) / width * calibration.imageWidth;
      final double v = (y + 0.5) / height * calibration.imageHeight;
      final Vec2? ground = calibration.projectToGround(PixelPoint(u, v));
      double distance = ground == null ? 200 : ground.length;

      if (objects != null) {
        for (final MapEntry<BoundingBox, double> e in objects.entries) {
          final double nx = (x + 0.5) / width;
          final double ny = (y + 0.5) / height;
          if (nx >= e.key.left &&
              nx <= e.key.right &&
              ny >= e.key.top &&
              ny <= e.key.bottom) {
            distance = e.value;
          }
        }
      }
      values[y * width + x] = gain / distance + offset;
    }
  }

  return DepthMap(
    width: width,
    height: height,
    values: values,
    scale: DepthScale.relativeInverse,
    frameId: 1,
    timestampMicros: 400000,
    globalConfidence: 0,
  );
}

void main() {
  const DepthFusion fusion = DepthFusion();

  group('relative depth scale recovery', () {
    test('an unfitted relative map refuses to report metres', () {
      final DepthMap raw = syntheticRelativeDepth();
      expect(raw.isFitted, isFalse);
      expect(raw.distanceAt(0.5, 0.8), isNull);
      expect(raw.confidenceAt(0.5, 0.8), 0);
    });

    test('fitting against the ground plane recovers the true scale', () {
      final DepthMap raw = syntheticRelativeDepth(gain: 40, offset: 0.05);
      final DepthMap fitted = fusion.fitToGroundPlane(
        depth: raw,
        calibration: calibration,
      );

      expect(fitted.isFitted, isTrue);
      expect(fitted.globalConfidence, greaterThan(0.6));

      // A road point at a known range must now read back correctly.
      for (final double range in <double>[10, 20, 30, 40]) {
        final PixelPoint p =
            calibration.projectGroundToImage(Vec2(0, range))!;
        final double? d = fitted.distanceAt(
          p.u / calibration.imageWidth,
          p.v / calibration.imageHeight,
        );
        expect(d, isNotNull, reason: 'range $range');
        expect(d!, closeTo(range, range * 0.08), reason: 'range $range');
      }
    });

    test('a fit with too few anchors leaves the map unfitted', () {
      final DepthMap raw = syntheticRelativeDepth();
      // No anchors at all.
      expect(raw.fitToMetric(const <DepthMetricAnchor>[]).isFitted, isFalse);
    });
  });

  group('cue fusion', () {
    test('agreeing cues give a tight, confident estimate', () {
      final ObjectTrack track =
          trackAt(objectClass: ObjectClass.car, range: 25);
      final DepthMap depth = fusion.fitToGroundPlane(
        depth: syntheticRelativeDepth(
          objects: <BoundingBox, double>{track.box: 25.0},
        ),
        calibration: calibration,
      );

      final FusedDistance result = fusion.fuse(
        track: track,
        calibration: calibration,
        depth: depth,
        egoMotion: ego(),
      );

      expect(result.distanceMeters, closeTo(25, 2.5));
      expect(result.cues.length, greaterThanOrEqualTo(3));
      expect(result.agreement, greaterThan(0.5));
      expect(result.confidence.value, greaterThan(0.5));
    });

    test('works with no depth model at all, on geometry alone', () {
      final ObjectTrack track =
          trackAt(objectClass: ObjectClass.car, range: 18);
      final FusedDistance result = fusion.fuse(
        track: track,
        calibration: calibration,
        depth: DepthMap.unavailable(frameId: 1, timestampMicros: 0),
        egoMotion: ego(),
      );

      expect(result.cues.map((DepthCue c) => c.source),
          isNot(contains('depth-net')));
      expect(result.distanceMeters, closeTo(18, 3));
      expect(result.confidence.value, greaterThan(0.3));
    });

    test('a wildly wrong depth cue is down-weighted, not trusted', () {
      final ObjectTrack track =
          trackAt(objectClass: ObjectClass.car, range: 20);
      // The network claims 70 m for an object that geometry puts at 20 m.
      final DepthMap depth = fusion.fitToGroundPlane(
        depth: syntheticRelativeDepth(
          objects: <BoundingBox, double>{track.box: 70.0},
        ),
        calibration: calibration,
      );

      final FusedDistance result = fusion.fuse(
        track: track,
        calibration: calibration,
        depth: depth,
        egoMotion: ego(),
      );

      expect(result.distanceMeters, lessThan(32),
          reason: 'the outlier must not drag the estimate to 70 m');
      expect(result.agreement, lessThan(0.85),
          reason: 'disagreement must be visible in the agreement score');
    });

    test('confidence falls with distance as geometry degrades', () {
      final FusedDistance near = fusion.fuse(
        track: trackAt(objectClass: ObjectClass.car, range: 12),
        calibration: calibration,
        depth: DepthMap.unavailable(frameId: 1, timestampMicros: 0),
        egoMotion: ego(),
      );
      final FusedDistance far = fusion.fuse(
        track: trackAt(objectClass: ObjectClass.car, range: 70),
        calibration: calibration,
        depth: DepthMap.unavailable(frameId: 1, timestampMicros: 0),
        egoMotion: ego(),
      );
      expect(far.sigmaMeters, greaterThan(near.sigmaMeters));
    });

    test('an uncalibrated camera widens the geometric uncertainty', () {
      final CameraCalibration guessed =
          calibration.copyWith(isCalibrated: false);
      final ObjectTrack track =
          trackAt(objectClass: ObjectClass.car, range: 20);

      final FusedDistance calibrated = fusion.fuse(
        track: track,
        calibration: calibration,
        depth: DepthMap.unavailable(frameId: 1, timestampMicros: 0),
        egoMotion: ego(),
      );
      final FusedDistance uncalibrated = fusion.fuse(
        track: track,
        calibration: guessed,
        depth: DepthMap.unavailable(frameId: 1, timestampMicros: 0),
        egoMotion: ego(),
      );
      expect(uncalibrated.sigmaMeters,
          greaterThan(calibrated.sigmaMeters));
    });

    test('a truck and a car with the same box are placed differently', () {
      // Same pixel size, very different physical size: the size cue must pull
      // the truck much further away.
      final ObjectTrack car =
          trackAt(objectClass: ObjectClass.car, range: 20);
      final ObjectTrack truck = ObjectTrack(
        id: 2,
        objectClass: ObjectClass.truck,
        confidence: Confidence(0.9),
        box: car.box,
        estimatedDistanceMeters: 20,
        distanceConfidence: Confidence(0.5),
        position: const Vec2(0, 20),
        velocityWorld: const Vec2.zero(),
        relativeVelocity: const Vec2.zero(),
        direction: MotionDirection.approaching,
        laneRelation: LaneRelation.egoLane,
        collisionRisk: CollisionRisk.low,
        timeToCollisionSeconds: null,
        firstSeenMicros: 0,
        lastSeenMicros: 0,
        frameId: 0,
        age: 1,
        isConfirmed: true,
      );

      final DepthCue carSize = fusion
          .fuse(
            track: car,
            calibration: calibration,
            depth: DepthMap.unavailable(frameId: 1, timestampMicros: 0),
            egoMotion: ego(),
          )
          .cues
          .firstWhere((DepthCue c) => c.source == 'size');
      final DepthCue truckSize = fusion
          .fuse(
            track: truck,
            calibration: calibration,
            depth: DepthMap.unavailable(frameId: 1, timestampMicros: 0),
            egoMotion: ego(),
          )
          .cues
          .firstWhere((DepthCue c) => c.source == 'size');

      expect(truckSize.distanceMeters,
          greaterThan(carSize.distanceMeters * 1.8));
      // A truck's size prior is much looser, so its sigma is relatively wider.
      expect(truckSize.sigmaMeters / truckSize.distanceMeters,
          greaterThan(carSize.sigmaMeters / carSize.distanceMeters));
    });
  });

  group('motion parallax', () {
    test('recovers range for a stationary object from ego displacement', () {
      // The object is stationary; we travel 10 m/s for 0.1 s = 1 m.
      // At 20 m the apparent height grows by 20/19.
      const double range = 20;
      final ObjectTrack base =
          trackAt(objectClass: ObjectClass.car, range: range);
      final ObjectTrack previous =
          trackAt(objectClass: ObjectClass.car, range: range + 1);

      final ObjectTrack track = trackAt(
        objectClass: ObjectClass.car,
        range: range,
        direction: MotionDirection.stationary,
        historyBoxes: <BoundingBox>[previous.box, base.box],
      );

      final FusedDistance result = fusion.fuse(
        track: track,
        calibration: calibration,
        depth: DepthMap.unavailable(frameId: 1, timestampMicros: 0),
        egoMotion: ego(speed: 10),
        previousDistanceMeters: range + 1,
        previousTimestampSeconds: 0.3,
        currentTimestampSeconds: 0.4,
      );

      final Iterable<DepthCue> parallax =
          result.cues.where((DepthCue c) => c.source == 'parallax');
      expect(parallax, isNotEmpty);
      expect(parallax.first.distanceMeters, closeTo(range, 4));
    });

    test('is not offered for a moving object', () {
      final ObjectTrack track = trackAt(
        objectClass: ObjectClass.car,
        range: 20,
        direction: MotionDirection.approaching,
        historyBoxes: <BoundingBox>[
          trackAt(objectClass: ObjectClass.car, range: 21).box,
          trackAt(objectClass: ObjectClass.car, range: 20).box,
        ],
      );
      final FusedDistance result = fusion.fuse(
        track: track,
        calibration: calibration,
        depth: DepthMap.unavailable(frameId: 1, timestampMicros: 0),
        egoMotion: ego(speed: 10),
        previousDistanceMeters: 21,
        previousTimestampSeconds: 0.3,
        currentTimestampSeconds: 0.4,
      );
      expect(result.cues.where((DepthCue c) => c.source == 'parallax'),
          isEmpty);
    });

    test('is not offered when the vehicle has barely moved', () {
      final ObjectTrack track = trackAt(
        objectClass: ObjectClass.car,
        range: 20,
        direction: MotionDirection.stationary,
        historyBoxes: <BoundingBox>[
          trackAt(objectClass: ObjectClass.car, range: 20.05).box,
          trackAt(objectClass: ObjectClass.car, range: 20).box,
        ],
      );
      final FusedDistance result = fusion.fuse(
        track: track,
        calibration: calibration,
        depth: DepthMap.unavailable(frameId: 1, timestampMicros: 0),
        egoMotion: ego(speed: 0.4),
        previousDistanceMeters: 20.05,
        previousTimestampSeconds: 0.3,
        currentTimestampSeconds: 0.4,
      );
      expect(result.cues.where((DepthCue c) => c.source == 'parallax'),
          isEmpty);
    });
  });

  group('safety bias', () {
    test('a disputed pedestrian distance is biased towards the nearer cue',
        () {
      final ObjectTrack person =
          trackAt(objectClass: ObjectClass.person, range: 15);
      // The network insists the pedestrian is much further away.
      final DepthMap depth = fusion.fitToGroundPlane(
        depth: syntheticRelativeDepth(
          objects: <BoundingBox, double>{person.box: 40.0},
        ),
        calibration: calibration,
      );

      final FusedDistance withDispute = fusion.fuse(
        track: person,
        calibration: calibration,
        depth: depth,
        egoMotion: ego(),
      );
      expect(withDispute.agreement, lessThan(0.75));
      // Must not drift towards the optimistic 40 m reading.
      expect(withDispute.distanceMeters, lessThan(22));
    });
  });
}
