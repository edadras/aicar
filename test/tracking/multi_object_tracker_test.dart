import 'package:aicar/camera/camera_calibration.dart';
import 'package:aicar/core/geometry.dart';
import 'package:aicar/perception/detection.dart';
import 'package:aicar/perception/object_class.dart';
import 'package:aicar/sensors/ego_motion.dart';
import 'package:aicar/tracking/multi_object_tracker.dart';
import 'package:aicar/tracking/object_track.dart';
import 'package:flutter_test/flutter_test.dart';

const CameraCalibration calibration = CameraCalibration(
  imageWidth: 640,
  imageHeight: 360,
  horizontalFovDegrees: 73.7,
  cameraHeightMeters: 1.2,
  pitchDegrees: 2.0,
  rollDegrees: 0,
  yawDegrees: 0,
  lateralOffsetMeters: 0,
  longitudinalOffsetMeters: 2.0,
  isCalibrated: true,
);

/// Build the detection an ideal detector would emit for an object standing at
/// [position] in the vehicle frame — projecting its contact patch and its
/// class-typical size through the same camera model the tracker inverts.
Detection syntheticDetection({
  required ObjectClass objectClass,
  required Vec2 position,
  required int frameId,
  required int timestampMicros,
  double score = 0.9,
}) {
  final PixelPoint? contact = calibration.projectGroundToImage(position);
  if (contact == null) {
    throw ArgumentError('$position is not visible from this camera');
  }
  final PhysicalSizePrior prior = objectClass.sizePrior;
  final double range = position.length;
  final double heightPx = calibration.fy * prior.height / range;
  final double widthPx = calibration.fx * prior.width / range;

  return Detection(
    objectClass: objectClass,
    box: BoundingBox.fromPixels(
      left: contact.u - widthPx / 2,
      top: contact.v - heightPx,
      right: contact.u + widthPx / 2,
      bottom: contact.v,
      imageWidth: calibration.imageWidth,
      imageHeight: calibration.imageHeight,
    ),
    score: score,
    frameId: frameId,
    timestampMicros: timestampMicros,
    rawLabel: objectClass.label,
  );
}

EgoMotionState ego({double speed = 0, double yawRate = 0, int ts = 0}) =>
    EgoMotionState(
      speedMps: speed,
      headingDegrees: 0,
      yawRateRadPerS: yawRate,
      longitudinalAccelMps2: 0,
      lateralAccelMps2: 0,
      position: null,
      timestampMicros: ts,
      speedConfidence: 0.9,
      headingConfidence: 0.9,
    );

DetectionResult wrap(List<Detection> dets, int frameId, int ts) =>
    DetectionResult(
      detections: dets,
      frameId: frameId,
      timestampMicros: ts,
      inferenceMicros: 1000,
      modelName: 'synthetic',
    );

void main() {
  group('MultiObjectTracker identity', () {
    test('keeps one stable id for an approaching vehicle', () {
      final MultiObjectTracker tracker =
          MultiObjectTracker(calibration: calibration);

      const double dt = 0.05; // 20 FPS
      double range = 45;
      const double closingSpeed = 10; // m/s
      int? firstId;

      for (int i = 0; i < 40; i++) {
        final int ts = (i * dt * 1e6).round();
        final List<ObjectTrack> tracks = tracker.update(
          detections: wrap(
            <Detection>[
              syntheticDetection(
                objectClass: ObjectClass.car,
                position: Vec2(0, range),
                frameId: i,
                timestampMicros: ts,
              ),
            ],
            i,
            ts,
          ),
          egoMotion: ego(ts: ts),
          timestampMicros: ts,
        );
        range -= closingSpeed * dt;

        if (i >= 3) {
          expect(tracks, hasLength(1), reason: 'frame $i');
          firstId ??= tracks.first.id;
          expect(tracks.first.id, firstId, reason: 'id must not change');
        }
      }
    });

    test('converges on the true closing speed', () {
      final MultiObjectTracker tracker =
          MultiObjectTracker(calibration: calibration);

      const double dt = 0.05;
      double range = 50;
      const double closingSpeed = 12;

      List<ObjectTrack> tracks = const <ObjectTrack>[];
      for (int i = 0; i < 60; i++) {
        final int ts = (i * dt * 1e6).round();
        tracks = tracker.update(
          detections: wrap(
            <Detection>[
              syntheticDetection(
                objectClass: ObjectClass.car,
                position: Vec2(0, range),
                frameId: i,
                timestampMicros: ts,
              ),
            ],
            i,
            ts,
          ),
          egoMotion: ego(ts: ts),
          timestampMicros: ts,
        );
        range -= closingSpeed * dt;
      }

      expect(tracks, hasLength(1));
      final ObjectTrack t = tracks.first;
      expect(t.relativeVelocity.y, closeTo(-closingSpeed, 2.0));
      expect(t.closingSpeedMps, greaterThan(0));
      expect(t.velocityConfidence, greaterThan(0.5));
      expect(t.estimatedDistanceMeters, closeTo(range + closingSpeed * dt, 3.0));
    });

    test('ego motion is removed: a parked car reads as stationary', () {
      final MultiObjectTracker tracker =
          MultiObjectTracker(calibration: calibration);

      const double dt = 0.05;
      const double egoSpeed = 14; // we drive past a parked car
      double range = 60;

      List<ObjectTrack> tracks = const <ObjectTrack>[];
      for (int i = 0; i < 50 && range > 8; i++) {
        final int ts = (i * dt * 1e6).round();
        tracks = tracker.update(
          detections: wrap(
            <Detection>[
              syntheticDetection(
                objectClass: ObjectClass.car,
                position: Vec2(2.8, range),
                frameId: i,
                timestampMicros: ts,
              ),
            ],
            i,
            ts,
          ),
          egoMotion: ego(speed: egoSpeed, ts: ts),
          timestampMicros: ts,
        );
        range -= egoSpeed * dt;
      }

      expect(tracks, hasLength(1));
      final ObjectTrack t = tracks.first;
      // Relative velocity should be ~-egoSpeed; absolute world speed ~0.
      expect(t.relativeVelocity.y, closeTo(-egoSpeed, 3.0));
      expect(t.direction, MotionDirection.stationary);
    });
  });

  group('MultiObjectTracker robustness', () {
    test('does not confirm a single-frame false positive', () {
      final MultiObjectTracker tracker =
          MultiObjectTracker(calibration: calibration);

      final List<ObjectTrack> first = tracker.update(
        detections: wrap(
          <Detection>[
            syntheticDetection(
              objectClass: ObjectClass.person,
              position: const Vec2(1.0, 15),
              frameId: 0,
              timestampMicros: 0,
            ),
          ],
          0,
          0,
        ),
        egoMotion: ego(),
        timestampMicros: 0,
      );
      expect(first, isEmpty, reason: 'not yet confirmed');

      // The detection vanishes; the unconfirmed track must be discarded.
      for (int i = 1; i < 5; i++) {
        final int ts = (i * 0.05 * 1e6).round();
        final List<ObjectTrack> t = tracker.update(
          detections: wrap(const <Detection>[], i, ts),
          egoMotion: ego(ts: ts),
          timestampMicros: ts,
        );
        expect(t, isEmpty);
      }
      expect(tracker.allTracks, isEmpty);
    });

    test('survives a short occlusion without losing the id', () {
      final MultiObjectTracker tracker =
          MultiObjectTracker(calibration: calibration);

      const double dt = 0.05;
      double range = 40;
      int? id;

      for (int i = 0; i < 30; i++) {
        final int ts = (i * dt * 1e6).round();
        // Frames 12-16: the object is hidden behind something.
        final bool occluded = i >= 12 && i <= 16;
        final List<ObjectTrack> tracks = tracker.update(
          detections: wrap(
            occluded
                ? const <Detection>[]
                : <Detection>[
                    syntheticDetection(
                      objectClass: ObjectClass.car,
                      position: Vec2(0, range),
                      frameId: i,
                      timestampMicros: ts,
                    ),
                  ],
            i,
            ts,
          ),
          egoMotion: ego(ts: ts),
          timestampMicros: ts,
        );
        range -= 8 * dt;

        if (i == 11) {
          expect(tracks, hasLength(1));
          id = tracks.first.id;
        }
        if (i == 20) {
          expect(tracks, hasLength(1));
          expect(tracks.first.id, id, reason: 'id must survive occlusion');
        }
      }
    });

    test('keeps two nearby vehicles apart', () {
      final MultiObjectTracker tracker =
          MultiObjectTracker(calibration: calibration);

      const double dt = 0.05;
      double leftRange = 30;
      double rightRange = 34;

      List<ObjectTrack> tracks = const <ObjectTrack>[];
      for (int i = 0; i < 30; i++) {
        final int ts = (i * dt * 1e6).round();
        tracks = tracker.update(
          detections: wrap(
            <Detection>[
              syntheticDetection(
                objectClass: ObjectClass.car,
                position: Vec2(-1.8, leftRange),
                frameId: i,
                timestampMicros: ts,
              ),
              syntheticDetection(
                objectClass: ObjectClass.car,
                position: Vec2(1.8, rightRange),
                frameId: i,
                timestampMicros: ts,
              ),
            ],
            i,
            ts,
          ),
          egoMotion: ego(ts: ts),
          timestampMicros: ts,
        );
        leftRange -= 6 * dt;
        rightRange -= 4 * dt;
      }

      expect(tracks, hasLength(2));
      expect(tracks.map((ObjectTrack t) => t.id).toSet(), hasLength(2));
      final List<ObjectTrack> sorted = List<ObjectTrack>.from(tracks)
        ..sort((ObjectTrack a, ObjectTrack b) =>
            a.position.x.compareTo(b.position.x));
      expect(sorted.first.position.x, lessThan(0));
      expect(sorted.last.position.x, greaterThan(0));
    });

    test('rejects an implausible velocity from a distance outlier', () {
      final MultiObjectTracker tracker =
          MultiObjectTracker(calibration: calibration);

      const double dt = 0.05;
      for (int i = 0; i < 20; i++) {
        final int ts = (i * dt * 1e6).round();
        // A pedestrian that "teleports" between 30 m and 10 m every frame.
        final double range = i.isEven ? 30 : 10;
        tracker.update(
          detections: wrap(
            <Detection>[
              syntheticDetection(
                objectClass: ObjectClass.person,
                position: Vec2(0, range),
                frameId: i,
                timestampMicros: ts,
              ),
            ],
            i,
            ts,
          ),
          egoMotion: ego(ts: ts),
          timestampMicros: ts,
        );
      }

      for (final ObjectTrack t in tracker.allTracks) {
        // A person cannot exceed ~6 m/s; the tracker clamps to that plus a
        // margin rather than reporting hundreds of m/s.
        expect(t.relativeVelocity.length,
            lessThanOrEqualTo(ObjectClass.person.maxPlausibleSpeedMps + 6));
      }
    });

    test('a long time gap resets the tracker rather than extrapolating', () {
      final MultiObjectTracker tracker =
          MultiObjectTracker(calibration: calibration);

      for (int i = 0; i < 10; i++) {
        final int ts = (i * 0.05 * 1e6).round();
        tracker.update(
          detections: wrap(
            <Detection>[
              syntheticDetection(
                objectClass: ObjectClass.car,
                position: Vec2(0, 25),
                frameId: i,
                timestampMicros: ts,
              ),
            ],
            i,
            ts,
          ),
          egoMotion: ego(ts: ts),
          timestampMicros: ts,
        );
      }
      expect(tracker.allTracks, isNotEmpty);

      final List<ObjectTrack> after = tracker.update(
        detections: wrap(const <Detection>[], 100, 10 * 1000000),
        egoMotion: ego(ts: 10 * 1000000),
        timestampMicros: 10 * 1000000,
      );
      expect(after, isEmpty);
      expect(tracker.allTracks, isEmpty);
    });
  });

  group('distance refinement feedback', () {
    test('a refined distance moves the estimate towards the new value', () {
      final MultiObjectTracker tracker =
          MultiObjectTracker(calibration: calibration);

      List<ObjectTrack> tracks = const <ObjectTrack>[];
      for (int i = 0; i < 6; i++) {
        final int ts = (i * 0.05 * 1e6).round();
        tracks = tracker.update(
          detections: wrap(
            <Detection>[
              syntheticDetection(
                objectClass: ObjectClass.car,
                position: const Vec2(0, 30),
                frameId: i,
                timestampMicros: ts,
              ),
            ],
            i,
            ts,
          ),
          egoMotion: ego(ts: ts),
          timestampMicros: ts,
        );
      }
      expect(tracks, hasLength(1));
      final double before = tracks.first.estimatedDistanceMeters;
      expect(before, closeTo(30, 2));

      tracker.refineDistance(tracks.first.id, 24.0, 0.9);

      final List<ObjectTrack> refreshed = tracker.update(
        detections: wrap(
          <Detection>[
            syntheticDetection(
              objectClass: ObjectClass.car,
              position: const Vec2(0, 30),
              frameId: 6,
              timestampMicros: 300000,
            ),
          ],
          6,
          300000,
        ),
        egoMotion: ego(ts: 300000),
        timestampMicros: 300000,
      );
      expect(refreshed, hasLength(1));
      expect(refreshed.first.estimatedDistanceMeters, lessThan(before));
    });

    test('a zero-confidence refinement is ignored', () {
      final MultiObjectTracker tracker =
          MultiObjectTracker(calibration: calibration);
      for (int i = 0; i < 5; i++) {
        final int ts = (i * 0.05 * 1e6).round();
        tracker.update(
          detections: wrap(
            <Detection>[
              syntheticDetection(
                objectClass: ObjectClass.car,
                position: const Vec2(0, 30),
                frameId: i,
                timestampMicros: ts,
              ),
            ],
            i,
            ts,
          ),
          egoMotion: ego(ts: ts),
          timestampMicros: ts,
        );
      }
      final ObjectTrack t = tracker.allTracks.first;
      final double before = t.estimatedDistanceMeters;
      tracker.refineDistance(t.id, 5.0, 0.0);
      expect(tracker.allTracks.first.estimatedDistanceMeters, before);
    });
  });
}
