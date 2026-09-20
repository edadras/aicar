import 'package:aicar/camera/camera_calibration.dart';
import 'package:aicar/camera/camera_frame.dart';
import 'package:aicar/road/classical_lane_detector.dart';
import 'package:aicar/road/lane.dart';
import 'package:flutter_test/flutter_test.dart';

import '../support/synthetic_road.dart';

const CameraCalibration calibration = CameraCalibration(
  imageWidth: 640,
  imageHeight: 360,
  horizontalFovDegrees: 73.7,
  cameraHeightMeters: 1.25,
  pitchDegrees: 3.0,
  rollDegrees: 0,
  yawDegrees: 0,
  lateralOffsetMeters: 0,
  longitudinalOffsetMeters: 2.0,
  isCalibrated: true,
);

void main() {
  group('ClassicalLaneDetector on a straight painted road', () {
    late ClassicalLaneDetector detector;

    setUp(() {
      detector = ClassicalLaneDetector(calibration: calibration);
    });

    test('recovers both boundaries at the right lateral positions', () async {
      const SyntheticRoad road = SyntheticRoad(calibration: calibration);
      final CameraFrame frame = road.renderFrame();

      final LaneDetectionResult result = await detector.detectLanes(frame);

      expect(result.mode, LaneMode.bothBoundaries);
      expect(result.left, isNotNull);
      expect(result.right, isNotNull);

      // Within 25 cm of the truth at a representative look-ahead.
      expect(result.left!.lateralAtUnchecked(15), closeTo(-1.75, 0.25));
      expect(result.right!.lateralAtUnchecked(15), closeTo(1.75, 0.25));
      expect(result.left!.confidence.value, greaterThan(0.4));
      expect(result.right!.confidence.value, greaterThan(0.4));
    });

    test('measures the lane width', () async {
      const SyntheticRoad road = SyntheticRoad(calibration: calibration);
      final CameraFrame frame = road.renderFrame();
      // Lane width is tracked over time, so run a few frames.
      LaneDetectionResult? result;
      for (int i = 0; i < 12; i++) {
        result = await detector.detectLanes(
          road.renderFrame(id: i, timestampMicros: i * 50000),
          previous: result,
        );
      }
      expect(frame.width, 640);
      expect(result!.laneWidthMeters, closeTo(3.5, 0.4));
      expect(result.laneWidthConfidence, greaterThan(0.2));
    });

    test('centres the ego vehicle when it is centred', () async {
      const SyntheticRoad road = SyntheticRoad(calibration: calibration);
      final LaneDetectionResult result =
          await detector.detectLanes(road.renderFrame());
      expect(result.egoLateralOffsetMeters, closeTo(0, 0.3));
      expect(result.egoHeadingErrorRadians, closeTo(0, 0.05));
    });

    test('detects a lateral offset when the vehicle is off centre', () async {
      // Shifting the painted lines right by 0.6 m is the same as the vehicle
      // sitting 0.6 m left of centre.
      const SyntheticRoad road =
          SyntheticRoad(calibration: calibration, lateralOffset: 0.6);
      final LaneDetectionResult result =
          await detector.detectLanes(road.renderFrame());
      expect(result.mode, LaneMode.bothBoundaries);
      expect(result.egoLateralOffsetMeters, closeTo(-0.6, 0.35));
    });

    test('produces a centreline midway between the boundaries', () async {
      const SyntheticRoad road = SyntheticRoad(calibration: calibration);
      final LaneDetectionResult result =
          await detector.detectLanes(road.renderFrame());
      expect(result.centerline, isNotNull);
      expect(result.centerline!.evaluate(20), closeTo(0, 0.3));
    });
  });

  group('ClassicalLaneDetector on harder scenes', () {
    test('follows a curved road', () async {
      final ClassicalLaneDetector detector =
          ClassicalLaneDetector(calibration: calibration);
      // Gentle right-hand curve: 0.0015 * y² gives ~0.6 m offset at 20 m.
      const SyntheticRoad road =
          SyntheticRoad(calibration: calibration, curvature: 0.0015);
      final LaneDetectionResult result =
          await detector.detectLanes(road.renderFrame());

      expect(result.left, isNotNull);
      expect(result.right, isNotNull);
      final double expectedAt25 = 0.0015 * 25 * 25;
      expect(
        result.centerline!.evaluate(25),
        closeTo(expectedAt25, 0.6),
      );
      // The curve must actually bend, not be fitted as a straight line.
      expect(result.centerline!.coefficients[2].abs(), greaterThan(0.0005));
    });

    test('classifies a dashed line as dashed and a solid line as solid',
        () async {
      final ClassicalLaneDetector detector =
          ClassicalLaneDetector(calibration: calibration);
      const SyntheticRoad road = SyntheticRoad(
        calibration: calibration,
        dashedLanes: <int>{1}, // right line dashed
      );
      final LaneDetectionResult result =
          await detector.detectLanes(road.renderFrame());

      expect(result.left?.lineType, LineType.solid);
      expect(result.right?.lineType, LineType.dashed);
    });

    test('survives sensor noise', () async {
      final ClassicalLaneDetector detector =
          ClassicalLaneDetector(calibration: calibration);
      const SyntheticRoad road =
          SyntheticRoad(calibration: calibration, noiseAmplitude: 18);
      final LaneDetectionResult result =
          await detector.detectLanes(road.renderFrame());

      expect(result.mode, LaneMode.bothBoundaries);
      expect(result.left!.lateralAtUnchecked(12), closeTo(-1.75, 0.4));
      expect(result.right!.lateralAtUnchecked(12), closeTo(1.75, 0.4));
    });

    test('reports NO_LANE_MODE on unmarked asphalt rather than inventing lanes',
        () async {
      final ClassicalLaneDetector detector =
          ClassicalLaneDetector(calibration: calibration);
      const SyntheticRoad road = SyntheticRoad(
        calibration: calibration,
        laneCenters: <double>[], // no markings at all
      );
      final LaneDetectionResult result =
          await detector.detectLanes(road.renderFrame());

      expect(result.boundaries, isEmpty);
      expect(
        result.mode,
        anyOf(LaneMode.noLane, LaneMode.none),
      );
      expect(result.overallConfidence, lessThan(0.3));
    });

    test('finds the adjacent lane when there are three markings', () async {
      final ClassicalLaneDetector detector =
          ClassicalLaneDetector(calibration: calibration);
      const SyntheticRoad road = SyntheticRoad(
        calibration: calibration,
        laneCenters: <double>[-1.75, 1.75, 5.25],
      );
      final LaneDetectionResult result =
          await detector.detectLanes(road.renderFrame());

      expect(result.mode, LaneMode.bothBoundaries);
      expect(result.hasRightAdjacentLane, isTrue);
      final LaneBoundary adjacent =
          result.boundaryAt(LanePosition.rightAdjacent)!;
      expect(adjacent.lateralAtUnchecked(12), closeTo(5.25, 0.5));
    });

    test('a single visible boundary yields single-boundary mode', () async {
      final ClassicalLaneDetector detector =
          ClassicalLaneDetector(calibration: calibration);
      const SyntheticRoad road =
          SyntheticRoad(calibration: calibration, laneCenters: <double>[-1.75]);
      final LaneDetectionResult result =
          await detector.detectLanes(road.renderFrame());

      expect(result.mode, LaneMode.singleBoundary);
      expect(result.left, isNotNull);
      expect(result.right, isNull);
      // The centreline is still available, inferred from the tracked width.
      expect(result.centerline, isNotNull);
      expect(result.centerline!.evaluate(15), greaterThan(-1.0));
    });
  });

  group('geometry rejection', () {
    test('an implausibly wide lane pair is not reported as a lane', () async {
      final ClassicalLaneDetector detector =
          ClassicalLaneDetector(calibration: calibration);
      // 3.4 m either side = a 6.8 m "lane", far wider than any real one.
      const SyntheticRoad road = SyntheticRoad(
        calibration: calibration,
        laneCenters: <double>[-3.4, 3.4],
      );
      final LaneDetectionResult result =
          await detector.detectLanes(road.renderFrame());
      expect(result.mode, isNot(LaneMode.bothBoundaries));
    });
  });
}
