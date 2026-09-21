import 'dart:typed_data';

import 'package:aicar/road/drivable_area_builder.dart';
import 'package:aicar/road/road_segmentation.dart';
import 'package:flutter_test/flutter_test.dart';

import '../support/world_builder.dart';

/// Paint is not the end of the road.
///
/// An appearance-based segmenter sees a zebra crossing as a bright band
/// across the carriageway and stops classifying road there. Before this was
/// handled, the corridor ended at every crossing, the planner reported the
/// path blocked, and the stack proposed braking for a piece of paint.
void main() {
  const DrivableAreaBuilder builder = DrivableAreaBuilder();

  /// A segmentation grid that is all road, except for [gapRows] rows of
  /// "unknown" starting [gapStart] rows below the horizon.
  RoadSegmentation grid({
    int width = 128,
    int height = 72,
    int? gapStartRow,
    int gapRows = 0,
  }) {
    final Uint8List classes = Uint8List(width * height);
    final Float32List confidence = Float32List(width * height);
    final int horizon =
        (testCalibration.horizonYNormalized * height).round();

    for (int y = 0; y < height; y++) {
      final bool inGap = gapStartRow != null &&
          y >= gapStartRow - gapRows + 1 &&
          y <= gapStartRow;
      for (int x = 0; x < width; x++) {
        final int i = y * width + x;
        if (y <= horizon) {
          classes[i] = SurfaceClass.unknown.index;
          confidence[i] = 0.2;
        } else if (inGap) {
          // Bright paint: smooth, so not an obstacle, but not road either.
          classes[i] = SurfaceClass.unknown.index;
          confidence[i] = 0.35;
        } else {
          classes[i] = SurfaceClass.drivableRoad.index;
          confidence[i] = 0.8;
        }
      }
    }

    return RoadSegmentation(
      width: width,
      height: height,
      classIndices: classes,
      classConfidence: confidence,
      frameId: 1,
      timestampMicros: 0,
      modelName: 'test',
    );
  }

  double rangeOf(RoadSegmentation s) => builder
      .build(segmentation: s, calibration: testCalibration)
      .maxRangeMeters;

  test('an uninterrupted road gives the full corridor', () {
    expect(rangeOf(grid()), greaterThan(10));
  });

  test('a painted band across the road does not end the corridor', () {
    final double clear = rangeOf(grid());
    // A band a few grid rows deep, partway up the image.
    final int horizon =
        (testCalibration.horizonYNormalized * 72).round();
    final double bridged =
        rangeOf(grid(gapStartRow: horizon + 14, gapRows: 3));

    expect(bridged, greaterThan(clear * 0.8),
        reason: 'the road resumes beyond the paint, so the paint is surface, '
            'not the end of the road');
  });

  test('the bridged samples are marked as inferred, not observed', () {
    final int horizon =
        (testCalibration.horizonYNormalized * 72).round();
    final DrivableArea area = builder.build(
      segmentation: grid(gapStartRow: horizon + 14, gapRows: 3),
      calibration: testCalibration,
    );
    final DrivableArea clean = builder.build(
      segmentation: grid(),
      calibration: testCalibration,
    );
    expect(area.confidence, lessThan(clean.confidence),
        reason: 'a corridor that had to be inferred across a gap is worth '
            'less than one that was seen');
  });

  test('a road that genuinely ends still ends', () {
    // Everything beyond the band is non-road: nothing resumes, so there is
    // nothing to bridge to.
    final int height = 72;
    final int width = 128;
    final Uint8List classes = Uint8List(width * height);
    final Float32List confidence = Float32List(width * height);
    final int horizon =
        (testCalibration.horizonYNormalized * height).round();
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        final int i = y * width + x;
        final bool road = y > horizon + 18;
        classes[i] = road
            ? SurfaceClass.drivableRoad.index
            : SurfaceClass.unknown.index;
        confidence[i] = road ? 0.8 : 0.3;
      }
    }

    final DrivableArea area = builder.build(
      segmentation: RoadSegmentation(
        width: width,
        height: height,
        classIndices: classes,
        classConfidence: confidence,
        frameId: 1,
        timestampMicros: 0,
        modelName: 'test',
      ),
      calibration: testCalibration,
    );
    final double full = rangeOf(grid());
    expect(area.maxRangeMeters, lessThan(full));
  });
}
