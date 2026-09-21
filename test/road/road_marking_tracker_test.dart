import 'package:aicar/core/confidence.dart';
import 'package:aicar/road/road_marking.dart';
import 'package:aicar/road/road_marking_tracker.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  RoadMarking sighting(
    RoadMarkingType type,
    double distance, {
    double confidence = 0.6,
    int frameId = 1,
  }) =>
      RoadMarking(
        type: type,
        distanceMeters: distance,
        depthMeters: 3,
        lateralCenterMeters: 0,
        widthMeters: 6,
        confidence: Confidence(confidence),
        frameId: frameId,
        timestampMicros: frameId * 50000,
      );

  RoadMarkingResult frame(List<RoadMarking> markings, {int id = 1}) =>
      RoadMarkingResult(
        markings: markings,
        frameId: id,
        timestampMicros: id * 50000,
        searchRangeMeters: 32,
      );

  group('accumulation', () {
    test('one sighting is never enough to act on', () {
      final RoadMarkingTracker t = RoadMarkingTracker();
      t.update(
        result: frame(<RoadMarking>[
          sighting(RoadMarkingType.crosswalk, 20),
        ]),
        travelledMeters: 0,
      );
      expect(t.all, hasLength(1));
      expect(t.confirmed, isEmpty);
    });

    test('repeated sightings of the same crossing confirm it', () {
      final RoadMarkingTracker t = RoadMarkingTracker();
      // Approaching at 10 m/s, one frame every 0.5 s: 5 m closer each time.
      double d = 25;
      for (int i = 0; i < 4; i++) {
        t.update(
          result: frame(<RoadMarking>[
            sighting(RoadMarkingType.crosswalk, d, frameId: i),
          ], id: i),
          travelledMeters: i == 0 ? 0 : 5,
        );
        d -= 5;
      }

      expect(t.confirmed, hasLength(1));
      final RoadMarking m = t.confirmed.single;
      expect(m.observationCount, greaterThanOrEqualTo(3));
      expect(m.distanceMeters, closeTo(10, 0.1),
          reason: 'the tracked distance follows the newest measurement');
    });

    test('a one-frame phantom decays away instead of sticking', () {
      final RoadMarkingTracker t = RoadMarkingTracker();
      t.update(
        result: frame(<RoadMarking>[
          sighting(RoadMarkingType.stopLine, 18, confidence: 0.5),
        ]),
        travelledMeters: 0,
      );
      for (int i = 1; i < 12; i++) {
        t.update(result: frame(const <RoadMarking>[], id: i),
            travelledMeters: 0.5);
      }
      expect(t.all, isEmpty);
      expect(t.confirmed, isEmpty);
    });

    test('a marking beyond the search range is not penalised for absence', () {
      final RoadMarkingTracker t = RoadMarkingTracker();
      t.update(
        result: frame(<RoadMarking>[
          sighting(RoadMarkingType.crosswalk, 40, confidence: 0.6),
        ]),
        travelledMeters: 0,
      );
      final double before = t.all.single.confidence.value;
      // We could only see to 32 m, so not re-seeing something at 40 m says
      // nothing at all about whether it is there.
      t.update(result: frame(const <RoadMarking>[], id: 2),
          travelledMeters: 0);
      expect(t.all.single.confidence.value, closeTo(before, 1e-9));
    });

    test('markings we have driven over are forgotten', () {
      final RoadMarkingTracker t = RoadMarkingTracker();
      for (int i = 0; i < 4; i++) {
        t.update(
          result: frame(<RoadMarking>[
            sighting(RoadMarkingType.speedBump, 12.0 - i * 4, frameId: i),
          ], id: i),
          travelledMeters: i == 0 ? 0 : 4,
        );
      }
      expect(t.confirmed, isNotEmpty);
      for (int i = 4; i < 10; i++) {
        t.update(result: frame(const <RoadMarking>[], id: i),
            travelledMeters: 4);
      }
      expect(t.all, isEmpty);
    });
  });

  group('advisory speed', () {
    test('eases in with distance rather than snapping', () {
      final RoadMarkingTracker t = RoadMarkingTracker();
      for (int i = 0; i < 4; i++) {
        t.update(
          result: frame(<RoadMarking>[
            sighting(RoadMarkingType.speedBump, 40, confidence: 0.7,
                frameId: i),
          ], id: i),
          travelledMeters: 0,
        );
      }
      final double? far = t.advisorySpeedMps(20);
      expect(far, isNotNull);
      expect(far, greaterThan(RoadMarkingType.speedBump.advisorySpeedMps));
      expect(far, lessThanOrEqualTo(20));
    });

    test('a stop line on its own imposes no speed', () {
      final RoadMarkingTracker t = RoadMarkingTracker();
      for (int i = 0; i < 4; i++) {
        t.update(
          result: frame(<RoadMarking>[
            sighting(RoadMarkingType.stopLine, 20, confidence: 0.8,
                frameId: i),
          ], id: i),
          travelledMeters: 0,
        );
      }
      expect(t.confirmed, isNotEmpty);
      expect(t.advisorySpeedMps(14), isNull,
          reason: 'what governs a junction is the sign or signal, not paint');
    });
  });

  group('scoring its own claims', () {
    test('a bump that was felt is recorded as confirmed', () {
      final RoadMarkingTracker t = RoadMarkingTracker();
      double d = 14;
      for (int i = 0; i < 4; i++) {
        t.update(
          result: frame(<RoadMarking>[
            sighting(RoadMarkingType.speedBump, d, confidence: 0.7,
                frameId: i),
          ], id: i),
          travelledMeters: i == 0 ? 0 : 4,
        );
        d -= 4;
      }
      expect(t.bumpScore.$1, 1, reason: 'one bump committed to');

      // Roll over it, with the jolt the detector implicitly predicted.
      t.update(
          result: frame(const <RoadMarking>[], id: 5), travelledMeters: 2,
          verticalAccelMps2: 3.1);
      t.update(
          result: frame(const <RoadMarking>[], id: 6), travelledMeters: 4,
          verticalAccelMps2: 0.2);
      expect(t.bumpScore.$2, 1, reason: 'and it was actually felt');
    });

    test('a bump that was not felt is counted as predicted but not confirmed',
        () {
      final RoadMarkingTracker t = RoadMarkingTracker();
      double d = 14;
      for (int i = 0; i < 4; i++) {
        t.update(
          result: frame(<RoadMarking>[
            sighting(RoadMarkingType.speedBump, d, confidence: 0.7,
                frameId: i),
          ], id: i),
          travelledMeters: i == 0 ? 0 : 4,
        );
        d -= 4;
      }
      t.update(
          result: frame(const <RoadMarking>[], id: 5), travelledMeters: 2,
          verticalAccelMps2: 0.1);
      t.update(
          result: frame(const <RoadMarking>[], id: 6), travelledMeters: 4,
          verticalAccelMps2: 0.1);
      expect(t.bumpScore, (1, 0));
    });
  });

  test('a degraded frame neither adds nor erodes evidence', () {
    final RoadMarkingTracker t = RoadMarkingTracker();
    for (int i = 0; i < 4; i++) {
      t.update(
        result: frame(<RoadMarking>[
          sighting(RoadMarkingType.crosswalk, 22, frameId: i),
        ], id: i),
        travelledMeters: 0,
      );
    }
    final double before = t.all.single.confidence.value;
    t.update(
      result: RoadMarkingResult.unavailable(
        frameId: 9,
        timestampMicros: 0,
        reason: 'camera shows too little road surface',
      ),
      travelledMeters: 0,
    );
    expect(t.all.single.confidence.value, closeTo(before, 1e-9));
  });
}
