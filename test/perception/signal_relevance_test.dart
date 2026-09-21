import 'package:aicar/perception/signal_relevance.dart';
import 'package:flutter_test/flutter_test.dart';

/// Whether a signal head governs *us*.
///
/// The failure this guards against is specific and dangerous: attributing the
/// cross street's red light to our own lane, and braking in the middle of a
/// junction.
void main() {
  const SignalRelevanceResolver resolver = SignalRelevanceResolver();

  RelevanceEvidence head({
    required double lateral,
    required double distance,
    double pathLateral = 0,
    double? aspect = 0.33,
    double? drift,
    double? junction,
    double junctionConfidence = 0,
    double boxHeight = 24,
  }) =>
      RelevanceEvidence(
        lateralOffsetMeters: lateral,
        distanceMeters: distance,
        pathLateralAtDistance: pathLateral,
        egoLaneHalfWidth: 1.75,
        boxHeightPixels: boxHeight,
        aspectRatio: aspect,
        lateralDriftPerMetre: drift,
        junctionDistanceMeters: junction,
        junctionConfidence: junctionConfidence,
      );

  group('position alone', () {
    test('a head over our lane is ours', () {
      final RelevanceVerdict v =
          resolver.resolve(head(lateral: 0.3, distance: 25));
      expect(v.governsEgo, isTrue);
      expect(v.confidence, greaterThan(0.4));
    });

    test('a head far across the road is not', () {
      final RelevanceVerdict v =
          resolver.resolve(head(lateral: 16, distance: 25));
      expect(v.governsEgo, isFalse);
    });

    test('a head just outside the tolerance is unknown, not guessed', () {
      final RelevanceVerdict v =
          resolver.resolve(head(lateral: 6.5, distance: 25));
      expect(v.governsEgo, isNull);
      expect(v.reasons.join(' '), contains('not far enough to rule out'));
    });

    test('tolerance widens with distance', () {
      // The same lateral offset is more forgivable further away, because both
      // the range estimate and the path prediction are looser out there.
      expect(resolver.resolve(head(lateral: 5.0, distance: 60)).governsEgo,
          isTrue);
      expect(resolver.resolve(head(lateral: 5.0, distance: 10)).governsEgo,
          isNot(isTrue));
    });
  });

  group('junction agreement', () {
    test('a head at the junction we are approaching is strongly ours', () {
      final RelevanceVerdict near = resolver.resolve(head(
        lateral: 1.0,
        distance: 30,
        junction: 28,
        junctionConfidence: 0.8,
      ));
      final RelevanceVerdict blind =
          resolver.resolve(head(lateral: 1.0, distance: 30));
      expect(near.governsEgo, isTrue);
      expect(near.confidence, greaterThan(blind.confidence));
      expect(near.reasons.join(' '), contains('matches the junction'));
    });

    test('a head nowhere near the junction governs something else', () {
      // This is the gantry case: almost over our path, but belonging to a
      // junction 70 m further on.
      final RelevanceVerdict v = resolver.resolve(head(
        lateral: 0.5,
        distance: 90,
        junction: 25,
        junctionConfidence: 0.85,
      ));
      expect(v.governsEgo, isNot(isTrue));
      expect(v.reasons.join(' '), contains('governs something else'));
    });

    test('a junction we are unsure of does not move the verdict', () {
      final RelevanceVerdict weak = resolver.resolve(head(
        lateral: 1.0,
        distance: 90,
        junction: 25,
        junctionConfidence: 0.2,
      ));
      final RelevanceVerdict none =
          resolver.resolve(head(lateral: 1.0, distance: 90));
      expect(weak.confidence, closeTo(none.confidence, 0.001));
    });
  });

  group('foreshortening', () {
    test('an edge-on housing is aimed at someone else', () {
      final RelevanceVerdict v = resolver.resolve(head(
        lateral: 1.0,
        distance: 25,
        aspect: 0.11,
      ));
      expect(v.governsEgo, isNot(isTrue));
      expect(v.reasons.join(' '), contains('edge-on'));
    });

    test('a normal vertical housing is not penalised', () {
      expect(
        resolver.resolve(head(lateral: 1.0, distance: 25, aspect: 0.33))
            .governsEgo,
        isTrue,
      );
    });

    test('an unknown orientation is not guessed at', () {
      // A horizontally mounted head has an entirely different signature;
      // applying the vertical rule to it would misread every one.
      expect(
        resolver.resolve(head(lateral: 1.0, distance: 25, aspect: null))
            .governsEgo,
        isTrue,
      );
      expect(
        resolver.resolve(head(lateral: 1.0, distance: 25, aspect: 2.9))
            .governsEgo,
        isTrue,
      );
    });
  });

  group('lateral drift', () {
    test('a head sweeping across our path as we approach is not ours', () {
      final RelevanceVerdict v =
          resolver.resolve(head(lateral: 1.0, distance: 25, drift: 0.3));
      expect(v.governsEgo, isNot(isTrue));
      expect(v.reasons.join(' '), contains('sliding across our path'));
    });

    test('a head that stays put over our lane is ours', () {
      expect(
        resolver.resolve(head(lateral: 1.0, distance: 25, drift: 0.01))
            .governsEgo,
        isTrue,
      );
    });

    test('drift plus an ambiguous position settles it', () {
      final RelevanceVerdict v =
          resolver.resolve(head(lateral: 6.5, distance: 25, drift: 0.25));
      expect(v.governsEgo, isFalse,
          reason: 'position alone was unknown; drift decides');
    });
  });

  group('the verdict explains itself', () {
    test('every verdict carries its reasoning', () {
      for (final RelevanceEvidence e in <RelevanceEvidence>[
        head(lateral: 0.3, distance: 25),
        head(lateral: 16, distance: 25),
        head(lateral: 6.5, distance: 25),
        head(lateral: 1.0, distance: 25, aspect: 0.11),
      ]) {
        expect(resolver.resolve(e).reasons, isNotEmpty);
      }
    });
  });
}
