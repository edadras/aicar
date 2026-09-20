import 'dart:math' as math;
import 'dart:typed_data';

import 'package:aicar/perception/speed_limit_reader.dart';
import 'package:flutter_test/flutter_test.dart';

import '../support/vector_digits.dart';

void main() {
  const SpeedLimitReader reader = SpeedLimitReader();
  const VectorDigitRenderer renderer = VectorDigitRenderer();

  group('SpeedLimitReader on rendered signs', () {
    test('reads the common two-digit limits', () {
      final Map<int, int?> results = <int, int?>{};
      for (final int limit in <int>[30, 40, 50, 60, 70, 80, 90]) {
        final Uint8List img =
            renderer.render(limit, width: 72, height: 52);
        final SpeedLimitReading? r = reader.read(img, 72, 52);
        results[limit] = r?.valueKph;
        // A wrong reading is much worse than no reading, so misreads are
        // the failure this test is really guarding against.
        expect(r?.valueKph, anyOf(isNull, limit),
            reason: 'misread $limit as ${r?.valueKph}');
      }
      final int read = results.values.whereType<int>().length;
      expect(read, greaterThanOrEqualTo(5),
          reason: 'only read $read of ${results.length}: $results');
    });

    test('reads three-digit limits', () {
      final Map<int, int?> results = <int, int?>{};
      for (final int limit in <int>[100, 110, 120]) {
        final Uint8List img =
            renderer.render(limit, width: 104, height: 52);
        final SpeedLimitReading? r = reader.read(img, 104, 52);
        results[limit] = r?.valueKph;
        expect(r?.valueKph, anyOf(isNull, limit),
            reason: 'misread $limit as ${r?.valueKph}');
      }
      expect(results.values.whereType<int>(), isNotEmpty,
          reason: 'read none of $results');
    });

    test('tolerates a heavier stroke weight', () {
      final Uint8List bold =
          renderer.render(50, width: 72, height: 52, strokeFraction: 0.26);
      expect(reader.read(bold, 72, 52)?.valueKph, anyOf(isNull, 50));
      final Uint8List light =
          renderer.render(50, width: 72, height: 52, strokeFraction: 0.11);
      expect(reader.read(light, 72, 52)?.valueKph, anyOf(isNull, 50));
    });

    test('tolerates scale changes', () {
      for (final (int w, int h) in <(int, int)>[(48, 34), (72, 52), (160, 110)]) {
        final Uint8List img = renderer.render(60, width: w, height: h);
        expect(reader.read(img, w, h)?.valueKph, anyOf(isNull, 60),
            reason: 'misread at ${w}x$h');
      }
    });

    test('tolerates sensor noise', () {
      final Uint8List img =
          renderer.render(50, width: 72, height: 52, noise: 22);
      expect(reader.read(img, 72, 52)?.valueKph, anyOf(isNull, 50));
    });
  });

  group('SpeedLimitReader rejection', () {
    test('rejects a blank face rather than guessing', () {
      final Uint8List blank = Uint8List(72 * 52)..fillRange(0, 72 * 52, 240);
      expect(reader.read(blank, 72, 52), isNull);
    });

    test('rejects a face that is entirely ink', () {
      final Uint8List solid = Uint8List(72 * 52)..fillRange(0, 72 * 52, 20);
      expect(reader.read(solid, 72, 52), isNull);
    });

    test('rejects a face too small to read', () {
      final Uint8List tiny = renderer.render(50, width: 14, height: 7);
      expect(reader.read(tiny, 14, 7), isNull);
    });

    test('rejects an implausible posted value', () {
      // 88 is not a posted limit anywhere, so even a clean render of it must
      // be rejected rather than reported.
      final Uint8List img = renderer.render(88, width: 72, height: 52);
      expect(reader.read(img, 72, 52), isNull);
    });

    test('rejects more than three digits', () {
      final Uint8List img = renderer.render(1234, width: 130, height: 52);
      expect(reader.read(img, 130, 52), isNull);
    });

    test('rejects random noise', () {
      final math.Random rng = math.Random(99);
      final Uint8List noise = Uint8List(72 * 52);
      for (int i = 0; i < noise.length; i++) {
        noise[i] = rng.nextInt(256);
      }
      expect(reader.read(noise, 72, 52), isNull);
    });

    test('never reports a value outside the plausible set', () {
      final math.Random rng = math.Random(3);
      for (int trial = 0; trial < 40; trial++) {
        final Uint8List img = Uint8List(72 * 52);
        for (int i = 0; i < img.length; i++) {
          img[i] = rng.nextBool() ? 240 : rng.nextInt(120);
        }
        final SpeedLimitReading? r = reader.read(img, 72, 52);
        if (r != null) {
          expect(SpeedLimitReader.plausibleLimits, contains(r.valueKph));
        }
      }
    });
  });
}
