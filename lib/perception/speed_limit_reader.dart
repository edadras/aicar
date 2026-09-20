import 'dart:math' as math;
import 'dart:typed_data';

import '../core/geometry.dart';

/// Reads the number from a speed-limit sign.
///
/// This is the classical fallback used when no sign-classification model is
/// installed. It is deliberately conservative: it segments digits, matches
/// them against embedded glyph templates, and reports a confidence that
/// reflects how weak template matching on a motion-blurred 30-pixel sign
/// really is. Callers must not act on a single reading — see
/// [TrafficSign.isSpeedLimitTrustworthy], which requires three consistent
/// observations at 70% confidence before the value influences anything.
///
/// Where a trained classifier is available it is used in preference; this
/// exists so the feature is not simply absent on a device with no models.
class SpeedLimitReader {
  const SpeedLimitReader({
    this.glyphWidth = 8,
    this.glyphHeight = 12,
    this.minDigitHeightPixels = 8,
    this.minMatchScore = 0.74,
  });

  final int glyphWidth;
  final int glyphHeight;

  /// Below this a digit is too small to read; reporting a guess would be
  /// worse than reporting nothing.
  final int minDigitHeightPixels;

  /// Minimum correlation score, on the 0..1 scale [_agreement] returns. An
  /// uncorrelated glyph scores 0.5, so anything at or below that is noise.
  final double minMatchScore;

  /// Plausible posted limits. Anything outside this set is a misread, and
  /// rejecting it removes most of the failure modes of template matching.
  static const Set<int> plausibleLimits = <int>{
    5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90,
    95, 100, 110, 120, 130,
  };

  /// Per-digit detail from the last read, for the developer debug screen and
  /// for diagnosing misreads on recorded sessions.
  SpeedLimitDebug? debugRead(Uint8List gray, int width, int height) {
    if (height < minDigitHeightPixels || width < minDigitHeightPixels) {
      return const SpeedLimitDebug(
          rejectedBecause: 'face smaller than the minimum readable size');
    }
    final int threshold = _otsu(gray);
    final Uint8List ink = Uint8List(width * height);
    int inkCount = 0;
    for (int i = 0; i < gray.length; i++) {
      if (gray[i] < threshold) {
        ink[i] = 1;
        inkCount++;
      }
    }
    final double inkRatio = inkCount / (width * height);
    if (inkRatio < 0.06 || inkRatio > 0.62) {
      return SpeedLimitDebug(
        inkRatio: inkRatio,
        rejectedBecause:
            'ink ratio ${inkRatio.toStringAsFixed(2)} outside 0.06..0.62',
      );
    }

    final List<_DigitBox> boxes = _segmentDigits(ink, width, height);
    if (boxes.isEmpty) {
      return SpeedLimitDebug(
          inkRatio: inkRatio, rejectedBecause: 'no digit-shaped column runs');
    }
    if (boxes.length > 3) {
      return SpeedLimitDebug(
        inkRatio: inkRatio,
        rejectedBecause: '${boxes.length} segments (max 3)',
      );
    }

    final List<int> digits = <int>[];
    final List<double> scores = <double>[];
    for (final _DigitBox b in boxes) {
      final (int digit, double score) = _matchGlyph(_normalize(ink, width, b));
      digits.add(digit);
      scores.add(score);
    }
    final int? value = int.tryParse(digits.join());
    return SpeedLimitDebug(
      inkRatio: inkRatio,
      digits: digits,
      scores: scores,
      parsedValue: value,
      rejectedBecause: scores.any((double s) => s < minMatchScore)
          ? 'glyph score below $minMatchScore'
          : (value == null || !plausibleLimits.contains(value)
              ? 'value $value is not a posted limit'
              : null),
    );
  }

  /// Read the digits from a grayscale crop of a sign face.
  ///
  /// [gray] is the sign's interior, already cropped inside the red annulus.
  /// Returns `null` when nothing legible is found.
  SpeedLimitReading? read(Uint8List gray, int width, int height) {
    if (height < minDigitHeightPixels || width < minDigitHeightPixels) {
      return null;
    }

    // Speed-limit signs are dark digits on a white field. Otsu splits them
    // without needing a fixed threshold, which matters across day/night and
    // retro-reflective faces.
    final int threshold = _otsu(gray);
    final Uint8List ink = Uint8List(width * height);
    int inkCount = 0;
    for (int i = 0; i < gray.length; i++) {
      if (gray[i] < threshold) {
        ink[i] = 1;
        inkCount++;
      }
    }

    // A face that is almost entirely ink or almost entirely blank is not a
    // numeral panel.
    final double inkRatio = inkCount / (width * height);
    if (inkRatio < 0.06 || inkRatio > 0.62) return null;

    final List<_DigitBox> boxes = _segmentDigits(ink, width, height);
    if (boxes.isEmpty || boxes.length > 3) return null;

    final StringBuffer digits = StringBuffer();
    double scoreSum = 0;
    for (final _DigitBox b in boxes) {
      final Float32List glyph = _normalize(ink, width, b);
      final (int digit, double score) = _matchGlyph(glyph);
      if (score < minMatchScore) return null;
      digits.write(digit);
      scoreSum += score;
    }

    final int? value = int.tryParse(digits.toString());
    if (value == null || value <= 0) return null;
    if (!plausibleLimits.contains(value)) return null;

    final double meanScore = scoreSum / boxes.length;
    // Fewer digits means less evidence that the segmentation was right, and a
    // small face means less evidence full stop.
    final double sizeQuality =
        clampDouble((height - minDigitHeightPixels) / 20.0, 0, 1);
    final double confidence = clampDouble(
      (meanScore - minMatchScore) / (1 - minMatchScore) * 0.7 +
          0.3 * sizeQuality,
      0,
      0.92,
    );

    return SpeedLimitReading(
      valueKph: value,
      confidence: confidence,
      digitCount: boxes.length,
      meanGlyphScore: meanScore,
    );
  }

  /// Split the ink into digit-sized connected column runs.
  List<_DigitBox> _segmentDigits(Uint8List ink, int width, int height) {
    final Int32List columnInk = Int32List(width);
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        columnInk[x] += ink[y * width + x];
      }
    }

    final List<_DigitBox> boxes = <_DigitBox>[];
    int runStart = -1;
    for (int x = 0; x <= width; x++) {
      final bool hasInk = x < width && columnInk[x] > 0;
      if (hasInk && runStart < 0) {
        runStart = x;
      } else if (!hasInk && runStart >= 0) {
        final int runWidth = x - runStart;
        if (runWidth >= 2) {
          final _DigitBox? box = _boundRun(ink, width, height, runStart, x);
          if (box != null) boxes.add(box);
        }
        runStart = -1;
      }
    }

    // Reject anything that is not digit-shaped: too flat, too tall, or too
    // small a fraction of the face.
    boxes.removeWhere((_DigitBox b) {
      final double aspect = b.width / b.height;
      return b.height < minDigitHeightPixels ||
          b.height < height * 0.35 ||
          aspect > 1.1 ||
          aspect < 0.12;
    });

    boxes.sort((_DigitBox a, _DigitBox b) => a.x0.compareTo(b.x0));
    return boxes;
  }

  _DigitBox? _boundRun(
      Uint8List ink, int width, int height, int x0, int x1) {
    int minY = height;
    int maxY = -1;
    for (int y = 0; y < height; y++) {
      for (int x = x0; x < x1; x++) {
        if (ink[y * width + x] == 1) {
          if (y < minY) minY = y;
          if (y > maxY) maxY = y;
        }
      }
    }
    if (maxY < minY) return null;
    return _DigitBox(x0: x0, y0: minY, x1: x1, y1: maxY + 1);
  }

  /// Resample a digit into the template grid as **ink density per cell**.
  ///
  /// Binarising each cell (the obvious approach) makes the match extremely
  /// sensitive to stroke weight: the same '5' set in a light and a bold face
  /// produce different binary grids and score as different digits. Keeping
  /// the coverage fraction instead means a heavier stroke raises every cell it
  /// touches proportionally, which the correlation below is invariant to.
  Float32List _normalize(Uint8List ink, int width, _DigitBox box) {
    final Float32List out = Float32List(glyphWidth * glyphHeight);
    final double sx = box.width / glyphWidth;
    final double sy = box.height / glyphHeight;

    for (int gy = 0; gy < glyphHeight; gy++) {
      final int y0 = box.y0 + (gy * sy).floor();
      final int y1 = math.max(y0 + 1, box.y0 + ((gy + 1) * sy).ceil());
      for (int gx = 0; gx < glyphWidth; gx++) {
        final int x0 = box.x0 + (gx * sx).floor();
        final int x1 = math.max(x0 + 1, box.x0 + ((gx + 1) * sx).ceil());

        int filled = 0;
        int total = 0;
        for (int y = y0; y < y1; y++) {
          for (int x = x0; x < x1; x++) {
            total++;
            if (ink[y * width + x] == 1) filled++;
          }
        }
        out[gy * glyphWidth + gx] = total > 0 ? filled / total : 0;
      }
    }
    return out;
  }

  /// Match against every template, allowing a one-cell shift in each
  /// direction to absorb segmentation jitter.
  (int, double) _matchGlyph(Float32List glyph) {
    int bestDigit = -1;
    double bestScore = -1;

    for (int digit = 0; digit <= 9; digit++) {
      final Float32List template = _blurredTemplates[digit];
      for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
          final double score = _agreement(glyph, template, dx, dy);
          if (score > bestScore) {
            bestScore = score;
            bestDigit = digit;
          }
        }
      }
    }
    return (bestDigit, bestScore);
  }

  /// Pearson correlation between the two density grids, mapped to 0..1.
  ///
  /// Correlation (rather than overlap) is what makes the match invariant to
  /// stroke weight and to how dark the ink is: it compares the *pattern* of
  /// where ink is concentrated, not how much of it there is.
  double _agreement(Float32List glyph, Float32List template, int dx, int dy) {
    final int n = glyphWidth * glyphHeight;
    double sumG = 0;
    double sumT = 0;
    double sumGG = 0;
    double sumTT = 0;
    double sumGT = 0;

    for (int y = 0; y < glyphHeight; y++) {
      final int ty = y + dy;
      for (int x = 0; x < glyphWidth; x++) {
        final int tx = x + dx;
        final double g = glyph[y * glyphWidth + x];
        final double t = (tx < 0 || ty < 0 || tx >= glyphWidth || ty >= glyphHeight)
            ? 0
            : template[ty * glyphWidth + tx];
        sumG += g;
        sumT += t;
        sumGG += g * g;
        sumTT += t * t;
        sumGT += g * t;
      }
    }

    final double covariance = sumGT - sumG * sumT / n;
    final double varG = sumGG - sumG * sumG / n;
    final double varT = sumTT - sumT * sumT / n;
    if (varG <= 1e-9 || varT <= 1e-9) return 0;

    final double r = covariance / math.sqrt(varG * varT);
    // Map [-1, 1] onto [0, 1]; an anti-correlated glyph is no match at all.
    return clampDouble((r + 1) / 2, 0, 1);
  }

  static int _otsu(Uint8List gray) {
    final Int32List hist = Int32List(256);
    for (final int v in gray) {
      hist[v]++;
    }
    final int total = gray.length;
    double sum = 0;
    for (int t = 0; t < 256; t++) {
      sum += t * hist[t];
    }
    double sumB = 0;
    int wB = 0;
    double maxVar = -1;
    // With a strongly bimodal histogram every threshold between the two peaks
    // is equally optimal, and taking the first one puts the threshold exactly
    // on the darker peak — which then classifies none of those pixels as ink.
    // Track the whole plateau and take its midpoint.
    int plateauStart = 128;
    int plateauEnd = 128;
    for (int t = 0; t < 256; t++) {
      wB += hist[t];
      if (wB == 0) continue;
      final int wF = total - wB;
      if (wF == 0) break;
      sumB += t * hist[t];
      final double mB = sumB / wB;
      final double mF = (sum - sumB) / wF;
      final double between = wB * wF * (mB - mF) * (mB - mF);
      if (between > maxVar) {
        maxVar = between;
        plateauStart = t;
        plateauEnd = t;
      } else if (between == maxVar) {
        plateauEnd = t;
      }
    }
    return (plateauStart + plateauEnd) ~/ 2;
  }

  /// 8x12 glyph templates for the digits, written as bitmaps so they can be
  /// read and adjusted by eye.
  ///
  /// They approximate the bold, closed-aperture typefaces used on European
  /// speed-limit signs. They are not an exact match for any one standard —
  /// which is precisely why a single reading is not trusted.
  static final List<Uint8List> _templates = <Uint8List>[
    _parse(const <String>[
      '..####..',
      '.##..##.',
      '##....##',
      '##....##',
      '##....##',
      '##....##',
      '##....##',
      '##....##',
      '##....##',
      '.##..##.',
      '..####..',
      '........',
    ]),
    _parse(const <String>[
      '...##...',
      '..###...',
      '.####...',
      '...##...',
      '...##...',
      '...##...',
      '...##...',
      '...##...',
      '...##...',
      '...##...',
      '.######.',
      '........',
    ]),
    _parse(const <String>[
      '.######.',
      '##....##',
      '......##',
      '......##',
      '.....##.',
      '....##..',
      '...##...',
      '..##....',
      '.##.....',
      '##......',
      '########',
      '........',
    ]),
    _parse(const <String>[
      '.######.',
      '##....##',
      '......##',
      '......##',
      '...####.',
      '......##',
      '......##',
      '......##',
      '##....##',
      '##....##',
      '.######.',
      '........',
    ]),
    _parse(const <String>[
      '.....##.',
      '....###.',
      '...####.',
      '..##.##.',
      '.##..##.',
      '##...##.',
      '########',
      '.....##.',
      '.....##.',
      '.....##.',
      '.....##.',
      '........',
    ]),
    _parse(const <String>[
      '########',
      '##......',
      '##......',
      '##......',
      '######..',
      '......##',
      '......##',
      '......##',
      '##....##',
      '##....##',
      '.#####..',
      '........',
    ]),
    _parse(const <String>[
      '..#####.',
      '.##....#',
      '##......',
      '##......',
      '##......',
      '######..',
      '##....##',
      '##....##',
      '##....##',
      '##....##',
      '.######.',
      '........',
    ]),
    _parse(const <String>[
      '########',
      '##....##',
      '......##',
      '.....##.',
      '.....##.',
      '....##..',
      '....##..',
      '...##...',
      '...##...',
      '..##....',
      '..##....',
      '........',
    ]),
    _parse(const <String>[
      '.######.',
      '##....##',
      '##....##',
      '##....##',
      '.######.',
      '##....##',
      '##....##',
      '##....##',
      '##....##',
      '##....##',
      '.######.',
      '........',
    ]),
    _parse(const <String>[
      '.######.',
      '##....##',
      '##....##',
      '##....##',
      '##....##',
      '..######',
      '......##',
      '......##',
      '.....##.',
      '.####...',
      '........',
      '........',
    ]),
  ];

  /// Templates as density grids with a light blur.
  ///
  /// Blurring is not cosmetic: it makes the correlation tolerant of a glyph
  /// whose strokes sit half a cell away from the template's, which is the
  /// normal case once a sign has been resampled from 26 pixels tall.
  static final List<Float32List> _blurredTemplates = <Float32List>[
    for (final Uint8List t in _templates) _blur(t),
  ];

  static Float32List _blur(Uint8List template) {
    const int w = 8;
    const int h = 12;
    final Float32List out = Float32List(w * h);
    for (int y = 0; y < h; y++) {
      for (int x = 0; x < w; x++) {
        double sum = 0;
        double weight = 0;
        for (int dy = -1; dy <= 1; dy++) {
          for (int dx = -1; dx <= 1; dx++) {
            final int nx = x + dx;
            final int ny = y + dy;
            if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
            // Centre weighted 4x, edges 2x, corners 1x: a 3x3 binomial kernel.
            final double k = (dx == 0 ? 2.0 : 1.0) * (dy == 0 ? 2.0 : 1.0);
            sum += k * template[ny * w + nx];
            weight += k;
          }
        }
        out[y * w + x] = weight <= 0 ? 0 : sum / weight;
      }
    }
    return out;
  }

  static Uint8List _parse(List<String> rows) {
    final Uint8List out = Uint8List(8 * 12);
    for (int y = 0; y < 12 && y < rows.length; y++) {
      final String row = rows[y];
      for (int x = 0; x < 8 && x < row.length; x++) {
        out[y * 8 + x] = row.codeUnitAt(x) == 0x23 ? 1 : 0; // '#'
      }
    }
    return out;
  }
}

class SpeedLimitReading {
  const SpeedLimitReading({
    required this.valueKph,
    required this.confidence,
    required this.digitCount,
    required this.meanGlyphScore,
  });

  final int valueKph;
  final double confidence;
  final int digitCount;
  final double meanGlyphScore;

  @override
  String toString() =>
      '$valueKph km/h (${(confidence * 100).round()}%, '
      '$digitCount digits, glyph ${meanGlyphScore.toStringAsFixed(2)})';
}

/// Diagnostic detail from [SpeedLimitReader.debugRead].
class SpeedLimitDebug {
  const SpeedLimitDebug({
    this.inkRatio = 0,
    this.digits = const <int>[],
    this.scores = const <double>[],
    this.parsedValue,
    this.rejectedBecause,
  });

  final double inkRatio;
  final List<int> digits;
  final List<double> scores;
  final int? parsedValue;
  final String? rejectedBecause;

  @override
  String toString() => 'SpeedLimitDebug(ink ${inkRatio.toStringAsFixed(2)}, '
      'digits $digits, scores '
      '${scores.map((double s) => s.toStringAsFixed(2)).toList()}, '
      'value $parsedValue'
      '${rejectedBecause == null ? '' : ', rejected: $rejectedBecause'})';
}

class _DigitBox {
  const _DigitBox({
    required this.x0,
    required this.y0,
    required this.x1,
    required this.y1,
  });

  final int x0;
  final int y0;
  final int x1;
  final int y1;

  int get width => x1 - x0;
  int get height => y1 - y0;
}
