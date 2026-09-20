import 'dart:math' as math;
import 'dart:typed_data';

/// A small stroke-based digit renderer used to test the speed-limit reader.
///
/// It is deliberately a *different* construction from the reader's bitmap
/// templates: digits are described as polylines and arcs in a unit box and
/// rasterised with a configurable stroke weight. A test that rendered from the
/// same bitmaps the matcher compares against would only prove the matcher can
/// recognise itself.
class VectorDigitRenderer {
  const VectorDigitRenderer();

  /// Render [value] into a grayscale image of the given size.
  Uint8List render(
    int value, {
    required int width,
    required int height,
    double strokeFraction = 0.16,
    int background = 240,
    int ink = 25,
    int noise = 0,
    int seed = 17,
    double slantDegrees = 0,
  }) {
    final String text = '$value';
    final Uint8List out = Uint8List(width * height)
      ..fillRange(0, width * height, background);

    final double marginY = height * 0.10;
    final double glyphHeight = height - 2 * marginY;
    final double gap = width * 0.06;
    final double glyphWidth =
        (width - gap * (text.length + 1)) / text.length;
    if (glyphWidth < 5 || glyphHeight < 8) return out;

    final double stroke = math.max(1.5, glyphWidth * strokeFraction);
    final double slant = math.tan(slantDegrees * math.pi / 180);

    double x = gap;
    for (int i = 0; i < text.length; i++) {
      _rasterise(
        out,
        width,
        height,
        _strokesFor(int.parse(text[i])),
        originX: x,
        originY: marginY,
        boxWidth: glyphWidth,
        boxHeight: glyphHeight,
        stroke: stroke,
        ink: ink,
        slant: slant,
      );
      x += glyphWidth + gap;
    }

    if (noise > 0) {
      final math.Random rng = math.Random(seed);
      for (int i = 0; i < out.length; i++) {
        out[i] = (out[i] + rng.nextInt(noise * 2 + 1) - noise).clamp(0, 255);
      }
    }
    return out;
  }

  /// Each digit as a list of polylines in a unit box (0..1, y down).
  /// Curves are approximated by short segments, which is all the rasteriser
  /// needs.
  List<List<_P>> _strokesFor(int digit) => switch (digit) {
        0 => <List<_P>>[_ellipse(0.5, 0.5, 0.36, 0.46)],
        1 => <List<_P>>[
            <_P>[const _P(0.22, 0.20), const _P(0.50, 0.03)],
            <_P>[const _P(0.50, 0.03), const _P(0.50, 0.97)],
            <_P>[const _P(0.24, 0.97), const _P(0.78, 0.97)],
          ],
        2 => <List<_P>>[
            _arc(0.5, 0.26, 0.34, 0.23, startDeg: 190, endDeg: 380),
            <_P>[const _P(0.86, 0.32), const _P(0.14, 0.95)],
            <_P>[const _P(0.12, 0.95), const _P(0.90, 0.95)],
          ],
        3 => <List<_P>>[
            _arc(0.50, 0.26, 0.33, 0.23, startDeg: 190, endDeg: 430),
            _arc(0.48, 0.73, 0.36, 0.24, startDeg: -60, endDeg: 200),
          ],
        4 => <List<_P>>[
            <_P>[const _P(0.72, 0.03), const _P(0.10, 0.70)],
            <_P>[const _P(0.10, 0.70), const _P(0.92, 0.70)],
            <_P>[const _P(0.72, 0.03), const _P(0.72, 0.97)],
          ],
        5 => <List<_P>>[
            <_P>[const _P(0.86, 0.05), const _P(0.20, 0.05)],
            <_P>[const _P(0.20, 0.05), const _P(0.16, 0.45)],
            _arc(0.48, 0.70, 0.38, 0.27, startDeg: -80, endDeg: 150),
          ],
        6 => <List<_P>>[
            _arc(0.52, 0.30, 0.38, 0.28, startDeg: 200, endDeg: 330),
            <_P>[const _P(0.14, 0.32), const _P(0.14, 0.70)],
            _ellipse(0.50, 0.71, 0.36, 0.26),
          ],
        7 => <List<_P>>[
            <_P>[const _P(0.10, 0.05), const _P(0.90, 0.05)],
            <_P>[const _P(0.90, 0.05), const _P(0.36, 0.97)],
          ],
        8 => <List<_P>>[
            _ellipse(0.50, 0.27, 0.32, 0.24),
            _ellipse(0.50, 0.74, 0.38, 0.24),
          ],
        9 => <List<_P>>[
            _ellipse(0.50, 0.29, 0.36, 0.26),
            <_P>[const _P(0.86, 0.30), const _P(0.86, 0.68)],
            _arc(0.50, 0.70, 0.36, 0.27, startDeg: 20, endDeg: 160),
          ],
        _ => <List<_P>>[],
      };

  static List<_P> _ellipse(double cx, double cy, double rx, double ry) =>
      _arc(cx, cy, rx, ry, startDeg: 0, endDeg: 360);

  static List<_P> _arc(
    double cx,
    double cy,
    double rx,
    double ry, {
    required double startDeg,
    required double endDeg,
  }) {
    final List<_P> points = <_P>[];
    const int steps = 36;
    for (int i = 0; i <= steps; i++) {
      final double t = startDeg + (endDeg - startDeg) * i / steps;
      final double r = t * math.pi / 180;
      points.add(_P(cx + rx * math.cos(r), cy + ry * math.sin(r)));
    }
    return points;
  }

  void _rasterise(
    Uint8List buffer,
    int bufWidth,
    int bufHeight,
    List<List<_P>> strokes, {
    required double originX,
    required double originY,
    required double boxWidth,
    required double boxHeight,
    required double stroke,
    required int ink,
    required double slant,
  }) {
    final double half = stroke / 2;
    final int x0 = originX.floor().clamp(0, bufWidth - 1);
    final int x1 = (originX + boxWidth + stroke).ceil().clamp(0, bufWidth);
    final int y0 = originY.floor().clamp(0, bufHeight - 1);
    final int y1 = (originY + boxHeight + stroke).ceil().clamp(0, bufHeight);

    for (int py = y0; py < y1; py++) {
      for (int px = x0; px < x1; px++) {
        // Undo the slant before testing, so italic text still rasterises from
        // the same unit-box description.
        final double localY = (py + 0.5 - originY);
        final double localX =
            (px + 0.5 - originX) - slant * (boxHeight - localY);
        final double ux = localX / boxWidth;
        final double uy = localY / boxHeight;

        bool hit = false;
        for (final List<_P> polyline in strokes) {
          for (int i = 0; i + 1 < polyline.length; i++) {
            final double d = _distanceToSegment(
              ux * boxWidth,
              uy * boxHeight,
              polyline[i].x * boxWidth,
              polyline[i].y * boxHeight,
              polyline[i + 1].x * boxWidth,
              polyline[i + 1].y * boxHeight,
            );
            if (d <= half) {
              hit = true;
              break;
            }
          }
          if (hit) break;
        }
        if (hit) buffer[py * bufWidth + px] = ink;
      }
    }
  }

  static double _distanceToSegment(
    double px,
    double py,
    double ax,
    double ay,
    double bx,
    double by,
  ) {
    final double dx = bx - ax;
    final double dy = by - ay;
    final double lengthSquared = dx * dx + dy * dy;
    if (lengthSquared < 1e-9) {
      return math.sqrt((px - ax) * (px - ax) + (py - ay) * (py - ay));
    }
    double t = ((px - ax) * dx + (py - ay) * dy) / lengthSquared;
    t = t.clamp(0.0, 1.0);
    final double cx = ax + t * dx;
    final double cy = ay + t * dy;
    return math.sqrt((px - cx) * (px - cx) + (py - cy) * (py - cy));
  }
}

class _P {
  const _P(this.x, this.y);
  final double x;
  final double y;
}
