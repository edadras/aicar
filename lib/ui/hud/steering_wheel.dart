import 'dart:math' as math;

import 'package:flutter/material.dart';

import '../theme.dart';

/// Graphical steering wheel that mirrors the simulated steering command.
///
/// The wheel rotates by the *steering-wheel* angle — the road-wheel angle
/// multiplied by the steering ratio — because that is the motion a driver
/// recognises. A road wheel at 8° barely looks turned; the 120° of wheel it
/// corresponds to reads instantly.
class SteeringWheelIndicator extends StatelessWidget {
  const SteeringWheelIndicator({
    super.key,
    required this.roadWheelDegrees,
    this.steeringRatio = 15.0,
    this.limitDegrees = 35.0,
    this.size = 92,
    this.isActive = true,
  });

  /// Road-wheel angle, positive = right.
  final double roadWheelDegrees;

  final double steeringRatio;
  final double limitDegrees;
  final double size;

  /// When false the wheel is drawn greyed out — the stack is not proposing a
  /// direction, which is different from proposing "straight ahead".
  final bool isActive;

  @override
  Widget build(BuildContext context) {
    final double wheelDegrees = roadWheelDegrees * steeringRatio;
    return SizedBox(
      width: size,
      height: size,
      child: CustomPaint(
        painter: _SteeringWheelPainter(
          wheelRotationDegrees: wheelDegrees,
          roadWheelDegrees: roadWheelDegrees,
          limitDegrees: limitDegrees,
          isActive: isActive,
        ),
      ),
    );
  }
}

class _SteeringWheelPainter extends CustomPainter {
  const _SteeringWheelPainter({
    required this.wheelRotationDegrees,
    required this.roadWheelDegrees,
    required this.limitDegrees,
    required this.isActive,
  });

  final double wheelRotationDegrees;
  final double roadWheelDegrees;
  final double limitDegrees;
  final bool isActive;

  @override
  void paint(Canvas canvas, Size size) {
    final Offset centre = Offset(size.width / 2, size.height / 2);
    final double radius = math.min(size.width, size.height) / 2 - 6;

    final Color rimColor = isActive
        ? (roadWheelDegrees.abs() > limitDegrees * 0.8
            ? HudTheme.warning
            : HudTheme.accent)
        : HudTheme.textDim;

    // Travel arc: how much of the available lock is being used.
    final double fraction =
        (roadWheelDegrees / limitDegrees).clamp(-1.0, 1.0);
    canvas.drawArc(
      Rect.fromCircle(center: centre, radius: radius + 4),
      -math.pi / 2,
      fraction * math.pi * 0.8,
      false,
      Paint()
        ..style = PaintingStyle.stroke
        ..strokeWidth = 3
        ..strokeCap = StrokeCap.round
        ..color = rimColor.withValues(alpha: 0.7),
    );

    canvas.save();
    canvas.translate(centre.dx, centre.dy);
    canvas.rotate(wheelRotationDegrees * math.pi / 180);

    final Paint rim = Paint()
      ..style = PaintingStyle.stroke
      ..strokeWidth = 5
      ..color = rimColor;
    canvas.drawCircle(Offset.zero, radius, rim);

    // Three spokes, like most real wheels: two at roughly 9 and 3 o'clock and
    // one below, which makes the rotation unambiguous at a glance.
    final Paint spoke = Paint()
      ..style = PaintingStyle.stroke
      ..strokeWidth = 4
      ..strokeCap = StrokeCap.round
      ..color = rimColor;

    for (final double angle in <double>[math.pi, 0, math.pi / 2]) {
      canvas.drawLine(
        Offset(math.cos(angle) * radius * 0.28,
            math.sin(angle) * radius * 0.28),
        Offset(math.cos(angle) * radius * 0.92,
            math.sin(angle) * radius * 0.92),
        spoke,
      );
    }

    canvas.drawCircle(
      Offset.zero,
      radius * 0.26,
      Paint()..color = HudTheme.surfaceRaised,
    );
    canvas.drawCircle(
      Offset.zero,
      radius * 0.26,
      Paint()
        ..style = PaintingStyle.stroke
        ..strokeWidth = 2
        ..color = rimColor,
    );

    // Top marker so the wheel's zero position stays readable when rotated.
    canvas.drawLine(
      Offset(0, -radius),
      Offset(0, -radius * 0.7),
      Paint()
        ..strokeWidth = 4
        ..strokeCap = StrokeCap.round
        ..color = HudTheme.textPrimary,
    );

    canvas.restore();

    // Fixed reference mark at true centre.
    canvas.drawLine(
      Offset(centre.dx, centre.dy - radius - 8),
      Offset(centre.dx, centre.dy - radius - 2),
      Paint()
        ..strokeWidth = 2
        ..color = HudTheme.textSecondary,
    );
  }

  @override
  bool shouldRepaint(_SteeringWheelPainter old) =>
      old.wheelRotationDegrees != wheelRotationDegrees ||
      old.isActive != isActive;
}

/// Horizontal bar for throttle or brake.
class PedalBar extends StatelessWidget {
  const PedalBar({
    super.key,
    required this.label,
    required this.percent,
    required this.color,
    this.emphasised = false,
  });

  final String label;
  final double percent;
  final Color color;

  /// Used for a full-authority brake, which should be impossible to miss.
  final bool emphasised;

  @override
  Widget build(BuildContext context) {
    final double fraction = (percent / 100).clamp(0.0, 1.0);
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      mainAxisSize: MainAxisSize.min,
      children: <Widget>[
        Row(
          mainAxisAlignment: MainAxisAlignment.spaceBetween,
          children: <Widget>[
            Text(label.toUpperCase(), style: HudTheme.hudLabel),
            Text(
              '${percent.round()}%',
              style: HudTheme.hudValue.copyWith(
                fontSize: 18,
                color: fraction > 0.02 ? color : HudTheme.textDim,
              ),
            ),
          ],
        ),
        const SizedBox(height: 5),
        ClipRRect(
          borderRadius: BorderRadius.circular(3),
          child: Stack(
            children: <Widget>[
              Container(height: 6, color: HudTheme.outline),
              FractionallySizedBox(
                widthFactor: fraction,
                child: Container(
                  height: 6,
                  decoration: BoxDecoration(
                    color: color,
                    boxShadow: emphasised && fraction > 0.9
                        ? <BoxShadow>[
                            BoxShadow(
                              color: color.withValues(alpha: 0.8),
                              blurRadius: 8,
                            ),
                          ]
                        : null,
                  ),
                ),
              ),
            ],
          ),
        ),
      ],
    );
  }
}
