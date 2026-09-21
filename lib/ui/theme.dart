import 'package:flutter/material.dart';

import '../tracking/object_track.dart';
import '../world_model/hazard.dart';

/// Colours and text styles for the HUD.
///
/// A driving HUD is read in a fraction of a second, in sunlight, at the edge
/// of vision. That drives every choice here: a dark base so the camera image
/// dominates, a small number of saturated accents each meaning exactly one
/// thing, and numerals in a monospaced face so a changing digit does not make
/// the whole readout jump.
class HudTheme {
  const HudTheme._();

  static const Color background = Color(0xFF07090C);
  static const Color surface = Color(0xFF12171D);
  static const Color surfaceRaised = Color(0xFF1B222B);
  static const Color outline = Color(0xFF2C3540);

  static const Color textPrimary = Color(0xFFEDF2F7);
  static const Color textSecondary = Color(0xFF93A1B0);
  static const Color textDim = Color(0xFF5C6875);

  /// The one "everything is fine" accent.
  static const Color accent = Color(0xFF3DDC97);

  /// Severity ramp. Deliberately only four steps: a driver cannot
  /// meaningfully distinguish more than that at a glance.
  static const Color info = Color(0xFF4FA8FF);
  static const Color caution = Color(0xFFFFC94D);
  static const Color warning = Color(0xFFFF8A3D);
  static const Color critical = Color(0xFFFF4D5E);

  /// Overlay colours, kept distinct from the severity ramp so a lane line is
  /// never mistaken for a warning.
  static const Color laneLine = Color(0xFF6FD3FF);
  static const Color laneLineAdjacent = Color(0xFF3F7F99);
  static const Color drivableArea = Color(0x332BE08B);
  static const Color plannedPath = Color(0xFF7CE6A6);
  static const Color plannedPathBlocked = Color(0xFFFF8A3D);
  static const Color roadEdge = Color(0xFFB98AFF);

  static Color forSeverity(HazardSeverity severity) => switch (severity) {
        HazardSeverity.info => info,
        HazardSeverity.caution => caution,
        HazardSeverity.warning => warning,
        HazardSeverity.critical => critical,
      };

  static Color forRisk(CollisionRisk risk) => switch (risk) {
        CollisionRisk.low => accent,
        CollisionRisk.medium => caution,
        CollisionRisk.high => warning,
        CollisionRisk.critical => critical,
      };

  /// Confidence ramp used for every confidence readout in the app, so the
  /// same colour always means the same level of trust.
  static Color forConfidence(double confidence) {
    if (confidence >= 0.75) return accent;
    if (confidence >= 0.45) return caution;
    if (confidence >= 0.2) return warning;
    return critical;
  }

  static const String monoFamily = 'monospace';

  static const TextStyle hudValue = TextStyle(
    fontFamily: monoFamily,
    fontSize: 30,
    fontWeight: FontWeight.w600,
    color: textPrimary,
    height: 1.0,
    letterSpacing: -0.5,
  );

  static const TextStyle hudValueLarge = TextStyle(
    fontFamily: monoFamily,
    fontSize: 44,
    fontWeight: FontWeight.w700,
    color: textPrimary,
    height: 1.0,
    letterSpacing: -1,
  );

  static const TextStyle hudLabel = TextStyle(
    fontSize: 11,
    fontWeight: FontWeight.w600,
    color: textSecondary,
    letterSpacing: 1.4,
  );

  static const TextStyle overlayLabel = TextStyle(
    fontFamily: monoFamily,
    fontSize: 11,
    fontWeight: FontWeight.w600,
    color: Colors.white,
    height: 1.25,
  );

  static const TextStyle body = TextStyle(
    fontSize: 14,
    color: textPrimary,
    height: 1.4,
  );

  static const TextStyle caption = TextStyle(
    fontSize: 12,
    color: textSecondary,
    height: 1.35,
  );

  static ThemeData theme() {
    final ColorScheme scheme = const ColorScheme.dark().copyWith(
      primary: accent,
      secondary: info,
      surface: surface,
      error: critical,
      onPrimary: background,
      onSurface: textPrimary,
    );

    return ThemeData(
      useMaterial3: true,
      brightness: Brightness.dark,
      colorScheme: scheme,
      scaffoldBackgroundColor: background,
      canvasColor: background,
      dividerColor: outline,
      appBarTheme: const AppBarTheme(
        backgroundColor: background,
        surfaceTintColor: Colors.transparent,
        elevation: 0,
        titleTextStyle: TextStyle(
          fontSize: 17,
          fontWeight: FontWeight.w600,
          color: textPrimary,
        ),
      ),
      cardTheme: CardThemeData(
        color: surface,
        surfaceTintColor: Colors.transparent,
        elevation: 0,
        margin: EdgeInsets.zero,
        shape: RoundedRectangleBorder(
          borderRadius: BorderRadius.circular(14),
          side: const BorderSide(color: outline),
        ),
      ),
      listTileTheme: const ListTileThemeData(
        iconColor: textSecondary,
        textColor: textPrimary,
      ),
      switchTheme: SwitchThemeData(
        thumbColor: WidgetStateProperty.resolveWith<Color>(
          (Set<WidgetState> states) => states.contains(WidgetState.selected)
              ? accent
              : textDim,
        ),
      ),
      sliderTheme: const SliderThemeData(
        activeTrackColor: accent,
        thumbColor: accent,
        inactiveTrackColor: outline,
      ),
      segmentedButtonTheme: SegmentedButtonThemeData(
        style: ButtonStyle(
          side: WidgetStateProperty.all(const BorderSide(color: outline)),
        ),
      ),
    );
  }
}

/// A labelled readout, used across the HUD and the data screens.
class HudReadout extends StatelessWidget {
  const HudReadout({
    super.key,
    required this.label,
    required this.value,
    this.unit,
    this.valueColor,
    this.large = false,
    this.width,
  });

  final String label;
  final String value;
  final String? unit;
  final Color? valueColor;
  final bool large;
  final double? width;

  @override
  Widget build(BuildContext context) {
    return SizedBox(
      width: width,
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        mainAxisSize: MainAxisSize.min,
        children: <Widget>[
          Text(label.toUpperCase(), style: HudTheme.hudLabel),
          const SizedBox(height: 4),
          Row(
            crossAxisAlignment: CrossAxisAlignment.baseline,
            textBaseline: TextBaseline.alphabetic,
            mainAxisSize: MainAxisSize.min,
            children: <Widget>[
              // Only flex when the caller gave a width. A readout placed
              // directly in an unbounded Row (which is how the hazard banner
              // and the performance screen use it) would otherwise ask a
              // Flexible child to fill infinite space and fail layout.
              if (width != null)
                Flexible(
                  child: Text(
                    value,
                    maxLines: 1,
                    overflow: TextOverflow.ellipsis,
                    style: (large ? HudTheme.hudValueLarge : HudTheme.hudValue)
                        .copyWith(color: valueColor ?? HudTheme.textPrimary),
                  ),
                )
              else
                Text(
                  value,
                  maxLines: 1,
                  overflow: TextOverflow.ellipsis,
                  style: (large ? HudTheme.hudValueLarge : HudTheme.hudValue)
                      .copyWith(color: valueColor ?? HudTheme.textPrimary),
                ),
              if (unit != null) ...<Widget>[
                const SizedBox(width: 3),
                Text(
                  unit!,
                  style: HudTheme.caption.copyWith(
                    color: HudTheme.textDim,
                    fontSize: 12,
                  ),
                ),
              ],
            ],
          ),
        ],
      ),
    );
  }
}

/// A small pill used for statuses and badges.
class HudBadge extends StatelessWidget {
  const HudBadge({
    super.key,
    required this.text,
    this.color = HudTheme.info,
    this.filled = false,
    this.icon,
  });

  final String text;
  final Color color;
  final bool filled;
  final IconData? icon;

  @override
  Widget build(BuildContext context) {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
      decoration: BoxDecoration(
        color: filled ? color : color.withValues(alpha: 0.12),
        borderRadius: BorderRadius.circular(6),
        border: Border.all(color: color.withValues(alpha: filled ? 1 : 0.5)),
      ),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: <Widget>[
          if (icon != null) ...<Widget>[
            Icon(icon,
                size: 12,
                color: filled ? HudTheme.background : color),
            const SizedBox(width: 4),
          ],
          Text(
            text,
            style: TextStyle(
              fontSize: 11,
              fontWeight: FontWeight.w700,
              letterSpacing: 0.6,
              color: filled ? HudTheme.background : color,
            ),
          ),
        ],
      ),
    );
  }
}

/// Section heading used on the settings-style screens.
class SectionHeader extends StatelessWidget {
  const SectionHeader(this.title, {super.key, this.subtitle, this.trailing});

  final String title;
  final String? subtitle;
  final Widget? trailing;

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.fromLTRB(4, 20, 4, 10),
      child: Row(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: <Widget>[
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: <Widget>[
                Text(
                  title.toUpperCase(),
                  style: HudTheme.hudLabel.copyWith(
                    color: HudTheme.textSecondary,
                    fontSize: 12,
                  ),
                ),
                if (subtitle != null) ...<Widget>[
                  const SizedBox(height: 4),
                  Text(subtitle!, style: HudTheme.caption),
                ],
              ],
            ),
          ),
          if (trailing != null) trailing!,
        ],
      ),
    );
  }
}
