import 'package:flutter/material.dart';

import '../../audio/alert_policy.dart';
import '../../pipeline/pipeline_result.dart';
import '../../simulation/turn_signal.dart';
import '../../world_model/hazard.dart';
import '../theme.dart';
import 'hud_panels.dart';

/// The HUD stripped back to what a driver can read at a glance.
///
/// The full HUD is an instrument panel: boxes, distances, confidences, a
/// decision and its reasoning. All of that is worth having and none of it is
/// worth reading at 100 km/h. Drive mode keeps three things — the speed, the
/// one thing that matters right now, and the indicators — and moves
/// everything else to Replay, where there is time to look at it properly.
///
/// The one thing that matters is chosen by the same policy that decides what
/// to say out loud, so the screen and the speaker never disagree.
class DriveModeHud extends StatelessWidget {
  const DriveModeHud({
    super.key,
    required this.result,
    required this.alert,
    this.audioEnabled = true,
  });

  final PipelineResult result;

  /// The alert the policy selected, or null when there is nothing to say.
  final DrivingAlert? alert;

  final bool audioEnabled;

  @override
  Widget build(BuildContext context) {
    final int speedKph = result.world.ego.speedKph.round();
    final int? limit = result.world.effectiveSpeedLimitKph;
    final bool overLimit = limit != null && speedKph > limit + 5;

    return IgnorePointer(
      child: Stack(
        children: <Widget>[
          // Speed, bottom left, big enough for peripheral vision.
          Positioned(
            left: 18,
            bottom: 24,
            child: Row(
              crossAxisAlignment: CrossAxisAlignment.baseline,
              textBaseline: TextBaseline.alphabetic,
              children: <Widget>[
                Text(
                  '$speedKph',
                  style: TextStyle(
                    fontFamily: HudTheme.monoFamily,
                    fontSize: 76,
                    height: 1,
                    fontWeight: FontWeight.w300,
                    color: overLimit ? HudTheme.critical : Colors.white,
                    shadows: const <Shadow>[
                      Shadow(blurRadius: 12, color: Colors.black87),
                    ],
                  ),
                ),
                const SizedBox(width: 8),
                const Text(
                  'km/h',
                  style: TextStyle(
                    fontFamily: HudTheme.monoFamily,
                    fontSize: 16,
                    color: Colors.white70,
                    shadows: <Shadow>[
                      Shadow(blurRadius: 8, color: Colors.black87),
                    ],
                  ),
                ),
              ],
            ),
          ),

          if (limit != null)
            Positioned(
              right: 18,
              bottom: 28,
              child: _SpeedLimitRoundel(
                limitKph: limit,
                exceeded: overLimit,
              ),
            ),

          if (alert != null)
            Positioned(
              left: 0,
              right: 0,
              top: 0,
              child: _AlertBanner(alert: alert!, muted: !audioEnabled),
            ),

          Positioned(
            left: 0,
            right: 0,
            bottom: 128,
            child: Center(
              child: _Indicators(
                signal: result.command.turnSignal,
                timestampMicros: result.world.timestampMicros,
              ),
            ),
          ),

          // Never removed, in any mode.
          const Positioned(
            left: 0,
            right: 0,
            bottom: 4,
            child: Center(child: SimulationOnlyBadge(compact: true)),
          ),
        ],
      ),
    );
  }
}

/// One line across the top, in the colour of its urgency.
///
/// Deliberately the only text on the screen besides the speed. Two things
/// competing for attention is one thing too many.
class _AlertBanner extends StatelessWidget {
  const _AlertBanner({required this.alert, required this.muted});

  final DrivingAlert alert;
  final bool muted;

  @override
  Widget build(BuildContext context) {
    final Color colour = switch (alert.severity) {
      HazardSeverity.critical => HudTheme.critical,
      HazardSeverity.warning => HudTheme.warning,
      HazardSeverity.caution => HudTheme.caution,
      HazardSeverity.info => HudTheme.info,
    };

    return Container(
      padding: const EdgeInsets.symmetric(vertical: 16, horizontal: 18),
      color: colour.withValues(alpha: 0.92),
      child: SafeArea(
        bottom: false,
        child: Row(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            if (muted) ...<Widget>[
              const Icon(Icons.volume_off, size: 20, color: Colors.black87),
              const SizedBox(width: 10),
            ],
            Flexible(
              child: Text(
                alert.displayText,
                textAlign: TextAlign.center,
                maxLines: 2,
                overflow: TextOverflow.ellipsis,
                style: const TextStyle(
                  fontFamily: HudTheme.monoFamily,
                  fontSize: 26,
                  fontWeight: FontWeight.w700,
                  color: Colors.black,
                  letterSpacing: 1.5,
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }
}

class _SpeedLimitRoundel extends StatelessWidget {
  const _SpeedLimitRoundel({required this.limitKph, required this.exceeded});

  final int limitKph;
  final bool exceeded;

  @override
  Widget build(BuildContext context) => Container(
        width: 74,
        height: 74,
        decoration: BoxDecoration(
          shape: BoxShape.circle,
          color: Colors.white,
          border: Border.all(
            color: exceeded ? HudTheme.critical : const Color(0xFFD62828),
            width: 7,
          ),
        ),
        alignment: Alignment.center,
        child: Text(
          '$limitKph',
          style: const TextStyle(
            fontFamily: HudTheme.monoFamily,
            fontSize: 26,
            fontWeight: FontWeight.w800,
            color: Colors.black,
          ),
        ),
      );
}

class _Indicators extends StatelessWidget {
  const _Indicators({required this.signal, required this.timestampMicros});

  final TurnSignalState signal;
  final int timestampMicros;

  @override
  Widget build(BuildContext context) {
    if (!signal.signal.isActive) return const SizedBox.shrink();
    final bool on = signal.blinkOn(timestampMicros);
    final Color colour = signal.signal == TurnSignal.hazard
        ? HudTheme.critical
        : HudTheme.caution;

    return Row(
      mainAxisSize: MainAxisSize.min,
      children: <Widget>[
        Icon(
          Icons.keyboard_double_arrow_left,
          size: 46,
          color: on && signal.signal.showsLeft
              ? colour
              : Colors.white.withValues(alpha: 0.12),
        ),
        const SizedBox(width: 60),
        Icon(
          Icons.keyboard_double_arrow_right,
          size: 46,
          color: on && signal.signal.showsRight
              ? colour
              : Colors.white.withValues(alpha: 0.12),
        ),
      ],
    );
  }
}
