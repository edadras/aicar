import 'package:flutter/material.dart';

import '../../core/confidence.dart';
import '../../core/safety.dart';
import '../../decision/driving_decision.dart';
import '../../perception/traffic_light.dart';
import '../../perception/traffic_sign.dart';
import '../../pipeline/pipeline_result.dart';
import '../../simulation/simulated_control.dart';
import '../../world_model/hazard.dart';
import '../../world_model/world_state.dart';
import '../theme.dart';
import 'steering_wheel.dart';

/// The bar along the bottom of the HUD: speed, steering, throttle, brake.
class ControlStrip extends StatelessWidget {
  const ControlStrip({
    super.key,
    required this.result,
    required this.steeringRatio,
    required this.steeringLimitDegrees,
  });

  final PipelineResult result;
  final double steeringRatio;
  final double steeringLimitDegrees;

  @override
  Widget build(BuildContext context) {
    final SimulatedControlCommand command = result.command;
    final WorldState world = result.world;
    final bool hasPath = result.path.isUsable;

    return Container(
      padding: const EdgeInsets.fromLTRB(16, 12, 16, 12),
      decoration: BoxDecoration(
        color: HudTheme.background.withValues(alpha: 0.88),
        border: const Border(top: BorderSide(color: HudTheme.outline)),
      ),
      child: Row(
        crossAxisAlignment: CrossAxisAlignment.center,
        children: <Widget>[
          HudReadout(
            label: 'Speed',
            value: world.ego.speedKph.toStringAsFixed(0),
            unit: 'km/h',
            large: true,
            width: 112,
            valueColor: HudTheme.forConfidence(world.ego.speedConfidence),
          ),
          const SizedBox(width: 8),
          HudReadout(
            label: 'Steering',
            value: hasPath ? command.steeringDisplay : '—',
            width: 86,
            valueColor: hasPath ? HudTheme.textPrimary : HudTheme.textDim,
          ),
          const SizedBox(width: 12),
          Expanded(
            child: Column(
              mainAxisSize: MainAxisSize.min,
              children: <Widget>[
                PedalBar(
                  label: 'Throttle',
                  percent: command.throttlePercent,
                  color: HudTheme.accent,
                ),
                const SizedBox(height: 10),
                PedalBar(
                  label: 'Brake',
                  percent: command.brakePercent,
                  color: command.isEmergency
                      ? HudTheme.critical
                      : HudTheme.warning,
                  emphasised: true,
                ),
              ],
            ),
          ),
          const SizedBox(width: 14),
          SteeringWheelIndicator(
            roadWheelDegrees: command.steeringAngleDegrees,
            steeringRatio: steeringRatio,
            limitDegrees: steeringLimitDegrees,
            isActive: hasPath,
            size: 78,
          ),
        ],
      ),
    );
  }
}

/// The decision panel: what the stack would do, and why.
class DecisionPanel extends StatelessWidget {
  const DecisionPanel({super.key, required this.decision});

  final DrivingDecision decision;

  @override
  Widget build(BuildContext context) {
    final Color color = switch (decision.state) {
      DrivingState.emergencyBrakeSimulation => HudTheme.critical,
      DrivingState.pedestrianYield ||
      DrivingState.obstacleAvoidanceSimulation =>
        HudTheme.warning,
      DrivingState.stop || DrivingState.wait => HudTheme.caution,
      DrivingState.uncertain => HudTheme.warning,
      _ => HudTheme.accent,
    };

    return Container(
      width: 244,
      padding: const EdgeInsets.all(12),
      decoration: BoxDecoration(
        color: HudTheme.background.withValues(alpha: 0.82),
        borderRadius: BorderRadius.circular(10),
        border: Border.all(color: color.withValues(alpha: 0.6)),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        mainAxisSize: MainAxisSize.min,
        children: <Widget>[
          Row(
            children: <Widget>[
              Expanded(
                child: Text(
                  decision.state.label,
                  style: HudTheme.hudValue.copyWith(
                    fontSize: 17,
                    color: color,
                  ),
                ),
              ),
              Text(
                '${decision.confidencePercent}%',
                style: HudTheme.caption.copyWith(
                  color: HudTheme.forConfidence(decision.confidence),
                  fontWeight: FontWeight.w700,
                ),
              ),
            ],
          ),
          const SizedBox(height: 6),
          Text(
            decision.reason,
            style: HudTheme.caption.copyWith(color: HudTheme.textPrimary),
            maxLines: 3,
            overflow: TextOverflow.ellipsis,
          ),
          if (decision.heldForSeconds > 0.5) ...<Widget>[
            const SizedBox(height: 4),
            Text(
              'held ${decision.heldForSeconds.toStringAsFixed(1)} s',
              style: HudTheme.caption.copyWith(
                color: HudTheme.textDim,
                fontSize: 11,
              ),
            ),
          ],
        ],
      ),
    );
  }
}

/// The warning banner across the top of the HUD.
class HazardBanner extends StatelessWidget {
  const HazardBanner({super.key, required this.world, this.pulse = 0});

  final WorldState world;
  final double pulse;

  @override
  Widget build(BuildContext context) {
    final Hazard? hazard = world.primaryHazard;
    final bool autonomyLow = world.autonomy.isLow;

    if (hazard == null && !autonomyLow) return const SizedBox.shrink();

    final Color color = hazard != null
        ? HudTheme.forSeverity(hazard.severity)
        : HudTheme.warning;
    final bool critical = hazard?.severity == HazardSeverity.critical;

    return Container(
      width: double.infinity,
      padding: const EdgeInsets.symmetric(horizontal: 14, vertical: 9),
      decoration: BoxDecoration(
        color: color.withValues(alpha: critical ? 0.25 + 0.25 * pulse : 0.18),
        border: Border.all(color: color, width: critical ? 2 : 1),
        borderRadius: BorderRadius.circular(10),
      ),
      child: Row(
        children: <Widget>[
          Icon(
            critical ? Icons.warning_amber_rounded : Icons.info_outline,
            color: color,
            size: 20,
          ),
          const SizedBox(width: 10),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              mainAxisSize: MainAxisSize.min,
              children: <Widget>[
                Text(
                  hazard?.type.label ?? 'AUTONOMY CONFIDENCE LOW',
                  style: HudTheme.hudValue.copyWith(
                    fontSize: 15,
                    color: color,
                  ),
                ),
                const SizedBox(height: 2),
                Text(
                  hazard?.description ??
                      'Overall ${(world.autonomy.overall * 100).round()}% — '
                          'weakest: ${world.autonomy.weakestSubsystem}',
                  style: HudTheme.caption,
                  maxLines: 2,
                  overflow: TextOverflow.ellipsis,
                ),
              ],
            ),
          ),
          if (hazard?.timeToCollisionSeconds != null)
            HudReadout(
              label: 'TTC',
              value: hazard!.timeToCollisionSeconds!.toStringAsFixed(1),
              unit: 's',
              valueColor: color,
            ),
        ],
      ),
    );
  }
}

/// Speed limit, traffic light and navigation, stacked down the right side.
class RoadContextPanel extends StatelessWidget {
  const RoadContextPanel({super.key, required this.world});

  final WorldState world;

  @override
  Widget build(BuildContext context) {
    final int? limit = world.effectiveSpeedLimitKph;
    final TrafficLight? light = world.governingTrafficLight;
    final RegulatoryContext regulatory = world.regulatory;

    final List<Widget> children = <Widget>[];

    if (limit != null) {
      children.add(_SpeedLimitDisc(
        limitKph: limit,
        confidence: regulatory.speedLimitConfidence,
        exceeded: world.ego.speedKph > limit + 5,
      ));
    }

    if (light != null) {
      children.add(_ContextCard(
        color: switch (light.color) {
          TrafficLightColor.red ||
          TrafficLightColor.redYellow =>
            HudTheme.critical,
          TrafficLightColor.yellow => HudTheme.caution,
          TrafficLightColor.green => HudTheme.accent,
          _ => HudTheme.textDim,
        },
        title: light.color.label,
        subtitle: light.distanceMeters == null
            ? light.arrow.label
            : '${light.distanceMeters!.toStringAsFixed(0)} m',
      ));
    }

    if (regulatory.inSchoolZone) {
      children.add(const _ContextCard(
        color: HudTheme.caution,
        title: 'SCHOOL ZONE',
        subtitle: '30 km/h',
      ));
    }
    if (regulatory.inRoadWorks) {
      children.add(const _ContextCard(
        color: HudTheme.warning,
        title: 'ROAD WORKS',
        subtitle: 'reduced limit',
      ));
    }
    if (regulatory.overtakingProhibited) {
      children.add(const _ContextCard(
        color: HudTheme.info,
        title: 'NO OVERTAKING',
        subtitle: 'lane change suppressed',
      ));
    }

    final String? instruction =
        world.routeProgress?.displayInstruction;
    if (instruction != null) {
      children.add(_ContextCard(
        color: HudTheme.info,
        title: world.navigationIntent.label,
        subtitle: instruction,
      ));
    }

    if (children.isEmpty) return const SizedBox.shrink();

    return Column(
      crossAxisAlignment: CrossAxisAlignment.end,
      mainAxisSize: MainAxisSize.min,
      children: <Widget>[
        for (final Widget child in children) ...<Widget>[
          child,
          const SizedBox(height: 8),
        ],
      ],
    );
  }
}

class _SpeedLimitDisc extends StatelessWidget {
  const _SpeedLimitDisc({
    required this.limitKph,
    required this.confidence,
    required this.exceeded,
  });

  final int limitKph;
  final double confidence;
  final bool exceeded;

  @override
  Widget build(BuildContext context) {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.end,
      mainAxisSize: MainAxisSize.min,
      children: <Widget>[
        Container(
          width: 62,
          height: 62,
          decoration: BoxDecoration(
            shape: BoxShape.circle,
            color: Colors.white,
            border: Border.all(
              color: exceeded ? HudTheme.critical : const Color(0xFFD62828),
              width: 6,
            ),
          ),
          alignment: Alignment.center,
          child: Text(
            '$limitKph',
            style: const TextStyle(
              fontFamily: HudTheme.monoFamily,
              fontSize: 22,
              fontWeight: FontWeight.w800,
              color: Colors.black,
            ),
          ),
        ),
        const SizedBox(height: 3),
        Text(
          '${(confidence * 100).round()}%',
          style: HudTheme.caption.copyWith(
            fontSize: 10,
            color: HudTheme.forConfidence(confidence),
          ),
        ),
      ],
    );
  }
}

class _ContextCard extends StatelessWidget {
  const _ContextCard({
    required this.color,
    required this.title,
    required this.subtitle,
  });

  final Color color;
  final String title;
  final String subtitle;

  @override
  Widget build(BuildContext context) {
    return Container(
      constraints: const BoxConstraints(maxWidth: 176),
      padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 7),
      decoration: BoxDecoration(
        color: HudTheme.background.withValues(alpha: 0.82),
        borderRadius: BorderRadius.circular(8),
        border: Border.all(color: color.withValues(alpha: 0.7)),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.end,
        mainAxisSize: MainAxisSize.min,
        children: <Widget>[
          Text(
            title,
            style: HudTheme.hudValue.copyWith(fontSize: 14, color: color),
            textAlign: TextAlign.right,
          ),
          Text(
            subtitle,
            style: HudTheme.caption.copyWith(fontSize: 11),
            textAlign: TextAlign.right,
          ),
        ],
      ),
    );
  }
}

/// The permanent reminder that nothing here reaches a vehicle.
class SimulationOnlyBadge extends StatelessWidget {
  const SimulationOnlyBadge({super.key, this.compact = false});

  final bool compact;

  @override
  Widget build(BuildContext context) {
    return Container(
      padding: EdgeInsets.symmetric(
        horizontal: compact ? 8 : 10,
        vertical: compact ? 4 : 6,
      ),
      decoration: BoxDecoration(
        color: HudTheme.info.withValues(alpha: 0.14),
        borderRadius: BorderRadius.circular(6),
        border: Border.all(color: HudTheme.info.withValues(alpha: 0.6)),
      ),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: <Widget>[
          const Icon(Icons.science_outlined,
              size: 13, color: HudTheme.info),
          const SizedBox(width: 6),
          Text(
            compact ? 'SIMULATION ONLY' : SafetyMode.banner,
            style: const TextStyle(
              fontSize: 10.5,
              fontWeight: FontWeight.w700,
              letterSpacing: 0.7,
              color: HudTheme.info,
            ),
          ),
        ],
      ),
    );
  }
}

/// Compact confidence breakdown, shown in the HUD corner and in Debug.
class AutonomyConfidencePanel extends StatelessWidget {
  const AutonomyConfidencePanel({
    super.key,
    required this.autonomy,
    this.dense = false,
  });

  final AutonomyConfidence autonomy;
  final bool dense;

  @override
  Widget build(BuildContext context) {
    final Map<String, double> parts = <String, double>{
      'Perception': autonomy.perception,
      'Lanes': autonomy.lanes,
      'Depth': autonomy.depth,
      'Ego': autonomy.egoMotion,
      'Plan': autonomy.planning,
    };

    return Container(
      padding: const EdgeInsets.all(10),
      decoration: BoxDecoration(
        color: HudTheme.background.withValues(alpha: 0.82),
        borderRadius: BorderRadius.circular(8),
        border: Border.all(
          color: autonomy.isLow ? HudTheme.warning : HudTheme.outline,
        ),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        mainAxisSize: MainAxisSize.min,
        children: <Widget>[
          Row(
            children: <Widget>[
              Text('AUTONOMY', style: HudTheme.hudLabel),
              const SizedBox(width: 8),
              Text(
                '${(autonomy.overall * 100).round()}%',
                style: HudTheme.hudValue.copyWith(
                  fontSize: 15,
                  color: HudTheme.forConfidence(autonomy.overall),
                ),
              ),
            ],
          ),
          if (!dense) ...<Widget>[
            const SizedBox(height: 6),
            for (final MapEntry<String, double> e in parts.entries)
              Padding(
                padding: const EdgeInsets.only(bottom: 3),
                child: Row(
                  children: <Widget>[
                    SizedBox(
                      width: 72,
                      child: Text(e.key, style: HudTheme.caption),
                    ),
                    Expanded(
                      child: ClipRRect(
                        borderRadius: BorderRadius.circular(2),
                        child: LinearProgressIndicator(
                          value: e.value.clamp(0.0, 1.0),
                          minHeight: 4,
                          backgroundColor: HudTheme.outline,
                          valueColor: AlwaysStoppedAnimation<Color>(
                            HudTheme.forConfidence(e.value),
                          ),
                        ),
                      ),
                    ),
                    const SizedBox(width: 6),
                    SizedBox(
                      width: 30,
                      child: Text(
                        '${(e.value * 100).round()}',
                        style: HudTheme.caption.copyWith(fontSize: 11),
                        textAlign: TextAlign.right,
                      ),
                    ),
                  ],
                ),
              ),
          ],
        ],
      ),
    );
  }
}
