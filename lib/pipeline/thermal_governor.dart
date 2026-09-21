import 'dart:math' as math;

import '../core/geometry.dart';
import '../core/logging.dart';
import '../debug/system_monitor.dart';
import 'pipeline_config.dart';

/// How hard the stack is currently allowed to work.
enum PerformanceLevel {
  /// Everything on, at the configured rates.
  full('FULL', 'All stages at their configured rates'),

  /// The expensive, slow-changing stages run less often.
  reduced('REDUCED', 'Depth and segmentation run less often'),

  /// Frame rate cut as well; only what the decision needs runs often.
  conservative('CONSERVATIVE', 'Frame rate cut; depth and segmentation rare'),

  /// Optional stages off. The stack still detects objects and still says so
  /// when it cannot keep up.
  survival('SURVIVAL', 'Optional stages off to keep detection running');

  const PerformanceLevel(this.label, this.description);
  final String label;
  final String description;
}

/// One decision by the governor, with the reason attached.
class PerformancePlan {
  const PerformancePlan({
    required this.level,
    required this.targetFps,
    required this.cadence,
    required this.toggles,
    required this.reason,
    this.isFrameRateInsufficient = false,
    this.requiredFps = 0,
    this.achievedFps = 0,
  });

  final PerformanceLevel level;
  final double targetFps;
  final StageCadence cadence;
  final PipelineStageToggles toggles;

  /// Why this level, in one line, for the HUD and the recording.
  final String reason;

  /// True when the achieved rate is below what the current speed needs.
  ///
  /// This is the honest half of throttling: a stack that quietly runs at
  /// 4 FPS at 100 km/h is looking at the road once every 7 metres and is not
  /// entitled to the confidence it would otherwise report.
  final bool isFrameRateInsufficient;

  final double requiredFps;
  final double achievedFps;

  Map<String, dynamic> toJson() => <String, dynamic>{
        'level': level.name,
        'targetFps': double.parse(targetFps.toStringAsFixed(1)),
        'achievedFps': double.parse(achievedFps.toStringAsFixed(1)),
        'requiredFps': double.parse(requiredFps.toStringAsFixed(1)),
        'reason': reason,
        if (isFrameRateInsufficient) 'frameRateInsufficient': true,
      };

  @override
  String toString() => 'PerformancePlan(${level.label} @ '
      '${targetFps.toStringAsFixed(0)} FPS: $reason)';
}

/// Decides how hard the pipeline may work, from heat, battery and latency.
///
/// On a phone clamped to a windscreen in sunlight, thermal throttling — not
/// the algorithms — is what limits sustained frame rate. Once the SoC is
/// being clocked down, asking it for more work produces *less* output and
/// more heat, so the only useful lever is to ask for less.
///
/// Three rules shape everything here:
///
///  * **Shed the slow-changing stages first.** The road does not move between
///    frames, so depth and segmentation tolerate running at a third of the
///    rate. Object detection does not: two frames in three reporting a stale
///    scene is exactly the lie this project exists to avoid, so the detector
///    keeps running every accepted frame at every level.
///  * **Fall fast, recover slowly.** A phone that has just cooled from
///    MODERATE to LIGHT is one minute of sunlight from MODERATE again.
///    Stepping down is immediate; stepping back up needs a sustained cool
///    period, or the stack oscillates and spends its budget on the
///    oscillation.
///  * **Say when it is not enough.** The required frame rate is not a
///    constant — it comes from speed. Below it the plan is marked
///    insufficient, autonomy confidence drops and the HUD says so, rather
///    than the stack reporting a confident view of a road it is barely
///    looking at.
class ThermalGovernor {
  ThermalGovernor({
    this.baseTargetFps = 20,
    this.metresPerLook = 1.5,
    this.minRequiredFps = 5,
    this.maxRequiredFps = 15,
    this.recoveryDwellSeconds = 45,
    this.lowBatteryPercent = 15,
  });

  static const String _tag = 'ThermalGovernor';

  /// The rate asked for when nothing is limiting us.
  final double baseTargetFps;

  /// How far the vehicle may travel between two looks at the road.
  ///
  /// This is what turns "is the frame rate enough?" from a matter of taste
  /// into arithmetic: at 100 km/h, 1.5 m between looks needs 18 FPS; at
  /// 30 km/h it needs 6. The requirement follows the speed.
  final double metresPerLook;

  final double minRequiredFps;
  final double maxRequiredFps;

  /// How long conditions must stay good before the governor steps back up.
  final double recoveryDwellSeconds;

  final int lowBatteryPercent;

  PerformanceLevel _level = PerformanceLevel.full;
  double _goodConditionSeconds = 0;
  PerformancePlan? _plan;

  PerformanceLevel get level => _level;
  PerformancePlan? get plan => _plan;

  void reset() {
    _level = PerformanceLevel.full;
    _goodConditionSeconds = 0;
    _plan = null;
  }

  /// Frame rate the current speed demands.
  double requiredFpsFor(double speedMps) => clampDouble(
        speedMps / metresPerLook,
        minRequiredFps,
        maxRequiredFps,
      );

  /// Re-evaluate. Called once per pipeline cycle.
  PerformancePlan update({
    required SystemSample? system,
    required double achievedFps,
    required double speedMps,
    required double pipelineP95Ms,
    required double dtSeconds,
    StageCadence? baseCadence,
    PipelineStageToggles? baseToggles,
  }) {
    final ThermalStatus thermal = system?.thermal ?? ThermalStatus.unknown;
    final bool lowBattery = system != null &&
        !system.isCharging &&
        system.batteryPercent <= lowBatteryPercent;

    // --- What the conditions demand -----------------------------------
    PerformanceLevel demanded = PerformanceLevel.full;
    String reason = 'thermal and battery headroom available';

    if (thermal.level >= ThermalStatus.critical.level) {
      demanded = PerformanceLevel.survival;
      reason = 'thermal ${thermal.label}: shedding every optional stage';
    } else if (thermal == ThermalStatus.severe) {
      demanded = PerformanceLevel.conservative;
      reason = 'thermal ${thermal.label}: the SoC is heavily clocked down';
    } else if (thermal == ThermalStatus.moderate) {
      demanded = PerformanceLevel.reduced;
      reason = 'thermal ${thermal.label}: the SoC is already being '
          'clocked down, so asking for more work returns less';
    } else if ((system?.thermalHeadroom ?? 0) >= 0.85) {
      // Acting on the forecast rather than the status. At 0.85 there is still
      // time to shed a stage and stay out of throttling altogether, which is
      // worth far more than reacting once the clocks are already down.
      demanded = PerformanceLevel.reduced;
      reason = 'thermal headroom '
          '${(system!.thermalHeadroom! * 100).round()}%: throttling '
          'forecast within a minute';
    }

    if (lowBattery && demanded.index < PerformanceLevel.conservative.index) {
      demanded = PerformanceLevel.conservative;
      reason = 'battery ${system.batteryPercent}%: a phone that dies '
          'mid-drive is worse than a lower frame rate';
    }

    // Latency is independent evidence: if the pipeline's p95 already exceeds
    // the frame budget, the target rate is fiction and the queue is just
    // adding delay to every result.
    final double budgetMs = 1000 / math.max(1, _targetFpsFor(_level));
    if (pipelineP95Ms > budgetMs * 1.35 &&
        demanded.index < PerformanceLevel.reduced.index) {
      demanded = PerformanceLevel.reduced;
      reason = 'pipeline p95 ${pipelineP95Ms.toStringAsFixed(0)} ms exceeds '
          'the ${budgetMs.toStringAsFixed(0)} ms frame budget';
    }

    // --- Fall fast, recover slowly ------------------------------------
    if (demanded.index > _level.index) {
      _level = demanded;
      _goodConditionSeconds = 0;
      Log.warn(_tag, 'stepping down to ${_level.label}: $reason');
    } else if (demanded.index < _level.index) {
      _goodConditionSeconds += dtSeconds;
      if (_goodConditionSeconds >= recoveryDwellSeconds) {
        // One step at a time, so recovery is a ramp rather than a spike that
        // puts the phone straight back into throttling.
        _level = PerformanceLevel.values[_level.index - 1];
        _goodConditionSeconds = 0;
        reason = 'conditions clear for '
            '${recoveryDwellSeconds.round()} s: stepping up to '
            '${_level.label}';
        Log.info(_tag, reason);
      } else {
        reason = '${_level.label} held — '
            '${(recoveryDwellSeconds - _goodConditionSeconds).round()} s '
            'before stepping up';
      }
    } else {
      _goodConditionSeconds = 0;
    }

    final double required = requiredFpsFor(speedMps);
    final double target = _targetFpsFor(_level);

    final PerformancePlan plan = PerformancePlan(
      level: _level,
      targetFps: target,
      cadence: _cadenceFor(_level, baseCadence),
      toggles: _togglesFor(_level, baseToggles),
      reason: reason,
      // Only a measured rate can be insufficient. Before the first second of
      // a drive there is nothing to judge.
      isFrameRateInsufficient: achievedFps > 0 && achievedFps < required,
      requiredFps: required,
      achievedFps: achievedFps,
    );
    _plan = plan;
    return plan;
  }

  double _targetFpsFor(PerformanceLevel level) => switch (level) {
        PerformanceLevel.full => baseTargetFps,
        PerformanceLevel.reduced => baseTargetFps * 0.75,
        PerformanceLevel.conservative => baseTargetFps * 0.5,
        PerformanceLevel.survival => baseTargetFps * 0.35,
      };

  /// Cadences per level.
  ///
  /// Every multiplier here applies to a stage whose subject changes slowly:
  /// the road surface, the lane geometry, the shape of the drivable area.
  /// None of them applies to object detection, which runs on every accepted
  /// frame at every level.
  StageCadence _cadenceFor(PerformanceLevel level, StageCadence? base) {
    final StageCadence b = base ?? const StageCadence();
    int scale(int n, int factor) => math.max(1, n * factor);

    return switch (level) {
      PerformanceLevel.full => b,
      PerformanceLevel.reduced => StageCadence(
          segmentationEveryNFrames: scale(b.segmentationEveryNFrames, 2),
          depthEveryNFrames: scale(b.depthEveryNFrames, 2),
          laneEveryNFrames: scale(b.laneEveryNFrames, 2),
          signsEveryNFrames: scale(b.signsEveryNFrames, 2),
          markingsEveryNFrames: scale(b.markingsEveryNFrames, 2),
        ),
      PerformanceLevel.conservative => StageCadence(
          segmentationEveryNFrames: scale(b.segmentationEveryNFrames, 3),
          depthEveryNFrames: scale(b.depthEveryNFrames, 3),
          laneEveryNFrames: scale(b.laneEveryNFrames, 3),
          signsEveryNFrames: scale(b.signsEveryNFrames, 3),
          markingsEveryNFrames: scale(b.markingsEveryNFrames, 3),
        ),
      PerformanceLevel.survival => StageCadence(
          segmentationEveryNFrames: scale(b.segmentationEveryNFrames, 4),
          depthEveryNFrames: scale(b.depthEveryNFrames, 4),
          laneEveryNFrames: scale(b.laneEveryNFrames, 4),
          signsEveryNFrames: scale(b.signsEveryNFrames, 4),
          markingsEveryNFrames: scale(b.markingsEveryNFrames, 4),
        ),
    };
  }

  /// At SURVIVAL the neural depth and segmentation stages are switched off
  /// outright rather than merely slowed.
  ///
  /// Both have classical fallbacks that cost almost nothing — ground-plane
  /// geometry for distance, region growing for the corridor — so turning them
  /// off costs confidence, which is reported, rather than capability, which
  /// would not be.
  PipelineStageToggles _togglesFor(
    PerformanceLevel level,
    PipelineStageToggles? base,
  ) {
    final PipelineStageToggles b = base ?? const PipelineStageToggles();
    if (level != PerformanceLevel.survival) return b;
    return b.copyWith(depth: false, segmentation: false);
  }
}
