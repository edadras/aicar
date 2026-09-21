import 'dart:async';

import '../camera/camera_frame.dart';
import '../core/logging.dart';
import '../decision/driving_decision.dart';
import '../pipeline/perception_pipeline.dart';
import '../pipeline/pipeline_result.dart';
import '../recording/recording_schema.dart';
import '../recording/session_store.dart';
import '../sensors/ego_motion.dart';
import '../sensors/sensor_fusion.dart';
import '../simulation/simulated_control.dart';
import 'replay_reader.dart';

/// How a recorded drive is played back.
enum ReplayMode {
  /// Show exactly what the stack decided at the time. Fast, needs no models,
  /// and is the ground truth for "what did it actually do".
  recordedResults('Recorded results',
      'Replays the decisions exactly as they were made'),

  /// Run the current models and logic over the recorded frames. This is what
  /// makes the recorder worth having: a new detector, a changed threshold or
  /// a rewritten planner can be evaluated against real footage, repeatably,
  /// and compared against what the old build did.
  rerunAi('Re-run AI',
      'Runs the currently selected models over the recorded frames');

  const ReplayMode(this.label, this.description);
  final String label;
  final String description;
}

/// One step of a replay: the recorded state, and (when re-running) the fresh
/// result, so the UI can show them side by side.
class ReplayStep {
  const ReplayStep({
    required this.index,
    required this.recorded,
    required this.frame,
    this.rerun,
    this.comparison,
  });

  final int index;
  final ReplayFrame recorded;

  /// The decoded image, when one was recorded.
  final CameraFrame? frame;

  /// Fresh pipeline output, in [ReplayMode.rerunAi].
  final PipelineResult? rerun;

  /// Difference between recorded and fresh results.
  final ReplayComparison? comparison;

  double get timestampSeconds => recorded.timestampSeconds;
}

/// Difference between what was recorded and what the current build produces.
///
/// This is the number a model change is judged on, so it is computed and
/// surfaced rather than left for the user to eyeball.
class ReplayComparison {
  const ReplayComparison({
    required this.decisionChanged,
    required this.recordedDecision,
    required this.rerunDecision,
    required this.steeringDeltaDegrees,
    required this.throttleDeltaPercent,
    required this.brakeDeltaPercent,
    required this.trackCountDelta,
    required this.recordedLatencyMs,
    required this.rerunLatencyMs,
  });

  final bool decisionChanged;
  final DrivingState? recordedDecision;
  final DrivingState rerunDecision;
  final double steeringDeltaDegrees;
  final double throttleDeltaPercent;
  final double brakeDeltaPercent;
  final int trackCountDelta;
  final double recordedLatencyMs;
  final double rerunLatencyMs;

  bool get isMaterialDifference =>
      decisionChanged ||
      steeringDeltaDegrees.abs() > 2.0 ||
      brakeDeltaPercent.abs() > 10;

  String get summary {
    if (!decisionChanged && !isMaterialDifference) return 'matches';
    final List<String> parts = <String>[];
    if (decisionChanged) {
      parts.add('${recordedDecision?.label ?? '-'} → ${rerunDecision.label}');
    }
    if (steeringDeltaDegrees.abs() > 0.5) {
      parts.add('steer ${steeringDeltaDegrees >= 0 ? '+' : ''}'
          '${steeringDeltaDegrees.toStringAsFixed(1)}°');
    }
    if (brakeDeltaPercent.abs() > 2) {
      parts.add('brake ${brakeDeltaPercent >= 0 ? '+' : ''}'
          '${brakeDeltaPercent.toStringAsFixed(0)}%');
    }
    if (trackCountDelta != 0) {
      parts.add('${trackCountDelta > 0 ? '+' : ''}$trackCountDelta tracks');
    }
    return parts.join(', ');
  }
}

/// Plays a recorded drive back, optionally re-running the AI over it.
class ReplayPlayer {
  ReplayPlayer({
    required this.session,
    required this.mode,
    this.pipeline,
    this.playbackSpeed = 1.0,
  }) : reader = ReplayReader(session);

  static const String _tag = 'ReplayPlayer';

  final RecordedSession session;
  final ReplayReader reader;
  final ReplayMode mode;

  /// Required for [ReplayMode.rerunAi].
  final PerceptionPipeline? pipeline;

  /// 1.0 = real time. 0 = as fast as possible, which is what a batch
  /// evaluation over a long drive wants.
  double playbackSpeed;

  final StreamController<ReplayStep> _steps =
      StreamController<ReplayStep>.broadcast();

  bool _playing = false;
  bool _paused = false;
  int _index = 0;
  int _totalFrames = 0;
  int _materialDifferences = 0;
  final List<ReplayComparison> _comparisons = <ReplayComparison>[];

  Stream<ReplayStep> get steps => _steps.stream;
  bool get isPlaying => _playing;
  bool get isPaused => _paused;
  int get currentIndex => _index;
  int get totalFrames => _totalFrames;
  int get materialDifferences => _materialDifferences;
  List<ReplayComparison> get comparisons =>
      List<ReplayComparison>.unmodifiable(_comparisons);

  /// Whether this session can actually be re-run.
  bool get canRerun => session.hasFrames;

  String? get rerunUnavailableReason {
    if (session.hasFrames) return null;
    return 'This drive was recorded without frames, so the AI cannot be '
        're-run over it. Record with "Data + frames" to enable this.';
  }

  Future<void> prepare() async {
    _totalFrames = await reader.countFrames();
    Log.info(_tag, 'prepared ${session.id}: $_totalFrames frames');
  }

  Future<void> play() async {
    if (_playing) return;
    if (mode == ReplayMode.rerunAi && !canRerun) {
      throw StateError(rerunUnavailableReason!);
    }
    _playing = true;
    _paused = false;
    _index = 0;
    _materialDifferences = 0;
    _comparisons.clear();

    // A re-run must start from a clean pipeline: carrying tracks or a held
    // path over from a live drive would make the comparison meaningless.
    pipeline?.reset();

    final EgoMotionEstimator? estimator =
        mode == ReplayMode.rerunAi ? EgoMotionEstimator() : null;

    int? previousTimestamp;

    try {
      await for (final ReplayFrame recorded in reader.frames()) {
        if (!_playing) break;
        while (_paused && _playing) {
          await Future<void>.delayed(const Duration(milliseconds: 50));
        }
        if (!_playing) break;

        // Real-time pacing.
        if (playbackSpeed > 0 && previousTimestamp != null) {
          final int deltaMicros =
              recorded.timestampMicros - previousTimestamp;
          if (deltaMicros > 0 && deltaMicros < 2000000) {
            await Future<void>.delayed(
              Duration(microseconds: (deltaMicros / playbackSpeed).round()),
            );
          }
        }
        previousTimestamp = recorded.timestampMicros;

        final ReplayStep step = await _buildStep(recorded, estimator);
        if (!_steps.isClosed) _steps.add(step);
        _index++;
      }
    } finally {
      _playing = false;
    }
  }

  Future<ReplayStep> _buildStep(
    ReplayFrame recorded,
    EgoMotionEstimator? estimator,
  ) async {
    CameraFrame? frame;
    if (recorded.imagePath != null) {
      frame = await reader.loadImage(recorded);
    }

    if (mode == ReplayMode.recordedResults || pipeline == null) {
      return ReplayStep(index: _index, recorded: recorded, frame: frame);
    }

    if (frame == null) {
      // No image for this cycle: nothing to re-run. Show the recorded result
      // rather than skipping, so the timeline stays continuous.
      return ReplayStep(index: _index, recorded: recorded, frame: null);
    }

    // Feed the recorded sensor history through a fresh estimator, so ego
    // motion in the re-run is derived the same way it was live rather than
    // being taken on trust from the recording.
    EgoMotionState ego;
    if (estimator != null) {
      for (final ImuSample sample in recorded.imuSamples) {
        estimator.onImu(
          rawAccelerationPhone: sample.accelerationMps2,
          angularRatePhone: sample.angularRateRadPerS,
          magneticHeadingDegrees: sample.magneticHeadingDegrees,
          timestampMicros: sample.timestampMicros,
        );
      }
      if (recorded.gpsFix != null) estimator.onGpsFix(recorded.gpsFix!);
      ego = estimator.stateAt(recorded.timestampMicros);
      // If the recording has no IMU stream (structured-only capture of an
      // older schema), fall back to the recorded ego state.
      if (recorded.imuSamples.isEmpty && recorded.ego != null) {
        ego = recorded.ego!;
      }
    } else {
      ego = recorded.ego ?? EgoMotionState.unknown;
    }

    final PipelineResult result = await pipeline!.process(
      frame: frame,
      ego: ego,
    );

    final ReplayComparison comparison = _compare(recorded, result);
    _comparisons.add(comparison);
    if (comparison.isMaterialDifference) _materialDifferences++;

    return ReplayStep(
      index: _index,
      recorded: recorded,
      frame: frame,
      rerun: result,
      comparison: comparison,
    );
  }

  ReplayComparison _compare(ReplayFrame recorded, PipelineResult rerun) {
    // Only the commands are compared: the simulated vehicle state is the
    // integral of the commands, so a difference there is implied by a
    // difference here and reporting both would double-count it.
    final SimulatedControlCommand? old = recorded.command;
    return ReplayComparison(
      decisionChanged: recorded.decision != null &&
          recorded.decision!.state != rerun.decision.state,
      recordedDecision: recorded.decision?.state,
      rerunDecision: rerun.decision.state,
      steeringDeltaDegrees: old == null
          ? 0
          : rerun.command.steeringAngleDegrees - old.steeringAngleDegrees,
      throttleDeltaPercent: old == null
          ? 0
          : rerun.command.throttlePercent - old.throttlePercent,
      brakeDeltaPercent:
          old == null ? 0 : rerun.command.brakePercent - old.brakePercent,
      trackCountDelta: recorded.world == null
          ? 0
          : rerun.world.tracks.length - recorded.world!.tracks.length,
      recordedLatencyMs: recorded.stageTimings['Total Pipeline'] ?? 0,
      rerunLatencyMs: rerun.totalLatencyMs,
    );
  }

  void pause() => _paused = true;
  void resume() => _paused = false;

  void stop() {
    _playing = false;
    _paused = false;
  }

  Future<void> dispose() async {
    stop();
    await _steps.close();
  }

  /// Aggregate report after a full re-run — what a model change actually did.
  ReplayReport report() {
    int decisionChanges = 0;
    double steeringSum = 0;
    double brakeSum = 0;
    double latencySum = 0;
    final Map<String, int> transitions = <String, int>{};

    for (final ReplayComparison c in _comparisons) {
      if (c.decisionChanged) {
        decisionChanges++;
        final String key =
            '${c.recordedDecision?.label ?? '-'} → ${c.rerunDecision.label}';
        transitions.update(key, (int v) => v + 1, ifAbsent: () => 1);
      }
      steeringSum += c.steeringDeltaDegrees.abs();
      brakeSum += c.brakeDeltaPercent.abs();
      latencySum += c.rerunLatencyMs;
    }

    final int n = _comparisons.isEmpty ? 1 : _comparisons.length;
    return ReplayReport(
      framesCompared: _comparisons.length,
      decisionChanges: decisionChanges,
      materialDifferences: _materialDifferences,
      meanAbsoluteSteeringDeltaDegrees: steeringSum / n,
      meanAbsoluteBrakeDeltaPercent: brakeSum / n,
      meanRerunLatencyMs: latencySum / n,
      decisionTransitions: transitions,
    );
  }
}

/// Summary of a full re-run against a recording.
class ReplayReport {
  const ReplayReport({
    required this.framesCompared,
    required this.decisionChanges,
    required this.materialDifferences,
    required this.meanAbsoluteSteeringDeltaDegrees,
    required this.meanAbsoluteBrakeDeltaPercent,
    required this.meanRerunLatencyMs,
    required this.decisionTransitions,
  });

  final int framesCompared;
  final int decisionChanges;
  final int materialDifferences;
  final double meanAbsoluteSteeringDeltaDegrees;
  final double meanAbsoluteBrakeDeltaPercent;
  final double meanRerunLatencyMs;
  final Map<String, int> decisionTransitions;

  double get decisionAgreement =>
      framesCompared == 0 ? 1 : 1 - decisionChanges / framesCompared;

  Map<String, dynamic> toJson() => <String, dynamic>{
        'frames': framesCompared,
        'decisionChanges': decisionChanges,
        'materialDifferences': materialDifferences,
        'agreement': double.parse(decisionAgreement.toStringAsFixed(4)),
        'meanSteeringDelta': double.parse(
            meanAbsoluteSteeringDeltaDegrees.toStringAsFixed(2)),
        'meanBrakeDelta':
            double.parse(meanAbsoluteBrakeDeltaPercent.toStringAsFixed(2)),
        'meanLatencyMs':
            double.parse(meanRerunLatencyMs.toStringAsFixed(2)),
        'transitions': decisionTransitions,
      };

  @override
  String toString() =>
      'ReplayReport($framesCompared frames, '
      '${(decisionAgreement * 100).toStringAsFixed(1)}% decision agreement, '
      '$materialDifferences material differences)';
}
