import 'dart:math' as math;

import 'ring_buffer.dart';
import 'time_sync.dart';

/// Per-stage timing statistics. One [StageProfile] exists for every pipeline
/// stage (`Object Detection 21ms`, `Depth 34ms`, ...) plus one for the
/// end-to-end latency.
class StageProfile {
  StageProfile(this.name, {int window = 64})
      : _samplesMicros = RingBuffer<int>(window);

  final String name;
  final RingBuffer<int> _samplesMicros;

  int _totalRuns = 0;
  int _peakMicros = 0;

  int get runs => _totalRuns;

  void record(int micros) {
    _samplesMicros.add(micros);
    _totalRuns++;
    if (micros > _peakMicros) _peakMicros = micros;
  }

  double get lastMs => _samplesMicros.isEmpty ? 0 : _samplesMicros.last / 1000.0;

  double get averageMs {
    if (_samplesMicros.isEmpty) return 0;
    int sum = 0;
    for (final int s in _samplesMicros) {
      sum += s;
    }
    return sum / _samplesMicros.length / 1000.0;
  }

  /// 95th percentile — the number that actually explains dropped frames.
  double get p95Ms {
    if (_samplesMicros.isEmpty) return 0;
    final List<int> sorted = _samplesMicros.toList()..sort();
    final int idx = math.min(
      sorted.length - 1,
      ((sorted.length - 1) * 0.95).round(),
    );
    return sorted[idx] / 1000.0;
  }

  double get peakMs => _peakMicros / 1000.0;

  void reset() {
    _samplesMicros.clear();
    _totalRuns = 0;
    _peakMicros = 0;
  }

  Map<String, dynamic> toJson() => <String, dynamic>{
        'name': name,
        'lastMs': double.parse(lastMs.toStringAsFixed(2)),
        'avgMs': double.parse(averageMs.toStringAsFixed(2)),
        'p95Ms': double.parse(p95Ms.toStringAsFixed(2)),
        'peakMs': double.parse(peakMs.toStringAsFixed(2)),
        'runs': _totalRuns,
      };

  @override
  String toString() => '$name ${lastMs.toStringAsFixed(1)}ms '
      '(avg ${averageMs.toStringAsFixed(1)}, p95 ${p95Ms.toStringAsFixed(1)})';
}

/// Canonical stage names. Using constants rather than ad-hoc strings keeps the
/// performance screen, the recorder and the debug overlay in agreement.
class PipelineStageNames {
  const PipelineStageNames._();
  static const String preprocess = 'Preprocess';
  static const String objectDetection = 'Object Detection';
  static const String tracking = 'Tracking';
  static const String segmentation = 'Segmentation';
  static const String laneDetection = 'Lane Detection';
  static const String roadEdge = 'Road Edges';
  static const String depth = 'Depth';
  static const String depthFusion = 'Depth Fusion';
  static const String roadMarkings = 'Road Markings';
  static const String trafficSign = 'Traffic Signs';
  static const String trafficLight = 'Traffic Lights';
  static const String egoMotion = 'Ego Motion';
  static const String worldModel = 'World Model';
  static const String planning = 'Planning';
  static const String collision = 'Collision Prediction';
  static const String decision = 'Decision';
  static const String vehicleSim = 'Vehicle Sim';
  static const String total = 'Total Pipeline';

  static const List<String> ordered = <String>[
    preprocess,
    objectDetection,
    tracking,
    segmentation,
    laneDetection,
    roadEdge,
    depth,
    depthFusion,
    roadMarkings,
    trafficSign,
    trafficLight,
    egoMotion,
    worldModel,
    planning,
    collision,
    decision,
    vehicleSim,
    total,
  ];
}

/// Collects [StageProfile]s and the two frame-rate counters the HUD shows:
/// camera FPS (how fast frames arrive) and processing FPS (how fast the
/// pipeline actually completes them). These differ by design — the camera is
/// never allowed to block on inference.
class PipelineProfiler {
  PipelineProfiler({MonotonicClock? clock, this.window = 64})
      : _clock = clock ?? MonotonicClock();

  final MonotonicClock _clock;
  final int window;
  final Map<String, StageProfile> _stages = <String, StageProfile>{};
  final RingBuffer<int> _cameraFrameMicros = RingBuffer<int>(60);
  final RingBuffer<int> _processedFrameMicros = RingBuffer<int>(60);

  int _droppedFrames = 0;
  int _processedFrames = 0;
  int _capturedFrames = 0;

  StageProfile stage(String name) =>
      _stages.putIfAbsent(name, () => StageProfile(name, window: window));

  List<StageProfile> get stages {
    final List<StageProfile> known = <StageProfile>[
      for (final String n in PipelineStageNames.ordered)
        if (_stages.containsKey(n)) _stages[n]!,
    ];
    final List<StageProfile> extra = _stages.values
        .where((StageProfile s) => !PipelineStageNames.ordered.contains(s.name))
        .toList()
      ..sort((StageProfile a, StageProfile b) => a.name.compareTo(b.name));
    return <StageProfile>[...known, ...extra];
  }

  /// Time a synchronous block.
  R measure<R>(String name, R Function() body) {
    final int t0 = _clock.micros;
    try {
      return body();
    } finally {
      stage(name).record(_clock.micros - t0);
    }
  }

  /// Time an asynchronous block (model inference over a platform channel).
  Future<R> measureAsync<R>(String name, Future<R> Function() body) async {
    final int t0 = _clock.micros;
    try {
      return await body();
    } finally {
      stage(name).record(_clock.micros - t0);
    }
  }

  void onFrameCaptured() {
    _capturedFrames++;
    _cameraFrameMicros.add(_clock.micros);
  }

  void onFrameDropped() => _droppedFrames++;

  void onFrameProcessed(int totalLatencyMicros) {
    _processedFrames++;
    _processedFrameMicros.add(_clock.micros);
    stage(PipelineStageNames.total).record(totalLatencyMicros);
  }

  double get cameraFps => _fps(_cameraFrameMicros);
  double get processingFps => _fps(_processedFrameMicros);

  int get droppedFrames => _droppedFrames;
  int get processedFrames => _processedFrames;
  int get capturedFrames => _capturedFrames;

  double get dropRate =>
      _capturedFrames == 0 ? 0 : _droppedFrames / _capturedFrames;

  double get totalLatencyMs => stage(PipelineStageNames.total).averageMs;

  static double _fps(RingBuffer<int> stamps) {
    if (stamps.length < 2) return 0;
    final int span = stamps.last - stamps.first;
    if (span <= 0) return 0;
    return (stamps.length - 1) * 1e6 / span;
  }

  void reset() {
    for (final StageProfile s in _stages.values) {
      s.reset();
    }
    _cameraFrameMicros.clear();
    _processedFrameMicros.clear();
    _droppedFrames = 0;
    _processedFrames = 0;
    _capturedFrames = 0;
  }

  Map<String, dynamic> toJson() => <String, dynamic>{
        'cameraFps': double.parse(cameraFps.toStringAsFixed(1)),
        'processingFps': double.parse(processingFps.toStringAsFixed(1)),
        'dropped': _droppedFrames,
        'processed': _processedFrames,
        'captured': _capturedFrames,
        'stages': <Map<String, dynamic>>[
          for (final StageProfile s in stages) s.toJson(),
        ],
      };
}
