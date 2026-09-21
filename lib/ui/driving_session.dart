import 'dart:async';

import 'package:camera/camera.dart';
import 'package:flutter/foundation.dart';
import 'package:package_info_plus/package_info_plus.dart' as pkg;

import '../ai/inference_backend.dart';
import '../ai/model_descriptor.dart';
import '../ai/model_registry.dart';
import '../camera/camera_calibration.dart';
import '../camera/camera_frame.dart';
import '../camera/camera_service.dart';
import '../camera/frame_scheduler.dart';
import '../camera/image_preprocessing.dart';
import '../core/logging.dart';
import '../core/profiling.dart';
import '../navigation/navigation_service.dart';
import '../navigation/route.dart';
import '../pipeline/perception_pipeline.dart';
import '../pipeline/pipeline_config.dart';
import '../pipeline/pipeline_factory.dart';
import '../pipeline/pipeline_result.dart';
import '../recording/recording_schema.dart';
import '../recording/session_recorder.dart';
import '../sensors/ego_motion.dart';
import '../sensors/sensor_hub.dart';
import 'calibration_store.dart';

/// Lifecycle of a live drive.
enum DrivingSessionState {
  idle('Idle'),
  starting('Starting'),
  running('Running'),
  paused('Paused'),
  error('Error');

  const DrivingSessionState(this.label);
  final String label;
}

/// Owns everything a live drive needs and exposes it to the UI.
///
/// Deliberately the only stateful object the widgets talk to: screens read
/// [latest] and call `start`/`stop`, and nothing in `lib/ui` touches a camera,
/// a sensor or a model directly.
class DrivingSession extends ChangeNotifier {
  DrivingSession({
    required this.registry,
    required this.backend,
    CalibrationStore? calibrationStore,
    NavigationService? navigation,
  })  : calibrationStore = calibrationStore ?? CalibrationStore(),
        navigation = navigation ?? NavigationService();

  static const String _tag = 'DrivingSession';

  final ModelRegistry registry;
  final InferenceBackend backend;
  final CalibrationStore calibrationStore;
  final NavigationService navigation;

  final SensorHub sensors = SensorHub();
  late final CameraService camera = CameraService(clock: sensors.clock);
  final FrameScheduler scheduler = FrameScheduler();

  PerceptionPipeline? _pipeline;
  PipelineComposition? _composition;
  SessionRecorder? _recorder;

  PipelineConfig _config = const PipelineConfig();
  CameraCalibration _calibration = CameraCalibration.galaxyS23Default();

  DrivingSessionState _state = DrivingSessionState.idle;
  String? _errorMessage;
  PipelineResult? _latest;
  CameraFrame? _latestFrame;

  StreamSubscription<YuvFrame>? _frameSub;
  StreamSubscription<GeoPosition>? _gpsSub;
  StreamSubscription<ImuSample>? _imuSub;

  String _appVersion = '0.0.0';
  String _deviceModel = 'unknown';
  String _androidVersion = 'unknown';

  // --- Public state -------------------------------------------------------

  DrivingSessionState get state => _state;
  String? get errorMessage => _errorMessage;
  PipelineResult? get latest => _latest;
  CameraFrame? get latestFrame => _latestFrame;
  PipelineComposition? get composition => _composition;
  PipelineConfig get config => _config;
  CameraCalibration get calibration => _calibration;
  PipelineProfiler? get profiler => _pipeline?.profiler;

  /// The live pipeline, exposed so Replay can re-run the very same models and
  /// settings that a live drive would use.
  PerceptionPipeline? get latestPipeline => _pipeline;
  SessionRecorder? get recorder => _recorder;
  bool get isRecording => _recorder?.isRecording ?? false;
  bool get isRunning => _state == DrivingSessionState.running;
  CameraController? get cameraController => camera.controller;

  /// Frame rate the camera is actually delivering.
  double get cameraFps => _pipeline?.profiler.cameraFps ?? 0;

  /// Frame rate the pipeline is actually completing.
  double get processingFps => _pipeline?.profiler.processingFps ?? 0;

  // --- Setup --------------------------------------------------------------

  Future<void> initialise() async {
    try {
      final pkg.PackageInfo info = await pkg.PackageInfo.fromPlatform();
      _appVersion = '${info.version}+${info.buildNumber}';
    } catch (_) {
      // Not fatal; the version only appears in recording metadata.
    }
    _calibration = await calibrationStore.load();
    await registry.refresh();
    notifyListeners();
  }

  void setDeviceInfo({required String model, required String androidVersion}) {
    _deviceModel = model;
    _androidVersion = androidVersion;
  }

  Future<void> applyConfig(PipelineConfig next) async {
    _config = next;
    scheduler.targetFps = next.camera.targetInferenceFps;
    _pipeline?.config = next;
    await camera.applyConfig(next.camera);
    notifyListeners();
  }

  Future<void> applyCalibration(CameraCalibration next) async {
    _calibration = next;
    await calibrationStore.save(next);
    camera.calibration = next;
    _pipeline?.calibration = next;
    notifyListeners();
  }

  /// Rebuild the pipeline — after a model change, for instance.
  Future<void> rebuildPipeline() async {
    await _pipeline?.dispose();
    final (PerceptionPipeline pipeline, PipelineComposition composition) =
        await PipelineFactory(registry: registry, backend: backend)
            .build(calibration: _calibration, config: _config);
    _pipeline = pipeline;
    _composition = composition;
    notifyListeners();
  }

  // --- Drive lifecycle ----------------------------------------------------

  Future<void> start() async {
    if (_state == DrivingSessionState.running ||
        _state == DrivingSessionState.starting) {
      return;
    }
    _setState(DrivingSessionState.starting);
    _errorMessage = null;

    try {
      await camera.initialize();
      camera.calibration = _calibration;
      _calibration = camera.calibration;

      if (_pipeline == null) await rebuildPipeline();
      _pipeline!.calibration = _calibration;

      await sensors.start();
      scheduler.reset();
      scheduler.targetFps = _config.camera.targetInferenceFps;

      _frameSub = camera.frames.listen(_onCameraFrame);
      _gpsSub = sensors.gpsFixes.listen(_onGpsFix);
      _imuSub = sensors.rawImuSamples.listen(_onImuSample);

      await camera.startStream();
      _setState(DrivingSessionState.running);
      Log.info(_tag, 'drive started');
    } catch (e, st) {
      _errorMessage = '$e';
      Log.error(_tag, 'failed to start drive', e, st);
      _setState(DrivingSessionState.error);
    }
  }

  Future<void> stop() async {
    await _frameSub?.cancel();
    await _gpsSub?.cancel();
    await _imuSub?.cancel();
    _frameSub = null;
    _gpsSub = null;
    _imuSub = null;

    await camera.stopStream();
    await sensors.stop();
    if (isRecording) await stopRecording();

    _setState(DrivingSessionState.idle);
    Log.info(_tag, 'drive stopped');
  }

  void pause() {
    if (_state != DrivingSessionState.running) return;
    _setState(DrivingSessionState.paused);
  }

  void resume() {
    if (_state != DrivingSessionState.paused) return;
    _setState(DrivingSessionState.running);
  }

  // --- Frame handling -----------------------------------------------------

  void _onCameraFrame(YuvFrame yuv) {
    _pipeline?.profiler.onFrameCaptured();
    if (_state != DrivingSessionState.running) return;

    final CameraFrame? frame = _toCameraFrame(yuv);
    if (frame == null) return;

    final CameraFrame? accepted = scheduler.offer(frame);
    if (accepted == null) {
      _pipeline?.profiler.onFrameDropped();
      return;
    }
    unawaited(_process(accepted));
  }

  /// Convert and downscale to the configured inference resolution.
  ///
  /// This runs on the main isolate because the platform's image-stream
  /// callback delivers here; it is the one piece of pixel work that cannot be
  /// moved, so it is kept to a single pass and the scheduler ensures it only
  /// happens for frames that will actually be processed.
  CameraFrame? _toCameraFrame(YuvFrame yuv) {
    try {
      final InferenceResolution target = _config.camera.inferenceResolution;
      final Uint8List rgb = ImagePreprocessing.yuv420ToRgb(yuv);
      final Uint8List scaled = ImagePreprocessing.resize(
        rgb,
        yuv.width,
        yuv.height,
        target.width,
        target.height,
      );
      return CameraFrame(
        id: yuv.id,
        timestampMicros: yuv.timestampMicros,
        width: target.width,
        height: target.height,
        bytes: scaled,
        format: PixelFormat.rgb888,
        calibration: _calibration.scaledTo(target.width, target.height),
        sensorRotationDegrees: yuv.sensorRotationDegrees,
      );
    } catch (e) {
      Log.warn(_tag, 'frame conversion failed: $e');
      return null;
    }
  }

  Future<void> _process(CameraFrame frame) async {
    final PerceptionPipeline? pipeline = _pipeline;
    if (pipeline == null) return;

    try {
      final EgoMotionState ego = sensors.stateAt(frame.timestampMicros);
      final RouteProgress? route = navigation.progress;

      final PipelineResult result = await pipeline.process(
        frame: frame,
        ego: ego,
        routeProgress: route,
      );

      _latest = result;
      _latestFrame = frame;
      _recorder?.recordResult(result, frame: frame);
      notifyListeners();
    } catch (e, st) {
      // A failed cycle must not wedge the pipeline: abort the scheduler slot
      // so the next frame is accepted.
      Log.error(_tag, 'pipeline cycle failed', e, st);
      scheduler.abort();
      return;
    } finally {
      final CameraFrame? next = scheduler.complete();
      if (next != null) unawaited(_process(next));
    }
  }

  void _onGpsFix(GeoPosition fix) {
    _recorder?.recordGps(fix);
    unawaited(navigation.updatePosition(fix));
  }

  void _onImuSample(ImuSample sample) => _recorder?.recordImu(sample);

  // --- Recording ----------------------------------------------------------

  Future<void> startRecording({
    RecordingMode mode = RecordingMode.withFrames,
    String? notes,
  }) async {
    if (isRecording) return;
    final SessionRecorder recorder = SessionRecorder(mode: mode);
    await recorder.start((String id) => SessionHeader(
          sessionId: id,
          startedAt: DateTime.now(),
          schemaVersion: recordingSchemaVersion,
          appVersion: _appVersion,
          deviceModel: _deviceModel,
          androidVersion: _androidVersion,
          calibration: _calibration.toJson(),
          models: <String, String>{
            for (final ModelRole role in ModelRole.values)
              role.name: registry.selectedFor(role)?.id ?? 'none',
          },
          pipelineConfig: <String, dynamic>{
            'inferenceResolution':
                _config.camera.inferenceResolution.label,
            'targetFps': _config.camera.targetInferenceFps,
            'toggles': _config.toggles.toJson(),
          },
          notes: notes,
        ));
    _recorder = recorder;
    notifyListeners();
  }

  Future<SessionFooter?> stopRecording() async {
    final SessionRecorder? recorder = _recorder;
    if (recorder == null) return null;
    final SessionFooter? footer =
        await recorder.stop(profiler: _pipeline?.profiler);
    _recorder = null;
    notifyListeners();
    return footer;
  }

  void addMarker(String label) => _recorder?.recordMarker(label);

  void _setState(DrivingSessionState next) {
    _state = next;
    notifyListeners();
  }

  @override
  Future<void> dispose() async {
    await stop();
    await _pipeline?.dispose();
    await camera.close();
    await sensors.dispose();
    await navigation.dispose();
    super.dispose();
  }
}
