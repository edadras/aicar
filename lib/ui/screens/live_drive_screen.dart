import 'package:camera/camera.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';

import '../../pipeline/pipeline_result.dart';
import '../../recording/session_recorder.dart';
import '../driving_session.dart';
import '../hud/hud_panels.dart';
import '../hud/perception_overlay.dart';
import '../theme.dart';
import 'developer_debug_screen.dart';

/// The main HUD: camera, overlays and the simulated controls.
class LiveDriveScreen extends StatefulWidget {
  const LiveDriveScreen({super.key});

  @override
  State<LiveDriveScreen> createState() => _LiveDriveScreenState();
}

class _LiveDriveScreenState extends State<LiveDriveScreen>
    with SingleTickerProviderStateMixin {
  late final AnimationController _pulse = AnimationController(
    vsync: this,
    duration: const Duration(milliseconds: 900),
  )..repeat(reverse: true);

  OverlayOptions _options = const OverlayOptions();
  bool _showDebug = false;

  @override
  void initState() {
    super.initState();
    // A driving HUD is landscape and must not sleep.
    SystemChrome.setPreferredOrientations(<DeviceOrientation>[
      DeviceOrientation.landscapeLeft,
      DeviceOrientation.landscapeRight,
    ]);
    SystemChrome.setEnabledSystemUIMode(SystemUiMode.immersiveSticky);
  }

  @override
  void dispose() {
    _pulse.dispose();
    SystemChrome.setPreferredOrientations(DeviceOrientation.values);
    SystemChrome.setEnabledSystemUIMode(SystemUiMode.edgeToEdge);
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final DrivingSession session = context.watch<DrivingSession>();
    final PipelineResult? result = session.latest;

    return Scaffold(
      backgroundColor: Colors.black,
      body: Stack(
        fit: StackFit.expand,
        children: <Widget>[
          _buildCameraLayer(session),
          if (result != null) _buildOverlay(session, result),
          _buildTopBar(session, result),
          if (result != null) _buildSidePanels(session, result),
          Positioned(
            left: 0,
            right: 0,
            bottom: 0,
            child: result == null
                ? _buildWaiting(session)
                : ControlStrip(
                    result: result,
                    steeringRatio: session.config.vehicle.steeringRatio,
                    steeringLimitDegrees: session.config.steeringLimitDegrees,
                  ),
          ),
          if (_showDebug && result != null)
            Positioned(
              right: 12,
              bottom: 130,
              child: DebugOverlayCard(session: session, result: result),
            ),
        ],
      ),
    );
  }

  Widget _buildCameraLayer(DrivingSession session) {
    final CameraController? controller = session.cameraController;
    if (controller == null || !controller.value.isInitialized) {
      return Container(
        color: Colors.black,
        alignment: Alignment.center,
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: <Widget>[
            const Icon(Icons.videocam_off_outlined,
                size: 44, color: HudTheme.textDim),
            const SizedBox(height: 12),
            Text(
              session.state == DrivingSessionState.error
                  ? 'Camera unavailable'
                  : 'Camera not started',
              style: HudTheme.body,
            ),
            if (session.errorMessage != null) ...<Widget>[
              const SizedBox(height: 6),
              SizedBox(
                width: 420,
                child: Text(
                  session.errorMessage!,
                  style: HudTheme.caption,
                  textAlign: TextAlign.center,
                ),
              ),
            ],
          ],
        ),
      );
    }

    // The preview's aspect ratio is reported in sensor orientation; covering
    // the screen keeps the overlay geometry aligned with what is drawn.
    return FittedBox(
      fit: BoxFit.cover,
      child: SizedBox(
        width: controller.value.previewSize?.height ?? 1280,
        height: controller.value.previewSize?.width ?? 720,
        child: CameraPreview(controller),
      ),
    );
  }

  Widget _buildOverlay(DrivingSession session, PipelineResult result) {
    return AnimatedBuilder(
      animation: _pulse,
      builder: (BuildContext context, _) => CustomPaint(
        painter: PerceptionOverlayPainter(
          world: result.world,
          path: result.path,
          options: _options,
          pulse: _pulse.value,
        ),
      ),
    );
  }

  Widget _buildTopBar(DrivingSession session, PipelineResult? result) {
    return Positioned(
      top: 0,
      left: 0,
      right: 0,
      child: SafeArea(
        child: Padding(
          padding: const EdgeInsets.fromLTRB(12, 8, 12, 0),
          child: Column(
            children: <Widget>[
              Row(
                children: <Widget>[
                  _RoundButton(
                    icon: Icons.arrow_back,
                    onPressed: () => Navigator.of(context).pop(),
                  ),
                  const SizedBox(width: 8),
                  const SimulationOnlyBadge(compact: true),
                  const SizedBox(width: 8),
                  if (session.isRecording)
                    HudBadge(
                      text: 'REC ${session.recorder?.frameCount ?? 0}',
                      color: HudTheme.critical,
                      filled: true,
                      icon: Icons.fiber_manual_record,
                    ),
                  const Spacer(),
                  HudBadge(
                    text: '${session.processingFps.toStringAsFixed(1)} FPS AI',
                    color: session.processingFps >= 10
                        ? HudTheme.accent
                        : HudTheme.caution,
                  ),
                  const SizedBox(width: 6),
                  HudBadge(
                    text: '${session.cameraFps.toStringAsFixed(0)} FPS CAM',
                    color: HudTheme.textSecondary,
                  ),
                  const SizedBox(width: 8),
                  _RoundButton(
                    icon: _showDebug
                        ? Icons.bug_report
                        : Icons.bug_report_outlined,
                    onPressed: () => setState(() => _showDebug = !_showDebug),
                  ),
                  const SizedBox(width: 6),
                  _RoundButton(
                    icon: Icons.layers_outlined,
                    onPressed: _showOverlaySheet,
                  ),
                  const SizedBox(width: 6),
                  _RoundButton(
                    icon: session.isRecording
                        ? Icons.stop_circle_outlined
                        : Icons.radio_button_checked,
                    color: session.isRecording
                        ? HudTheme.critical
                        : HudTheme.textPrimary,
                    onPressed: () => _toggleRecording(session),
                  ),
                ],
              ),
              if (result != null) ...<Widget>[
                const SizedBox(height: 8),
                HazardBanner(world: result.world, pulse: _pulse.value),
              ],
            ],
          ),
        ),
      ),
    );
  }

  Widget _buildSidePanels(DrivingSession session, PipelineResult result) {
    return Positioned(
      left: 12,
      right: 12,
      top: 96,
      bottom: 130,
      child: IgnorePointer(
        child: Row(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: <Widget>[
            Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              mainAxisSize: MainAxisSize.min,
              children: <Widget>[
                DecisionPanel(decision: result.decision),
                const SizedBox(height: 8),
                AutonomyConfidencePanel(
                  autonomy: result.world.autonomy,
                  dense: true,
                ),
                const SizedBox(height: 8),
                TurnSignalIndicators(
                  state: result.command.turnSignal,
                  timestampMicros: result.world.timestampMicros,
                ),
              ],
            ),
            const Spacer(),
            RoadContextPanel(world: result.world),
          ],
        ),
      ),
    );
  }

  Widget _buildWaiting(DrivingSession session) {
    return Container(
      padding: const EdgeInsets.all(20),
      color: HudTheme.background.withValues(alpha: 0.9),
      child: Row(
        children: <Widget>[
          const SizedBox(
            width: 18,
            height: 18,
            child: CircularProgressIndicator(strokeWidth: 2),
          ),
          const SizedBox(width: 14),
          Expanded(
            child: Text(
              switch (session.state) {
                DrivingSessionState.starting =>
                  'Starting camera and sensors…',
                DrivingSessionState.error =>
                  session.errorMessage ?? 'Failed to start',
                _ => 'Waiting for the first processed frame…',
              },
              style: HudTheme.body,
            ),
          ),
        ],
      ),
    );
  }

  Future<void> _toggleRecording(DrivingSession session) async {
    if (session.isRecording) {
      await session.stopRecording();
    } else {
      await session.startRecording(mode: RecordingMode.withFrames);
    }
  }

  void _showOverlaySheet() {
    showModalBottomSheet<void>(
      context: context,
      backgroundColor: HudTheme.surface,
      builder: (BuildContext context) => StatefulBuilder(
        builder: (BuildContext context, StateSetter setSheetState) {
          void toggle(OverlayOptions next) {
            setSheetState(() => _options = next);
            setState(() {});
          }

          return SingleChildScrollView(
            child: Column(
              mainAxisSize: MainAxisSize.min,
              children: <Widget>[
                const Padding(
                  padding: EdgeInsets.all(16),
                  child: SectionHeader('Overlays',
                      subtitle: 'A HUD that shows everything shows nothing'),
                ),
                SwitchListTile(
                  title: const Text('Bounding boxes'),
                  value: _options.boundingBoxes,
                  onChanged: (bool v) =>
                      toggle(_options.copyWith(boundingBoxes: v)),
                ),
                SwitchListTile(
                  title: const Text('Lane lines'),
                  value: _options.laneLines,
                  onChanged: (bool v) =>
                      toggle(_options.copyWith(laneLines: v)),
                ),
                SwitchListTile(
                  title: const Text('Drivable area'),
                  value: _options.drivableArea,
                  onChanged: (bool v) =>
                      toggle(_options.copyWith(drivableArea: v)),
                ),
                SwitchListTile(
                  title: const Text('Planned path'),
                  value: _options.plannedPath,
                  onChanged: (bool v) =>
                      toggle(_options.copyWith(plannedPath: v)),
                ),
                SwitchListTile(
                  title: const Text('Road edges'),
                  value: _options.roadEdges,
                  onChanged: (bool v) =>
                      toggle(_options.copyWith(roadEdges: v)),
                ),
                SwitchListTile(
                  title: const Text('Road markings'),
                  subtitle: const Text(
                      'Crossings, stop lines, speed bumps and the junction '
                      'they imply'),
                  value: _options.roadMarkings,
                  onChanged: (bool v) =>
                      toggle(_options.copyWith(roadMarkings: v)),
                ),
                SwitchListTile(
                  title: const Text('Predicted object paths'),
                  value: _options.predictedPaths,
                  onChanged: (bool v) =>
                      toggle(_options.copyWith(predictedPaths: v)),
                ),
                SwitchListTile(
                  title: const Text('Horizon line'),
                  subtitle: const Text('Useful when checking calibration'),
                  value: _options.horizonLine,
                  onChanged: (bool v) =>
                      toggle(_options.copyWith(horizonLine: v)),
                ),
                const SizedBox(height: 12),
              ],
            ),
          );
        },
      ),
    );
  }
}

class _RoundButton extends StatelessWidget {
  const _RoundButton({
    required this.icon,
    required this.onPressed,
    this.color = HudTheme.textPrimary,
  });

  final IconData icon;
  final VoidCallback onPressed;
  final Color color;

  @override
  Widget build(BuildContext context) {
    return Material(
      color: HudTheme.background.withValues(alpha: 0.7),
      shape: const CircleBorder(
        side: BorderSide(color: HudTheme.outline),
      ),
      child: InkWell(
        customBorder: const CircleBorder(),
        onTap: onPressed,
        child: Padding(
          padding: const EdgeInsets.all(9),
          child: Icon(icon, size: 19, color: color),
        ),
      ),
    );
  }
}
