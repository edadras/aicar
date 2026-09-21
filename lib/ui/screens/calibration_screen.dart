import 'package:camera/camera.dart';
import 'package:flutter/material.dart';
import 'package:provider/provider.dart';

import '../../camera/camera_calibration.dart';
import '../../core/geometry.dart';
import '../driving_session.dart';
import '../theme.dart';

/// Guided camera calibration.
///
/// Calibration is the single highest-leverage thing a user can do: every
/// distance, every lane offset and every time-to-collision is computed
/// through it. The wizard therefore asks for the two measurements that matter
/// most (height and pitch) and lets the user verify the result against the
/// live image rather than trusting a number they typed.
class CalibrationScreen extends StatefulWidget {
  const CalibrationScreen({super.key});

  @override
  State<CalibrationScreen> createState() => _CalibrationScreenState();
}

class _CalibrationScreenState extends State<CalibrationScreen> {
  int _step = 0;
  late CameraCalibration _draft;
  bool _started = false;

  @override
  void initState() {
    super.initState();
    _draft = context.read<DrivingSession>().calibration;
  }

  @override
  Widget build(BuildContext context) {
    final DrivingSession session = context.watch<DrivingSession>();

    return Scaffold(
      appBar: AppBar(
        title: const Text('Calibration'),
        actions: <Widget>[
          TextButton(
            onPressed: _finish,
            child: const Text('Save'),
          ),
        ],
      ),
      body: Column(
        children: <Widget>[
          LinearProgressIndicator(
            value: (_step + 1) / 5,
            backgroundColor: HudTheme.outline,
            minHeight: 3,
          ),
          Expanded(
            child: IndexedStack(
              index: _step,
              children: <Widget>[
                _mountStep(),
                _heightStep(),
                _pitchStep(session),
                _offsetStep(),
                _verifyStep(session),
              ],
            ),
          ),
          _navigation(),
        ],
      ),
    );
  }

  Widget _navigation() => Container(
        padding: const EdgeInsets.all(14),
        decoration: const BoxDecoration(
          border: Border(top: BorderSide(color: HudTheme.outline)),
        ),
        child: Row(
          children: <Widget>[
            if (_step > 0)
              OutlinedButton(
                onPressed: () => setState(() => _step--),
                child: const Text('Back'),
              ),
            const Spacer(),
            FilledButton(
              onPressed: _step < 4
                  ? () => setState(() => _step++)
                  : _finish,
              child: Text(_step < 4 ? 'Next' : 'Finish'),
            ),
          ],
        ),
      );

  Future<void> _finish() async {
    final DrivingSession session = context.read<DrivingSession>();
    final NavigatorState navigator = Navigator.of(context);
    await session.applyCalibration(
      _draft.copyWith(isCalibrated: true, calibratedAt: DateTime.now()),
    );
    if (mounted) navigator.pop();
  }

  Widget _stepScaffold({
    required String title,
    required String body,
    required List<Widget> children,
  }) =>
      ListView(
        padding: const EdgeInsets.all(16),
        children: <Widget>[
          Text(title,
              style: HudTheme.body.copyWith(
                  fontSize: 18, fontWeight: FontWeight.w600)),
          const SizedBox(height: 8),
          Text(body, style: HudTheme.caption),
          const SizedBox(height: 18),
          ...children,
        ],
      );

  Widget _mountStep() => _stepScaffold(
        title: 'Mount the phone',
        body: 'Fix the phone in a cradle on the dashboard or windscreen, '
            'landscape, with the rear camera looking straight down the road. '
            'It must not move during the drive: everything the system '
            'measures is relative to where the camera is now.\n\n'
            'Two things to get right:\n'
            '• the phone should be as close to level side-to-side as you can '
            'manage (any residual lean is corrected in step 4);\n'
            '• the bonnet should occupy no more than the bottom fifth of the '
            'frame, or the usable range is cut short.',
        children: <Widget>[
          Card(
            child: ListTile(
              leading: const Icon(Icons.info_outline, color: HudTheme.info),
              title: const Text('Why this matters', style: HudTheme.body),
              subtitle: Text(
                'With default values a car at 25 m can read anywhere between '
                '20 m and 32 m. After calibration the same estimate is good '
                'to about a metre.',
                style: HudTheme.caption,
              ),
            ),
          ),
        ],
      );

  Widget _heightStep() => _stepScaffold(
        title: 'Camera height',
        body: 'Measure from the road surface to the camera lens. A tape '
            'measure is best; most cars sit between 1.1 m and 1.4 m with a '
            'dashboard mount.\n\n'
            'This sets the scale of every ground-plane distance: a 10 % '
            'error here is a 10 % error in every distance the system reports.',
        children: <Widget>[
          _SliderRow(
            label: 'Height above the road',
            value: _draft.cameraHeightMeters,
            min: 0.6,
            max: 2.2,
            divisions: 160,
            format: (double v) => '${v.toStringAsFixed(2)} m',
            onChanged: (double v) => setState(
                () => _draft = _draft.copyWith(cameraHeightMeters: v)),
          ),
        ],
      );

  Widget _pitchStep(DrivingSession session) => _stepScaffold(
        title: 'Camera pitch',
        body: 'Adjust until the horizon line sits exactly on the real '
            'horizon — where the road meets the sky on a straight, flat '
            'stretch.\n\n'
            'Pitch is the hardest parameter to measure and the one that most '
            'affects far-field distance, which is why it is set visually '
            'rather than typed in.',
        children: <Widget>[
          _PreviewWithHorizon(
            calibration: _draft,
            controller: session.cameraController,
            onStartCamera: _started
                ? null
                : () async {
                    setState(() => _started = true);
                    await session.start();
                  },
          ),
          const SizedBox(height: 14),
          _SliderRow(
            label: 'Downward pitch',
            value: _draft.pitchDegrees,
            min: -10,
            max: 20,
            divisions: 300,
            format: (double v) => '${v.toStringAsFixed(1)}°',
            onChanged: (double v) =>
                setState(() => _draft = _draft.copyWith(pitchDegrees: v)),
          ),
          _SliderRow(
            label: 'Horizontal field of view',
            value: _draft.horizontalFovDegrees,
            min: 50,
            max: 110,
            divisions: 120,
            format: (double v) => '${v.toStringAsFixed(1)}°',
            onChanged: (double v) => setState(
                () => _draft = _draft.copyWith(horizontalFovDegrees: v)),
          ),
        ],
      );

  Widget _offsetStep() => _stepScaffold(
        title: 'Position in the vehicle',
        body: 'Where the camera sits relative to the vehicle centreline, and '
            'how far it leans.\n\n'
            'The lateral offset is what makes "the car is 0.4 m left of the '
            'lane centre" mean the *car*, not the phone. Roll corrects a '
            'phone that is not quite level, which otherwise tilts every lane '
            'line the system draws.',
        children: <Widget>[
          _SliderRow(
            label: 'Right of the vehicle centreline',
            value: _draft.lateralOffsetMeters,
            min: -1.2,
            max: 1.2,
            divisions: 240,
            format: (double v) => '${v >= 0 ? '+' : ''}'
                '${v.toStringAsFixed(2)} m',
            onChanged: (double v) => setState(
                () => _draft = _draft.copyWith(lateralOffsetMeters: v)),
          ),
          _SliderRow(
            label: 'Roll (phone leaning right)',
            value: _draft.rollDegrees,
            min: -12,
            max: 12,
            divisions: 240,
            format: (double v) => '${v.toStringAsFixed(1)}°',
            onChanged: (double v) =>
                setState(() => _draft = _draft.copyWith(rollDegrees: v)),
          ),
          _SliderRow(
            label: 'Yaw (camera pointing off-axis)',
            value: _draft.yawDegrees,
            min: -12,
            max: 12,
            divisions: 240,
            format: (double v) => '${v.toStringAsFixed(1)}°',
            onChanged: (double v) =>
                setState(() => _draft = _draft.copyWith(yawDegrees: v)),
          ),
          _SliderRow(
            label: 'Front axle to camera',
            value: _draft.longitudinalOffsetMeters,
            min: 0.5,
            max: 4.0,
            divisions: 140,
            format: (double v) => '${v.toStringAsFixed(2)} m',
            onChanged: (double v) => setState(() =>
                _draft = _draft.copyWith(longitudinalOffsetMeters: v)),
          ),
        ],
      );

  Widget _verifyStep(DrivingSession session) {
    // Distances the calibration implies, so the user can sanity-check them
    // against something they can see.
    final List<double> rows = <double>[5, 10, 20, 30, 50];
    return _stepScaffold(
      title: 'Check the result',
      body: 'These are the image rows where the calibration says each '
          'distance falls. Park behind a car and check that the number under '
          'its wheels matches the real gap — that single check catches almost '
          'every calibration mistake.',
      children: <Widget>[
        _PreviewWithHorizon(
          calibration: _draft,
          controller: session.cameraController,
          distanceMarkers: rows,
          onStartCamera: _started
              ? null
              : () async {
                  setState(() => _started = true);
                  await session.start();
                },
        ),
        const SizedBox(height: 16),
        Card(
          child: Padding(
            padding: const EdgeInsets.all(14),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: <Widget>[
                Text('Derived values', style: HudTheme.hudLabel),
                const SizedBox(height: 8),
                _kv('Focal length',
                    '${_draft.fx.toStringAsFixed(1)} px'),
                _kv('Vertical field of view',
                    '${_draft.verticalFovDegrees.toStringAsFixed(1)}°'),
                _kv('Horizon row',
                    '${_draft.horizonY.toStringAsFixed(0)} '
                        '/ ${_draft.imageHeight}'),
                _kv(
                  'Ground resolution at 30 m',
                  () {
                    final PixelPoint? p =
                        _draft.projectGroundToImage(const Vec2(0, 30));
                    if (p == null) return 'not visible';
                    final double? res =
                        _draft.groundResolutionAtRow(p.v);
                    return res == null
                        ? 'not visible'
                        : '${res.toStringAsFixed(2)} m / pixel';
                  }(),
                ),
                _kv(
                  'Usable ground range',
                  '${_usableRange(_draft).toStringAsFixed(0)} m',
                ),
              ],
            ),
          ),
        ),
      ],
    );
  }

  /// Distance at which one pixel of contact-point error is worth more than
  /// two metres — beyond that, ground-plane distance stops being useful.
  double _usableRange(CameraCalibration c) {
    for (double d = 5; d < 150; d += 1) {
      final PixelPoint? p = c.projectGroundToImage(Vec2(0, d));
      if (p == null) return d;
      final double? res = c.groundResolutionAtRow(p.v);
      if (res == null || res > 2.0) return d;
    }
    return 150;
  }

  Widget _kv(String k, String v) => Padding(
        padding: const EdgeInsets.symmetric(vertical: 3),
        child: Row(
          mainAxisAlignment: MainAxisAlignment.spaceBetween,
          children: <Widget>[
            Text(k, style: HudTheme.caption),
            Text(v,
                style: HudTheme.body.copyWith(
                    fontFamily: HudTheme.monoFamily, fontSize: 13)),
          ],
        ),
      );
}

class _SliderRow extends StatelessWidget {
  const _SliderRow({
    required this.label,
    required this.value,
    required this.min,
    required this.max,
    required this.divisions,
    required this.format,
    required this.onChanged,
  });

  final String label;
  final double value;
  final double min;
  final double max;
  final int divisions;
  final String Function(double) format;
  final ValueChanged<double> onChanged;

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.only(bottom: 10),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: <Widget>[
          Row(
            mainAxisAlignment: MainAxisAlignment.spaceBetween,
            children: <Widget>[
              Expanded(child: Text(label, style: HudTheme.caption)),
              Text(format(value),
                  style: HudTheme.body.copyWith(
                      fontFamily: HudTheme.monoFamily,
                      fontWeight: FontWeight.w600)),
            ],
          ),
          Slider(
            value: value.clamp(min, max),
            min: min,
            max: max,
            divisions: divisions,
            onChanged: onChanged,
          ),
        ],
      ),
    );
  }
}

/// Camera preview with the calibration's horizon and distance markers drawn
/// on it, so the numbers can be checked against reality.
class _PreviewWithHorizon extends StatelessWidget {
  const _PreviewWithHorizon({
    required this.calibration,
    required this.controller,
    this.distanceMarkers = const <double>[],
    this.onStartCamera,
  });

  final CameraCalibration calibration;
  final CameraController? controller;
  final List<double> distanceMarkers;
  final Future<void> Function()? onStartCamera;

  @override
  Widget build(BuildContext context) {
    return AspectRatio(
      aspectRatio: 16 / 9,
      child: ClipRRect(
        borderRadius: BorderRadius.circular(12),
        child: Stack(
          fit: StackFit.expand,
          children: <Widget>[
            if (controller != null && controller!.value.isInitialized)
              FittedBox(
                fit: BoxFit.cover,
                child: SizedBox(
                  width: controller!.value.previewSize?.height ?? 1280,
                  height: controller!.value.previewSize?.width ?? 720,
                  child: CameraPreview(controller!),
                ),
              )
            else
              Container(
                color: HudTheme.surface,
                alignment: Alignment.center,
                child: onStartCamera == null
                    ? const Text('Camera starting…',
                        style: HudTheme.caption)
                    : FilledButton.icon(
                        onPressed: onStartCamera,
                        icon: const Icon(Icons.videocam_outlined),
                        label: const Text('Start camera'),
                      ),
              ),
            CustomPaint(
              painter: _CalibrationGuidePainter(
                calibration: calibration,
                distanceMarkers: distanceMarkers,
              ),
            ),
          ],
        ),
      ),
    );
  }
}

class _CalibrationGuidePainter extends CustomPainter {
  const _CalibrationGuidePainter({
    required this.calibration,
    required this.distanceMarkers,
  });

  final CameraCalibration calibration;
  final List<double> distanceMarkers;

  @override
  void paint(Canvas canvas, Size size) {
    final double horizonY =
        calibration.horizonYNormalized * size.height;

    canvas.drawLine(
      Offset(0, horizonY),
      Offset(size.width, horizonY),
      Paint()
        ..strokeWidth = 2
        ..color = HudTheme.caution,
    );
    _label(canvas, Offset(6, horizonY - 16), 'HORIZON', HudTheme.caution);

    // Centreline, to check the yaw.
    final double centreX =
        calibration.cx / calibration.imageWidth * size.width;
    canvas.drawLine(
      Offset(centreX, horizonY),
      Offset(centreX, size.height),
      Paint()
        ..strokeWidth = 1
        ..color = HudTheme.info.withValues(alpha: 0.6),
    );

    for (final double distance in distanceMarkers) {
      final PixelPoint? left = calibration
          .projectGroundToImage(Vec2(-1.75, distance));
      final PixelPoint? right = calibration
          .projectGroundToImage(Vec2(1.75, distance));
      if (left == null || right == null) continue;

      final Offset a = Offset(
        left.u / calibration.imageWidth * size.width,
        left.v / calibration.imageHeight * size.height,
      );
      final Offset b = Offset(
        right.u / calibration.imageWidth * size.width,
        right.v / calibration.imageHeight * size.height,
      );
      if (a.dy < horizonY || a.dy > size.height) continue;

      canvas.drawLine(
        a,
        b,
        Paint()
          ..strokeWidth = 1.5
          ..color = HudTheme.accent.withValues(alpha: 0.85),
      );
      _label(canvas, Offset(b.dx + 4, b.dy - 8),
          '${distance.toStringAsFixed(0)} m', HudTheme.accent);
    }

    // A 3.5 m lane, drawn to check the lateral scale.
    final List<Offset> leftLane = <Offset>[];
    final List<Offset> rightLane = <Offset>[];
    for (double d = 4; d <= 45; d += 2) {
      final PixelPoint? l =
          calibration.projectGroundToImage(Vec2(-1.75, d));
      final PixelPoint? r =
          calibration.projectGroundToImage(Vec2(1.75, d));
      if (l != null) {
        leftLane.add(Offset(l.u / calibration.imageWidth * size.width,
            l.v / calibration.imageHeight * size.height));
      }
      if (r != null) {
        rightLane.add(Offset(r.u / calibration.imageWidth * size.width,
            r.v / calibration.imageHeight * size.height));
      }
    }
    final Paint lanePaint = Paint()
      ..style = PaintingStyle.stroke
      ..strokeWidth = 1.5
      ..color = HudTheme.laneLine.withValues(alpha: 0.7);
    for (final List<Offset> lane in <List<Offset>>[leftLane, rightLane]) {
      if (lane.length < 2) continue;
      final Path path = Path()..moveTo(lane.first.dx, lane.first.dy);
      for (final Offset o in lane.skip(1)) {
        path.lineTo(o.dx, o.dy);
      }
      canvas.drawPath(path, lanePaint);
    }
  }

  void _label(Canvas canvas, Offset at, String text, Color color) {
    final TextPainter painter = TextPainter(
      text: TextSpan(
        text: text,
        style: HudTheme.overlayLabel.copyWith(color: color),
      ),
      textDirection: TextDirection.ltr,
    )..layout();
    canvas.drawRect(
      Rect.fromLTWH(at.dx - 2, at.dy - 1, painter.width + 4,
          painter.height + 2),
      Paint()..color = Colors.black.withValues(alpha: 0.6),
    );
    painter.paint(canvas, at);
  }

  @override
  bool shouldRepaint(_CalibrationGuidePainter old) =>
      old.calibration.pitchDegrees != calibration.pitchDegrees ||
      old.calibration.cameraHeightMeters != calibration.cameraHeightMeters ||
      old.calibration.horizontalFovDegrees !=
          calibration.horizontalFovDegrees ||
      old.calibration.rollDegrees != calibration.rollDegrees ||
      old.calibration.yawDegrees != calibration.yawDegrees ||
      old.calibration.lateralOffsetMeters !=
          calibration.lateralOffsetMeters ||
      old.distanceMarkers.length != distanceMarkers.length;
}
