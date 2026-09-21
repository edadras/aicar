import 'dart:io';

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:path/path.dart' as p;
import 'package:path_provider/path_provider.dart';

import '../../validation/distance_validator.dart';
import '../driving_session.dart';
import '../theme.dart';

/// What the stack's distances are actually worth, measured on the road.
///
/// The number this screen produces is the one claim a simulator cannot make
/// for itself. It is collected with no props and no setup: every parked car
/// and sign post driven past is a free calibration target, because the
/// distance to a stationary object must shrink at exactly the speed the
/// vehicle is travelling — and speed is measured by GPS and the IMU, which
/// know nothing about the camera.
class DistanceAccuracyScreen extends StatefulWidget {
  const DistanceAccuracyScreen({super.key, required this.session});

  final DrivingSession session;

  @override
  State<DistanceAccuracyScreen> createState() =>
      _DistanceAccuracyScreenState();
}

class _DistanceAccuracyScreenState extends State<DistanceAccuracyScreen> {
  String? _exportedTo;

  Future<void> _export() async {
    final DistanceValidator? v =
        widget.session.latestPipeline?.distanceValidator;
    if (v == null || v.sampleCount == 0) return;

    final Directory dir = await getApplicationDocumentsDirectory();
    final String name =
        'distance-accuracy-${DateTime.now().millisecondsSinceEpoch}.csv';
    final File file = File(p.join(dir.path, name));
    await file.writeAsString(v.toCsv());
    if (!mounted) return;
    setState(() => _exportedTo = file.path);
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(content: Text('${v.sampleCount} samples written to $name')),
    );
  }

  @override
  Widget build(BuildContext context) {
    return AnimatedBuilder(
      animation: widget.session,
      builder: (BuildContext context, _) {
        final DistanceValidator? v =
            widget.session.latestPipeline?.distanceValidator;
        final DistanceAccuracyReport report =
            v?.report ?? DistanceAccuracyReport.empty;

        return Scaffold(
          appBar: AppBar(
            title: const Text('Distance accuracy'),
            actions: <Widget>[
              IconButton(
                icon: const Icon(Icons.ios_share),
                tooltip: 'Export CSV',
                onPressed: report.totalSamples == 0 ? null : _export,
              ),
              IconButton(
                icon: const Icon(Icons.restart_alt),
                tooltip: 'Clear samples',
                onPressed: v == null
                    ? null
                    : () => setState(v.reset),
              ),
            ],
          ),
          body: ListView(
            padding: const EdgeInsets.fromLTRB(14, 0, 14, 28),
            children: <Widget>[
              const SectionHeader(
                'How this works',
                subtitle: 'No tape measure, no cones, no surveyed target',
              ),
              const Card(
                child: Padding(
                  padding: EdgeInsets.all(14),
                  child: Text(
                    'For a stationary object, the distance to it must shrink '
                    'at exactly the speed you are travelling — and your speed '
                    'is measured by GPS and the IMU, which know nothing about '
                    'the camera. So every parked car and sign post you drive '
                    'past is a free calibration target, and the ratio of '
                    'measured closure to measured speed is the distance '
                    "estimator's scale error directly.\n\n"
                    'Samples are only taken above 4 m/s, while going roughly '
                    'straight, from objects the tracker is confident are '
                    'stationary. Drive a few minutes of ordinary road with '
                    'parked cars and the numbers below become meaningful.',
                    style: HudTheme.caption,
                  ),
                ),
              ),
              const SectionHeader('Result'),
              Card(
                child: Padding(
                  padding: const EdgeInsets.all(14),
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: <Widget>[
                      Row(
                        mainAxisAlignment: MainAxisAlignment.spaceAround,
                        children: <Widget>[
                          HudReadout(
                            label: 'Samples',
                            value: '${report.totalSamples}',
                            valueColor: report.isUsable
                                ? HudTheme.accent
                                : HudTheme.caution,
                          ),
                          HudReadout(
                            label: 'Scale',
                            value: report.totalSamples == 0
                                ? '—'
                                : report.overallScale.toStringAsFixed(3),
                            valueColor:
                                (report.overallScale - 1).abs() < 0.05
                                    ? HudTheme.accent
                                    : HudTheme.warning,
                          ),
                          HudReadout(
                            label: 'Error at 25 m',
                            value: report.errorAt(25) == null
                                ? '—'
                                : '±${report.errorAt(25)!.toStringAsFixed(1)}',
                            unit: 'm',
                          ),
                          HudReadout(
                            label: 'Targets now',
                            value: '${v?.eligibleTargets ?? 0}',
                          ),
                        ],
                      ),
                      if (!report.isUsable) ...<Widget>[
                        const SizedBox(height: 10),
                        Text(
                          report.totalSamples == 0
                              ? 'No samples yet. Start a drive.'
                              : 'Only ${report.totalSamples} samples — at '
                                  'least 30 before this is worth reading.',
                          style: HudTheme.caption
                              .copyWith(color: HudTheme.caution),
                        ),
                      ],
                      const SizedBox(height: 12),
                      Text(report.diagnosis, style: HudTheme.body),
                    ],
                  ),
                ),
              ),
              const SectionHeader(
                'By distance',
                subtitle: 'A flat profile means height; a rising one, pitch',
              ),
              Card(
                child: Padding(
                  padding: const EdgeInsets.all(14),
                  child: report.bands.isEmpty
                      ? const Text('Nothing measured yet.',
                          style: HudTheme.caption)
                      : Column(
                          children: <Widget>[
                            for (final DistanceBandResult b in report.bands)
                              _BandRow(band: b),
                          ],
                        ),
                ),
              ),
              if (_exportedTo != null) ...<Widget>[
                const SectionHeader('Last export'),
                Card(
                  child: InkWell(
                    onTap: () {
                      Clipboard.setData(ClipboardData(text: _exportedTo!));
                      ScaffoldMessenger.of(context).showSnackBar(
                        const SnackBar(content: Text('Path copied')),
                      );
                    },
                    child: Padding(
                      padding: const EdgeInsets.all(14),
                      child: Text(
                        _exportedTo!,
                        style: HudTheme.caption.copyWith(
                          fontFamily: HudTheme.monoFamily,
                        ),
                      ),
                    ),
                  ),
                ),
              ],
            ],
          ),
        );
      },
    );
  }
}

class _BandRow extends StatelessWidget {
  const _BandRow({required this.band});

  final DistanceBandResult band;

  @override
  Widget build(BuildContext context) {
    final Color colour = !band.isUsable
        ? HudTheme.textDim
        : (band.totalErrorMeters < band.centreMeters * 0.06
            ? HudTheme.accent
            : HudTheme.warning);

    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 4),
      child: Row(
        children: <Widget>[
          SizedBox(
            width: 78,
            child: Text(
              '${band.nearMeters.round()}–${band.farMeters.round()} m',
              style: HudTheme.caption.copyWith(
                fontFamily: HudTheme.monoFamily,
              ),
            ),
          ),
          SizedBox(
            width: 52,
            child: Text('n=${band.sampleCount}', style: HudTheme.caption),
          ),
          Expanded(
            child: Text(
              band.isUsable
                  ? 'x${band.medianScale.toStringAsFixed(3)}'
                  : 'not enough samples',
              style: HudTheme.caption,
            ),
          ),
          Text(
            band.isUsable
                ? '±${band.totalErrorMeters.toStringAsFixed(1)} m'
                : '—',
            style: HudTheme.body.copyWith(color: colour),
          ),
        ],
      ),
    );
  }
}
