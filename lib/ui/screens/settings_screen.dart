import 'package:flutter/material.dart';
import 'package:provider/provider.dart';

import 'package:camera/camera.dart' show ResolutionPreset;

import '../../core/safety.dart';
import '../../navigation/route_provider.dart';
import '../../recording/session_store.dart';
import '../../simulation/vehicle_state.dart';
import '../driving_session.dart';
import '../hud/hud_panels.dart';
import '../theme.dart';

/// Application settings: capture, vehicle model, storage and safety.
class SettingsScreen extends StatefulWidget {
  const SettingsScreen({super.key});

  @override
  State<SettingsScreen> createState() => _SettingsScreenState();
}

class _SettingsScreenState extends State<SettingsScreen> {
  final SessionStore _store = SessionStore();
  int? _recordingBytes;
  bool _offlineRouting = false;

  @override
  void initState() {
    super.initState();
    _store.totalSizeBytes().then((int bytes) {
      if (mounted) setState(() => _recordingBytes = bytes);
    });
  }

  @override
  Widget build(BuildContext context) {
    final DrivingSession session = context.watch<DrivingSession>();
    final VehicleParameters vehicle = session.config.vehicle;

    return Scaffold(
      appBar: AppBar(title: const Text('Settings')),
      body: ListView(
        padding: const EdgeInsets.fromLTRB(14, 0, 14, 28),
        children: <Widget>[
          const SectionHeader('Capture'),
          Card(
            child: Padding(
              padding: const EdgeInsets.all(14),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: <Widget>[
                  Text('Capture resolution', style: HudTheme.hudLabel),
                  const SizedBox(height: 8),
                  SegmentedButton<ResolutionPreset>(
                    segments: const <ButtonSegment<ResolutionPreset>>[
                      ButtonSegment<ResolutionPreset>(
                          value: ResolutionPreset.medium, label: Text('720p')),
                      ButtonSegment<ResolutionPreset>(
                          value: ResolutionPreset.high, label: Text('HD')),
                      ButtonSegment<ResolutionPreset>(
                          value: ResolutionPreset.veryHigh,
                          label: Text('1080p')),
                    ],
                    selected: <ResolutionPreset>{
                      session.config.camera.captureResolution
                    },
                    onSelectionChanged: (Set<ResolutionPreset> s) =>
                        session.applyConfig(
                      session.config.copyWith(
                        camera: session.config.camera
                            .copyWith(captureResolution: s.first),
                      ),
                    ),
                  ),
                  const SizedBox(height: 14),
                  SwitchListTile(
                    contentPadding: EdgeInsets.zero,
                    title: const Text('Lock exposure while driving',
                        style: HudTheme.body),
                    subtitle: const Text(
                      'Stops auto-exposure hunting between sky and asphalt, '
                      'which visibly flickers detections. Costs highlight '
                      'detail entering a tunnel.',
                      style: HudTheme.caption,
                    ),
                    value: session.config.camera.lockExposureWhileDriving,
                    onChanged: (bool v) => session.applyConfig(
                      session.config.copyWith(
                        camera: session.config.camera
                            .copyWith(lockExposureWhileDriving: v),
                      ),
                    ),
                  ),
                ],
              ),
            ),
          ),

          const SectionHeader(
            'Simulated vehicle',
            subtitle: 'The trajectory is only meaningful if the model roughly '
                'matches the car the phone is in',
          ),
          Card(
            child: Padding(
              padding: const EdgeInsets.all(14),
              child: Column(
                children: <Widget>[
                  _Slider(
                    label: 'Wheelbase',
                    value: vehicle.wheelbaseMeters,
                    min: 2.0,
                    max: 3.6,
                    format: (double v) => '${v.toStringAsFixed(2)} m',
                    onChanged: (double v) => session.applyConfig(
                      session.config.copyWith(
                        vehicle: VehicleParameters(
                          wheelbaseMeters: v,
                          massKg: vehicle.massKg,
                          maxSteeringAngleDegrees:
                              vehicle.maxSteeringAngleDegrees,
                          steeringRatio: vehicle.steeringRatio,
                          maxPowerKw: vehicle.maxPowerKw,
                        ),
                      ),
                    ),
                  ),
                  _Slider(
                    label: 'Mass',
                    value: vehicle.massKg,
                    min: 900,
                    max: 3000,
                    format: (double v) => '${v.round()} kg',
                    onChanged: (double v) => session.applyConfig(
                      session.config.copyWith(
                        vehicle: VehicleParameters(
                          wheelbaseMeters: vehicle.wheelbaseMeters,
                          massKg: v,
                          maxSteeringAngleDegrees:
                              vehicle.maxSteeringAngleDegrees,
                          steeringRatio: vehicle.steeringRatio,
                          maxPowerKw: vehicle.maxPowerKw,
                        ),
                      ),
                    ),
                  ),
                  _Slider(
                    label: 'Maximum road-wheel angle',
                    value: vehicle.maxSteeringAngleDegrees,
                    min: 20,
                    max: 45,
                    format: (double v) => '${v.toStringAsFixed(0)}°',
                    onChanged: (double v) => session.applyConfig(
                      session.config.copyWith(
                        vehicle: VehicleParameters(
                          wheelbaseMeters: vehicle.wheelbaseMeters,
                          massKg: vehicle.massKg,
                          maxSteeringAngleDegrees: v,
                          steeringRatio: vehicle.steeringRatio,
                          maxPowerKw: vehicle.maxPowerKw,
                        ),
                        steeringLimitDegrees: v,
                      ),
                    ),
                  ),
                  _Slider(
                    label: 'Steering ratio (wheel : road wheel)',
                    value: vehicle.steeringRatio,
                    min: 8,
                    max: 22,
                    format: (double v) => '${v.toStringAsFixed(1)} : 1',
                    onChanged: (double v) => session.applyConfig(
                      session.config.copyWith(
                        vehicle: VehicleParameters(
                          wheelbaseMeters: vehicle.wheelbaseMeters,
                          massKg: vehicle.massKg,
                          maxSteeringAngleDegrees:
                              vehicle.maxSteeringAngleDegrees,
                          steeringRatio: v,
                          maxPowerKw: vehicle.maxPowerKw,
                        ),
                      ),
                    ),
                  ),
                  const SizedBox(height: 4),
                  Align(
                    alignment: Alignment.centerLeft,
                    child: Text(
                      'Tightest turning circle: '
                      '${vehicle.minimumTurnRadiusMeters.toStringAsFixed(1)} m',
                      style: HudTheme.caption,
                    ),
                  ),
                ],
              ),
            ),
          ),

          const SectionHeader('Navigation'),
          Card(
            child: SwitchListTile(
              title: const Text('Offline only', style: HudTheme.body),
              subtitle: const Text(
                'Never contact a routing server. Destinations then show a '
                'straight line and a bearing, clearly labelled as not a road '
                'route.',
                style: HudTheme.caption,
              ),
              value: _offlineRouting,
              onChanged: (bool v) {
                setState(() => _offlineRouting = v);
                session.navigation.provider = v
                    ? const DirectLineRouteProvider()
                    : FallbackRouteProvider(<RouteProvider>[
                        OsrmRouteProvider(),
                        const DirectLineRouteProvider(),
                      ]);
              },
            ),
          ),

          const SectionHeader('Storage'),
          Card(
            child: Column(
              children: <Widget>[
                ListTile(
                  title: const Text('Recorded drives', style: HudTheme.body),
                  subtitle: Text(
                    _recordingBytes == null
                        ? 'Calculating…'
                        : '${(_recordingBytes! / 1024 / 1024)
                            .toStringAsFixed(1)} MB used',
                    style: HudTheme.caption,
                  ),
                ),
                ListTile(
                  leading: const Icon(Icons.cleaning_services_outlined),
                  title: const Text('Keep only the newest 500 MB',
                      style: HudTheme.body),
                  subtitle: const Text(
                    'Long drives fill a phone quickly, and running out of '
                    'space mid-drive is the worst time to find out.',
                    style: HudTheme.caption,
                  ),
                  onTap: () async {
                    final int deleted =
                        await _store.pruneTo(500 * 1024 * 1024);
                    final int bytes = await _store.totalSizeBytes();
                    if (!mounted) return;
                    setState(() => _recordingBytes = bytes);
                    if (!context.mounted) return;
                    ScaffoldMessenger.of(context).showSnackBar(
                      SnackBar(
                        content: Text(deleted == 0
                            ? 'Nothing to prune'
                            : 'Deleted $deleted drive(s)'),
                      ),
                    );
                  },
                ),
              ],
            ),
          ),

          const SectionHeader('Safety'),
          Card(
            child: Padding(
              padding: const EdgeInsets.all(14),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: <Widget>[
                  const SimulationOnlyBadge(),
                  const SizedBox(height: 10),
                  Text(
                    'SIMULATION_ONLY = ${SafetyMode.simulationOnly}',
                    style: HudTheme.body.copyWith(
                        fontFamily: HudTheme.monoFamily),
                  ),
                  const SizedBox(height: 8),
                  const Text(
                    'This is a compile-time constant with no setter and no '
                    'build flavour that changes it. Steering, throttle and '
                    'brake values exist only to be drawn and logged.',
                    style: HudTheme.caption,
                  ),
                ],
              ),
            ),
          ),
        ],
      ),
    );
  }
}

class _Slider extends StatelessWidget {
  const _Slider({
    required this.label,
    required this.value,
    required this.min,
    required this.max,
    required this.format,
    required this.onChanged,
  });

  final String label;
  final double value;
  final double min;
  final double max;
  final String Function(double) format;
  final ValueChanged<double> onChanged;

  @override
  Widget build(BuildContext context) => Column(
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
            onChanged: onChanged,
          ),
        ],
      );
}
