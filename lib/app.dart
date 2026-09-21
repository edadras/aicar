import 'package:device_info_plus/device_info_plus.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:permission_handler/permission_handler.dart';
import 'package:provider/provider.dart';

import 'ai/inference_backend.dart';
import 'ai/model_registry.dart';
import 'core/logging.dart';
import 'ui/driving_session.dart';
import 'ui/screens/dashboard_screen.dart';
import 'ui/theme.dart';

/// Root widget. Owns the single [DrivingSession] and the permission gate.
class AiCarApp extends StatefulWidget {
  const AiCarApp({super.key});

  @override
  State<AiCarApp> createState() => _AiCarAppState();
}

class _AiCarAppState extends State<AiCarApp> {
  late final DrivingSession _session;
  Future<void>? _bootstrap;

  @override
  void initState() {
    super.initState();
    _session = DrivingSession(
      registry: ModelRegistry(),
      backend: TfLiteBackend(),
    );
    _bootstrap = _initialise();
  }

  Future<void> _initialise() async {
    await _session.initialise();
    try {
      final AndroidDeviceInfo info = await DeviceInfoPlugin().androidInfo;
      _session.setDeviceInfo(
        model: info.model,
        androidVersion: info.version.release,
      );
      Log.info('App', 'running on ${info.model}, Android '
          '${info.version.release} (SDK ${info.version.sdkInt})');
    } catch (e) {
      Log.warn('App', 'device info unavailable: $e');
    }
  }

  @override
  void dispose() {
    _session.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return ChangeNotifierProvider<DrivingSession>.value(
      value: _session,
      child: MaterialApp(
        title: 'Autonomous Driving Simulator',
        debugShowCheckedModeBanner: false,
        theme: HudTheme.theme(),
        home: FutureBuilder<void>(
          future: _bootstrap,
          builder: (BuildContext context, AsyncSnapshot<void> snapshot) {
            if (snapshot.connectionState != ConnectionState.done) {
              return const _SplashScreen();
            }
            return const _PermissionGate(child: DashboardScreen());
          },
        ),
      ),
    );
  }
}

class _SplashScreen extends StatelessWidget {
  const _SplashScreen();

  @override
  Widget build(BuildContext context) => const Scaffold(
        body: Center(child: CircularProgressIndicator()),
      );
}

/// Requests the permissions the stack needs, and explains why each is needed.
///
/// Camera is mandatory: without it there is nothing to perceive. Location is
/// not — the perception stack runs without GNSS, at reduced speed confidence
/// and with no navigation — so a refusal there is accepted rather than
/// blocking the app.
class _PermissionGate extends StatefulWidget {
  const _PermissionGate({required this.child});

  final Widget child;

  @override
  State<_PermissionGate> createState() => _PermissionGateState();
}

class _PermissionGateState extends State<_PermissionGate> {
  bool _checked = false;
  bool _cameraGranted = false;
  bool _locationGranted = false;

  @override
  void initState() {
    super.initState();
    _request();
  }

  Future<void> _request() async {
    final Map<Permission, PermissionStatus> statuses =
        await <Permission>[
      Permission.camera,
      Permission.locationWhenInUse,
    ].request();

    if (!mounted) return;
    setState(() {
      _checked = true;
      _cameraGranted =
          statuses[Permission.camera]?.isGranted ?? false;
      _locationGranted =
          statuses[Permission.locationWhenInUse]?.isGranted ?? false;
    });
  }

  @override
  Widget build(BuildContext context) {
    if (!_checked) return const _SplashScreen();

    if (!_cameraGranted) {
      return Scaffold(
        body: Center(
          child: Padding(
            padding: const EdgeInsets.all(28),
            child: Column(
              mainAxisSize: MainAxisSize.min,
              children: <Widget>[
                const Icon(Icons.photo_camera_outlined,
                    size: 44, color: HudTheme.caution),
                const SizedBox(height: 16),
                const Text('Camera access is required',
                    style: HudTheme.body),
                const SizedBox(height: 8),
                const Text(
                  'This app analyses the road through the rear camera. '
                  'Without it there is nothing to perceive, so none of the '
                  'perception, planning or simulation features can run.',
                  style: HudTheme.caption,
                  textAlign: TextAlign.center,
                ),
                const SizedBox(height: 20),
                FilledButton(
                  onPressed: _request,
                  child: const Text('Grant camera access'),
                ),
                TextButton(
                  onPressed: openAppSettings,
                  child: const Text('Open app settings'),
                ),
              ],
            ),
          ),
        ),
      );
    }

    return Stack(
      children: <Widget>[
        widget.child,
        if (!_locationGranted)
          Positioned(
            left: 14,
            right: 14,
            bottom: 14,
            child: Material(
              color: HudTheme.surfaceRaised,
              borderRadius: BorderRadius.circular(10),
              child: Padding(
                padding: const EdgeInsets.all(12),
                child: Row(
                  children: <Widget>[
                    const Icon(Icons.location_off_outlined,
                        color: HudTheme.caution, size: 18),
                    const SizedBox(width: 10),
                    const Expanded(
                      child: Text(
                        'No location access: speed confidence is reduced and '
                        'navigation is unavailable. Everything else works.',
                        style: HudTheme.caption,
                      ),
                    ),
                    TextButton(
                      onPressed: _request,
                      child: const Text('Grant'),
                    ),
                  ],
                ),
              ),
            ),
          ),
      ],
    );
  }
}

/// Configure system UI before the app starts.
Future<void> configureSystemChrome() async {
  await SystemChrome.setPreferredOrientations(<DeviceOrientation>[
    DeviceOrientation.portraitUp,
    DeviceOrientation.landscapeLeft,
    DeviceOrientation.landscapeRight,
  ]);
  SystemChrome.setSystemUIOverlayStyle(const SystemUiOverlayStyle(
    statusBarColor: Colors.transparent,
    statusBarIconBrightness: Brightness.light,
    systemNavigationBarColor: HudTheme.background,
  ));
}
