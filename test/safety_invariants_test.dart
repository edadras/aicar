import 'dart:io';

import 'package:aicar/core/safety.dart';
import 'package:aicar/simulation/simulated_control.dart';
import 'package:flutter_test/flutter_test.dart';

/// Guards the project's central promise: this build cannot drive a vehicle.
///
/// These are source-level checks rather than behavioural ones, deliberately.
/// The strongest guarantee is not that the code chooses not to actuate — it is
/// that the code to actuate does not exist, and that a change introducing it
/// would fail here rather than in a car.
void main() {
  group('SafetyMode', () {
    test('simulation-only is hard-wired', () {
      expect(SafetyMode.simulationOnly, isTrue);
      expect(SafetyMode.assertSimulationOnly, returnsNormally);
    });

    test('the recording tag and banner say so unambiguously', () {
      expect(SafetyMode.recordingTag, 'SIMULATION_ONLY=true');
      expect(SafetyMode.banner.toLowerCase(),
          contains('no vehicle control'));
    });

    test('the forbidden interfaces are named', () {
      expect(SafetyMode.forbiddenInterfaces, contains('CAN bus'));
      expect(SafetyMode.forbiddenInterfaces,
          contains('ECU / OBD-II write'));
    });
  });

  group('control commands', () {
    test('declare themselves as simulation output', () {
      final SimulatedControlCommand command = SimulatedControlCommand(
        steeringAngleDegrees: -12,
        throttlePercent: 30,
        brakePercent: 0,
        timestampMicros: 0,
        frameId: 0,
      );
      expect(command.isSimulationOnly, isTrue);
      expect(command.toJson()['mode'], SafetyMode.recordingTag);
    });

    test('are clamped to their declared ranges', () {
      final SimulatedControlCommand command = SimulatedControlCommand(
        steeringAngleDegrees: 1000,
        throttlePercent: -50,
        brakePercent: 300,
        timestampMicros: 0,
        frameId: 0,
        steeringLimitDegrees: 35,
      );
      expect(command.steeringAngleDegrees, 35);
      expect(command.throttlePercent, 0);
      expect(command.brakePercent, 100);
    });

    test('an emergency stop is unambiguous', () {
      final SimulatedControlCommand command =
          SimulatedControlCommand.emergencyStop(
        timestampMicros: 0,
        frameId: 0,
        steeringAngleDegrees: 0,
      );
      expect(command.brakePercent, 100);
      expect(command.throttlePercent, 0);
      expect(command.isEmergency, isTrue);
    });
  });

  group('no vehicle interface exists in the source', () {
    late List<File> dartSources;
    late List<File> androidSources;

    setUpAll(() {
      dartSources = Directory('lib')
          .listSync(recursive: true)
          .whereType<File>()
          .where((File f) => f.path.endsWith('.dart'))
          .toList();
      androidSources = Directory('android/app/src')
          .listSync(recursive: true)
          .whereType<File>()
          .where((File f) =>
              f.path.endsWith('.kt') ||
              f.path.endsWith('.java') ||
              f.path.endsWith('.xml'))
          .toList();
    });

    /// Terms that would only appear in code that talks to a vehicle or to a
    /// dongle plugged into one.
    const List<String> forbiddenTerms = <String>[
      'canbus',
      'can_bus',
      'j1939',
      'isotp',
      'obd2',
      'obdii',
      'elm327',
      'bluetoothserial',
      'usbserial',
      'usbmanager',
      'setsteeringangle',
      'applybrake',
      'setthrottleactual',
    ];

    test('Dart sources contain no vehicle-bus symbols', () {
      final List<String> offenders = <String>[];
      for (final File file in dartSources) {
        final String text = file.readAsStringSync().toLowerCase();
        for (final String term in forbiddenTerms) {
          if (text.contains(term)) {
            offenders.add('${file.path}: $term');
          }
        }
      }
      expect(offenders, isEmpty,
          reason: 'vehicle interface symbols found:\n'
              '${offenders.join('\n')}');
    });

    test('Android sources contain no vehicle-bus symbols', () {
      final List<String> offenders = <String>[];
      for (final File file in androidSources) {
        final String text = file.readAsStringSync().toLowerCase();
        for (final String term in forbiddenTerms) {
          if (text.contains(term)) {
            offenders.add('${file.path}: $term');
          }
        }
      }
      expect(offenders, isEmpty,
          reason: 'vehicle interface symbols found:\n'
              '${offenders.join('\n')}');
    });

    test('the manifest requests no Bluetooth, USB or serial permission', () {
      final String manifest =
          File('android/app/src/main/AndroidManifest.xml')
              .readAsStringSync()
              .toLowerCase();

      // An app that cannot open a Bluetooth socket cannot reach an OBD
      // dongle, whatever its code says.
      expect(manifest.contains('permission.bluetooth'), isFalse);
      expect(manifest.contains('permission.bluetooth_connect'), isFalse);
      expect(manifest.contains('hardware.usb.host'), isFalse);
      expect(manifest.contains('usb_device_attached'), isFalse);
    });

    test('the manifest still removes the microphone and shared storage', () {
      // Audio *output* was added for driver alerts. Audio input was not, and
      // the difference has to stay visible in the permission list a user sees
      // at install — a driving tool that appears to want the microphone has
      // lost the argument before it starts.
      final String manifest =
          File('android/app/src/main/AndroidManifest.xml')
              .readAsStringSync();

      for (final String permission in <String>[
        'RECORD_AUDIO',
        'WRITE_EXTERNAL_STORAGE',
        'READ_EXTERNAL_STORAGE',
        'READ_PHONE_STATE',
      ]) {
        // Match the declaration, not the comment that explains it.
        final RegExp declaration = RegExp(
          '<uses-permission\\s+android:name='
          '"android\\.permission\\.$permission"\\s+tools:node="remove"',
        );
        expect(declaration.hasMatch(manifest), isTrue,
            reason: '$permission must be removed, not merely unused');
      }
    });

    test('no vehicle-bus package is declared as a dependency', () {
      final String pubspec =
          File('pubspec.yaml').readAsStringSync().toLowerCase();
      for (final String term in <String>[
        'obd',
        'can_bus',
        'flutter_blue',
        'usb_serial',
        'flutter_bluetooth',
      ]) {
        expect(pubspec.contains(term), isFalse,
            reason: 'pubspec.yaml declares "$term"');
      }
    });
  });
}
