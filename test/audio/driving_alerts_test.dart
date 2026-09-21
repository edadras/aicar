import 'dart:typed_data';

import 'package:aicar/audio/alert_policy.dart';
import 'package:aicar/audio/device_alert_sink.dart';
import 'package:aicar/audio/driving_alerts.dart';
import 'package:aicar/core/confidence.dart';
import 'package:aicar/decision/driving_decision.dart';
import 'package:aicar/world_model/hazard.dart';
import 'package:aicar/world_model/world_state.dart';
import 'package:flutter_test/flutter_test.dart';

import '../support/world_builder.dart';

/// Records what would have been played, so restraint can be measured.
class _RecordingSink implements AlertSink {
  final List<String> events = <String>[];

  @override
  Future<void> tone(HazardSeverity severity) async =>
      events.add('tone:${severity.name}');

  @override
  Future<void> speak(String text) async => events.add('speak:$text');

  @override
  Future<void> stopSpeaking() async => events.add('stop');

  @override
  Future<void> dispose() async {}

  List<String> get spoken =>
      events.where((String e) => e.startsWith('speak:')).toList();
  List<String> get tones =>
      events.where((String e) => e.startsWith('tone:')).toList();
  void clear() => events.clear();
}

void main() {
  /// Let the alert's async issue path run.
  ///
  /// [DrivingAlerts.update] deliberately does not await the sink — a
  /// collision warning must not be held up by a speech engine, and the
  /// pipeline must not be held up by either. So the play call lands a
  /// microtask later, and a test has to pump for it.
  Future<void> pump() => Future<void>.delayed(Duration.zero);

  DrivingDecision decide(
    DrivingState state, {
    String reason = 'test',
    int ts = 0,
  }) =>
      DrivingDecision(
        state: state,
        reason: reason,
        confidence: 0.8,
        timestampMicros: ts,
        frameId: 1,
      );

  WorldState worldWith({
    List<Hazard> hazards = const <Hazard>[],
    double speed = 14,
    int ts = 0,
    AutonomyConfidence? autonomy,
  }) =>
      testWorld(
        timestampMicros: ts,
        ego: testEgo(speedMps: speed, ts: ts),
        hazards: hazards,
        autonomy: autonomy,
      );

  Hazard hazard(HazardType type, HazardSeverity severity) => Hazard(
        type: type,
        severity: severity,
        description: type.label,
        confidence: 0.8,
      );

  group('what gets said at all', () {
    test('an ordinary clear road is silent', () async {
      final _RecordingSink sink = _RecordingSink();
      DrivingAlerts(sink: sink)
          .update(world: worldWith(), decision: decide(DrivingState.cruise));
      await pump();
      expect(sink.events, isEmpty);
    });

    test('a simulated emergency brake is a tone, not a sentence', () async {
      // A speech engine takes a few hundred milliseconds to start. That is
      // fine for "speed bump ahead" and far too slow for this.
      final _RecordingSink sink = _RecordingSink();
      DrivingAlerts(sink: sink).update(
        world: worldWith(),
        decision: decide(DrivingState.emergencyBrakeSimulation),
      );
      await pump();
      expect(sink.tones, isNotEmpty);
      expect(sink.spoken, isEmpty);
    });

    test('a pedestrian yield gets both a tone and a word', () async {
      final _RecordingSink sink = _RecordingSink();
      DrivingAlerts(sink: sink).update(
        world: worldWith(),
        decision: decide(DrivingState.pedestrianYield),
      );
      await pump();
      expect(sink.tones, isNotEmpty);
      expect(sink.spoken, <String>['speak:Pedestrian']);
    });

    test('a caution is spoken without a tone', () async {
      final _RecordingSink sink = _RecordingSink();
      DrivingAlerts(sink: sink).update(
        world: worldWith(hazards: <Hazard>[
          hazard(HazardType.speedBumpAhead, HazardSeverity.caution),
        ]),
        decision: decide(DrivingState.slowDown),
      );
      await pump();
      expect(sink.spoken, <String>['speak:Speed bump']);
      expect(sink.tones, isEmpty);
    });

    test('the system doubting itself is worth saying', () async {
      final _RecordingSink sink = _RecordingSink();
      DrivingAlerts(sink: sink).update(
        world: worldWith(
          autonomy: AutonomyConfidence.compute(
            perception: 0.05,
            lanes: 0.1,
            depth: 0.1,
            egoMotion: 0.2,
            planning: 0.1,
          ),
        ),
        decision: decide(DrivingState.uncertain),
      );
      await pump();
      expect(sink.spoken, <String>['speak:System uncertain']);
    });

    test('a stopped vehicle is not told about the road it is looking at', () async {
      final _RecordingSink sink = _RecordingSink();
      DrivingAlerts(sink: sink).update(
        world: worldWith(
          speed: 0,
          hazards: <Hazard>[
            hazard(HazardType.redLight, HazardSeverity.warning),
          ],
        ),
        decision: decide(DrivingState.wait),
      );
      await pump();
      expect(sink.events, isEmpty);
    });
  });

  group('restraint', () {
    test('a continuous hazard is announced once, not every frame', () async {
      // The single most important behaviour here. Twenty announcements a
      // second teaches a driver to ignore the system inside a minute.
      final _RecordingSink sink = _RecordingSink();
      final DrivingAlerts alerts = DrivingAlerts(sink: sink);
      for (int i = 0; i < 100; i++) {
        alerts.update(
          world: worldWith(
            ts: i * 50000,
            hazards: <Hazard>[
              hazard(HazardType.crosswalkAhead, HazardSeverity.caution),
            ],
          ),
          decision: decide(DrivingState.slowDown, ts: i * 50000),
        );
        await pump();
      }
      expect(sink.spoken, hasLength(1));
    });

    test('it re-announces only after the situation genuinely clears', () async {
      final _RecordingSink sink = _RecordingSink();
      final DrivingAlerts alerts = DrivingAlerts(sink: sink);
      final List<Hazard> h = <Hazard>[
        hazard(HazardType.crosswalkAhead, HazardSeverity.caution),
      ];

      alerts.update(world: worldWith(ts: 0, hazards: h),
          decision: decide(DrivingState.slowDown));
      await pump();
      expect(sink.spoken, hasLength(1));

      // Gone for a while...
      for (int t = 1; t < 60; t++) {
        alerts.update(
          world: worldWith(ts: t * 1000000),
          decision: decide(DrivingState.cruise),
        );
      }
      await pump();
      // ...and back. A new crossing is new information.
      alerts.update(
        world: worldWith(ts: 61 * 1000000, hazards: h),
        decision: decide(DrivingState.slowDown),
      );
      await pump();
      expect(sink.spoken, hasLength(2));
    });

    test('a TTC flickering across a threshold does not chatter', () async {
      final _RecordingSink sink = _RecordingSink();
      final DrivingAlerts alerts = DrivingAlerts(sink: sink);
      final List<Hazard> h = <Hazard>[
        hazard(HazardType.leadVehicleTooClose, HazardSeverity.warning),
      ];
      // Present, absent, present, absent... every other frame at 20 FPS.
      for (int i = 0; i < 200; i++) {
        alerts.update(
          world: worldWith(
            ts: i * 50000,
            hazards: i.isEven ? h : const <Hazard>[],
          ),
          decision: decide(DrivingState.slowDown, ts: i * 50000),
        );
        await pump();
      }
      expect(sink.spoken, hasLength(1),
          reason: 'a 50 ms gap is not the hazard clearing');
    });

    test('cautions are rationed', () async {
      final _RecordingSink sink = _RecordingSink();
      final DrivingAlerts alerts = DrivingAlerts(sink: sink);
      // Two different cautions a few seconds apart.
      alerts.update(
        world: worldWith(ts: 0, hazards: <Hazard>[
          hazard(HazardType.speedBumpAhead, HazardSeverity.caution),
        ]),
        decision: decide(DrivingState.slowDown),
      );
      alerts.update(
        world: worldWith(ts: 5000000, hazards: <Hazard>[
          hazard(HazardType.crosswalkAhead, HazardSeverity.caution),
        ]),
        decision: decide(DrivingState.slowDown),
      );
      await pump();
      expect(sink.spoken, hasLength(1),
          reason: 'the second was inside the twenty-second ration');
    });

    test('urgency is never rationed', () async {
      final _RecordingSink sink = _RecordingSink();
      final DrivingAlerts alerts = DrivingAlerts(sink: sink);
      alerts.update(
        world: worldWith(ts: 0, hazards: <Hazard>[
          hazard(HazardType.speedBumpAhead, HazardSeverity.caution),
        ]),
        decision: decide(DrivingState.slowDown),
      );
      sink.clear();
      alerts.update(
        world: worldWith(ts: 500000),
        decision: decide(DrivingState.emergencyBrakeSimulation),
      );
      await pump();
      expect(sink.tones, isNotEmpty);
    });

    test('a critical alert repeats while it lasts, but not every frame', () async {
      final _RecordingSink sink = _RecordingSink();
      final DrivingAlerts alerts = DrivingAlerts(sink: sink);
      // Four seconds of continuous emergency at 20 FPS.
      for (int i = 0; i < 80; i++) {
        alerts.update(
          world: worldWith(ts: i * 50000),
          decision: decide(DrivingState.emergencyBrakeSimulation),
        );
        await pump();
      }
      // At 0.8 s spacing that is about five tones, not eighty.
      expect(sink.tones.length, inInclusiveRange(4, 7));
    });

    test('an urgent alert interrupts a leisurely one', () async {
      final _RecordingSink sink = _RecordingSink();
      final DrivingAlerts alerts = DrivingAlerts(sink: sink);
      alerts.update(
        world: worldWith(ts: 0, hazards: <Hazard>[
          hazard(HazardType.speedBumpAhead, HazardSeverity.caution),
        ]),
        decision: decide(DrivingState.slowDown),
      );
      sink.clear();
      alerts.update(
        world: worldWith(ts: 200000),
        decision: decide(DrivingState.emergencyBrakeSimulation),
      );
      await pump();
      expect(sink.events.first, 'stop',
          reason: 'speech in progress must be cut short, not queued behind');
    });

    test('muting silences audio but not the screen', () async {
      final _RecordingSink sink = _RecordingSink();
      final DrivingAlerts alerts = DrivingAlerts(sink: sink, enabled: false);
      alerts.update(
        world: worldWith(),
        decision: decide(DrivingState.emergencyBrakeSimulation),
      );
      await pump();
      expect(sink.events, isEmpty);
      expect(alerts.current, isNotNull,
          reason: 'the HUD still has something to show');
    });
  });

  group('the tones themselves', () {
    test('each severity is a valid WAV', () async {
      for (final HazardSeverity s in HazardSeverity.values) {
        final Uint8List wav = DeviceAlertSink.toneBytesFor(s);
        expect(String.fromCharCodes(wav.sublist(0, 4)), 'RIFF');
        expect(String.fromCharCodes(wav.sublist(8, 12)), 'WAVE');
        final ByteData d = ByteData.sublistView(wav);
        expect(d.getUint32(40, Endian.little), wav.length - 44,
            reason: 'the data chunk size must match the payload');
        expect(d.getUint16(34, Endian.little), 16, reason: '16-bit');
        expect(d.getUint32(24, Endian.little), 22050);
      }
    });

    test('urgency is carried by rate, not just by volume', () async {
      // A driver has to tell them apart without looking, which means the
      // tones must differ in structure rather than only in loudness.
      final int critical = DeviceAlertSink.toneBytesFor(
        HazardSeverity.critical,
      ).length;
      final int caution =
          DeviceAlertSink.toneBytesFor(HazardSeverity.caution).length;
      expect(critical, isNot(caution));
    });

    test('a tone starts and ends at silence', () async {
      // A square-edged tone clicks, and a click is what an ear treats as
      // noise rather than as a signal.
      final Uint8List wav =
          DeviceAlertSink.toneBytesFor(HazardSeverity.warning);
      final ByteData d = ByteData.sublistView(wav);
      expect(d.getInt16(44, Endian.little).abs(), lessThan(200));
      expect(d.getInt16(wav.length - 2, Endian.little).abs(), lessThan(200));
    });
  });

  test('a policy decision explains itself', () async {
    const AlertPolicy policy = AlertPolicy();
    final DrivingAlert? a = policy.evaluate(
      world: worldWith(hazards: <Hazard>[
        hazard(HazardType.crossingTraffic, HazardSeverity.warning),
      ]),
      decision: decide(DrivingState.slowDown),
    );
    expect(a, isNotNull);
    expect(a!.displayText, isNotEmpty);
    expect(a.spokenText, isNotEmpty);
    expect(a.spokenText.split(' ').length, lessThanOrEqualTo(4),
        reason: 'a driver hears the first three words');
  });
}
