import 'dart:math' as math;
import 'dart:typed_data';

import 'package:audioplayers/audioplayers.dart';
import 'package:flutter_tts/flutter_tts.dart';

import '../core/logging.dart';
import '../world_model/hazard.dart';
import 'driving_alerts.dart';

/// Plays alerts on the device.
///
/// Tones are **synthesised**, not shipped as assets. Three reasons, and the
/// third is the one that matters: it keeps the APK smaller, it lets the tone
/// for each severity be derived from that severity rather than chosen by
/// hand, and it means an alert cannot silently fail because an asset was
/// missing from a build.
///
/// Speech comes from the device's own engine, so it works offline with the
/// voices the user already has. Everything here is best-effort: a phone with
/// no TTS voice installed, or with audio focus held by something else, still
/// shows every alert on screen. Audio is an addition to the HUD, never a
/// replacement for it.
class DeviceAlertSink implements AlertSink {
  DeviceAlertSink({AudioPlayer? player, FlutterTts? tts})
      : _player = player ?? AudioPlayer(playerId: 'aicar-alerts'),
        _tts = tts ?? FlutterTts();

  static const String _tag = 'AlertSink';
  static const int _sampleRate = 22050;

  final AudioPlayer _player;
  final FlutterTts _tts;

  final Map<HazardSeverity, Uint8List> _tones =
      <HazardSeverity, Uint8List>{};
  bool _ttsReady = false;

  Future<void> initialise() async {
    try {
      await _player.setReleaseMode(ReleaseMode.stop);
      // Ducking rather than exclusive focus: the driver's music should dip,
      // not stop, and navigation from another app must still be audible.
      await _player.setAudioContext(
        AudioContext(
          android: const AudioContextAndroid(
            isSpeakerphoneOn: false,
            stayAwake: false,
            contentType: AndroidContentType.sonification,
            usageType: AndroidUsageType.assistanceSonification,
            audioFocus: AndroidAudioFocus.gainTransientMayDuck,
          ),
          iOS: AudioContextIOS(
            category: AVAudioSessionCategory.playback,
            options: const <AVAudioSessionOptions>{
              AVAudioSessionOptions.duckOthers,
            },
          ),
        ),
      );
    } catch (e) {
      Log.warn(_tag, 'audio session setup failed: $e');
    }

    try {
      await _tts.setSpeechRate(0.55);
      await _tts.setVolume(1.0);
      await _tts.awaitSpeakCompletion(false);
      _ttsReady = true;
    } catch (e) {
      Log.warn(_tag, 'no speech engine available: $e');
      _ttsReady = false;
    }
  }

  @override
  Future<void> tone(HazardSeverity severity) async {
    try {
      final Uint8List wav =
          _tones[severity] ??= _synthesise(severity);
      await _player.play(BytesSource(wav), volume: _volumeFor(severity));
    } catch (e) {
      Log.warn(_tag, 'tone failed: $e');
    }
  }

  @override
  Future<void> speak(String text) async {
    if (!_ttsReady) return;
    try {
      await _tts.speak(text);
    } catch (e) {
      Log.warn(_tag, 'speech failed: $e');
    }
  }

  @override
  Future<void> stopSpeaking() async {
    try {
      if (_ttsReady) await _tts.stop();
      await _player.stop();
    } catch (_) {
      // Stopping something that is not playing is not an error worth logging
      // on the path of a collision warning.
    }
  }

  @override
  Future<void> dispose() async {
    await stopSpeaking();
    await _player.dispose();
  }

  double _volumeFor(HazardSeverity severity) => switch (severity) {
        HazardSeverity.critical => 1.0,
        HazardSeverity.warning => 0.85,
        HazardSeverity.caution => 0.6,
        HazardSeverity.info => 0.4,
      };

  /// Build a WAV for one severity.
  ///
  /// Urgency is carried by rate and pitch rather than by volume alone, which
  /// is how every alarm worth its salt works: a critical alert is three fast
  /// high pips, a caution a single low one. A driver can tell them apart
  /// without looking, which is the entire point of using sound.
  static Uint8List _synthesise(HazardSeverity severity) {
    final (int pips, double frequency, double pipSeconds, double gapSeconds) =
        switch (severity) {
      HazardSeverity.critical => (3, 1180.0, 0.075, 0.045),
      HazardSeverity.warning => (2, 880.0, 0.090, 0.060),
      HazardSeverity.caution => (1, 620.0, 0.130, 0.0),
      HazardSeverity.info => (1, 520.0, 0.090, 0.0),
    };

    final int pipSamples = (pipSeconds * _sampleRate).round();
    final int gapSamples = (gapSeconds * _sampleRate).round();
    final int total = pips * pipSamples + (pips - 1) * gapSamples;

    final Int16List samples = Int16List(total);
    int cursor = 0;
    for (int p = 0; p < pips; p++) {
      for (int i = 0; i < pipSamples; i++) {
        // Raised-cosine envelope. A square-edged tone clicks, and a click is
        // exactly the artefact a driver's ear treats as noise rather than as
        // a signal.
        final double envelope =
            0.5 - 0.5 * math.cos(2 * math.pi * i / (pipSamples - 1));
        final double value = math.sin(
              2 * math.pi * frequency * i / _sampleRate,
            ) *
            envelope;
        samples[cursor + i] = (value * 26000).round();
      }
      cursor += pipSamples;
      if (p < pips - 1) cursor += gapSamples;
    }

    return _wavOf(samples);
  }

  /// Wrap PCM in a 44-byte canonical WAV header.
  static Uint8List _wavOf(Int16List samples) {
    const int headerBytes = 44;
    final int dataBytes = samples.length * 2;
    final ByteData out = ByteData(headerBytes + dataBytes);

    void ascii(int offset, String s) {
      for (int i = 0; i < s.length; i++) {
        out.setUint8(offset + i, s.codeUnitAt(i));
      }
    }

    ascii(0, 'RIFF');
    out.setUint32(4, 36 + dataBytes, Endian.little);
    ascii(8, 'WAVE');
    ascii(12, 'fmt ');
    out.setUint32(16, 16, Endian.little); // PCM chunk size
    out.setUint16(20, 1, Endian.little); // PCM
    out.setUint16(22, 1, Endian.little); // mono
    out.setUint32(24, _sampleRate, Endian.little);
    out.setUint32(28, _sampleRate * 2, Endian.little); // byte rate
    out.setUint16(32, 2, Endian.little); // block align
    out.setUint16(34, 16, Endian.little); // bits per sample
    ascii(36, 'data');
    out.setUint32(40, dataBytes, Endian.little);

    for (int i = 0; i < samples.length; i++) {
      out.setInt16(headerBytes + i * 2, samples[i], Endian.little);
    }
    return out.buffer.asUint8List();
  }

  /// Exposed so a test can check the generated audio is a valid WAV rather
  /// than trusting that it plays.
  static Uint8List toneBytesFor(HazardSeverity severity) =>
      _synthesise(severity);
}
