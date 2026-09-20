import 'dart:collection';
import 'dart:developer' as developer;

import 'ring_buffer.dart';

enum LogLevel { trace, debug, info, warn, error }

class LogRecord {
  const LogRecord(this.level, this.tag, this.message, this.timestamp);

  final LogLevel level;
  final String tag;
  final String message;
  final DateTime timestamp;

  @override
  String toString() =>
      '${timestamp.toIso8601String()} [${level.name.toUpperCase()}] $tag: $message';
}

/// Tiny in-process logger. Keeps the last N records in memory so the developer
/// debug screen can show them on-device without a USB cable, and mirrors to
/// `dart:developer` for `flutter logs`.
class Log {
  Log._();

  static LogLevel minimumLevel = LogLevel.debug;
  static final RingBuffer<LogRecord> _history = RingBuffer<LogRecord>(400);

  static UnmodifiableListView<LogRecord> get history =>
      UnmodifiableListView<LogRecord>(_history.toList());

  static void trace(String tag, String message) =>
      _log(LogLevel.trace, tag, message);
  static void debug(String tag, String message) =>
      _log(LogLevel.debug, tag, message);
  static void info(String tag, String message) =>
      _log(LogLevel.info, tag, message);
  static void warn(String tag, String message) =>
      _log(LogLevel.warn, tag, message);
  static void error(String tag, String message,
          [Object? error, StackTrace? stack]) =>
      _log(LogLevel.error, tag,
          '$message${error == null ? '' : ' :: $error'}${stack == null ? '' : '\n$stack'}');

  static void _log(LogLevel level, String tag, String message) {
    if (level.index < minimumLevel.index) return;
    final LogRecord record = LogRecord(level, tag, message, DateTime.now());
    _history.add(record);
    developer.log(message, name: tag, level: _developerLevel(level));
  }

  static int _developerLevel(LogLevel l) => switch (l) {
        LogLevel.trace => 300,
        LogLevel.debug => 500,
        LogLevel.info => 800,
        LogLevel.warn => 900,
        LogLevel.error => 1000,
      };

  static void clear() => _history.clear();
}
