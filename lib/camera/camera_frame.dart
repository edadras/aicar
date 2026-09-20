import 'dart:typed_data';

import 'camera_calibration.dart';

/// Pixel layout of a [CameraFrame]'s buffer.
enum PixelFormat {
  /// 3 bytes per pixel, red first.
  rgb888,

  /// 1 byte per pixel luminance.
  gray8,
}

/// A single camera frame after preprocessing, ready for inference and drawing.
///
/// Frames are immutable value objects holding plain typed data so they can be
/// handed to a background isolate with `TransferableTypedData` and never need
/// a lock. `id` is monotonically increasing per session, which is what ties a
/// recorded JPEG, a JSONL record and a profiler sample together.
class CameraFrame {
  CameraFrame({
    required this.id,
    required this.timestampMicros,
    required this.width,
    required this.height,
    required this.bytes,
    required this.format,
    required this.calibration,
    this.sensorRotationDegrees = 0,
    this.exposureNanos,
    this.isoSensitivity,
  })  : assert(width > 0 && height > 0),
        assert(bytes.length >= width * height * _bytesPerPixel(format));

  final int id;

  /// Monotonic timestamp of **exposure**, not of delivery.
  final int timestampMicros;

  final int width;
  final int height;
  final Uint8List bytes;
  final PixelFormat format;

  /// Calibration already rescaled to this frame's resolution.
  final CameraCalibration calibration;

  final int sensorRotationDegrees;
  final int? exposureNanos;
  final int? isoSensitivity;

  int get bytesPerPixel => _bytesPerPixel(format);
  int get pixelCount => width * height;
  double get aspectRatio => width / height;
  double get timestampSeconds => timestampMicros / 1e6;

  static int _bytesPerPixel(PixelFormat f) => switch (f) {
        PixelFormat.rgb888 => 3,
        PixelFormat.gray8 => 1,
      };

  /// Red/green/blue at a pixel. Returns the luminance three times for gray
  /// frames so callers do not need to branch.
  (int, int, int) pixelAt(int x, int y) {
    if (x < 0 || y < 0 || x >= width || y >= height) return (0, 0, 0);
    if (format == PixelFormat.gray8) {
      final int l = bytes[y * width + x];
      return (l, l, l);
    }
    final int i = (y * width + x) * 3;
    return (bytes[i], bytes[i + 1], bytes[i + 2]);
  }

  int luminanceAt(int x, int y) {
    if (format == PixelFormat.gray8) {
      if (x < 0 || y < 0 || x >= width || y >= height) return 0;
      return bytes[y * width + x];
    }
    final (int r, int g, int b) = pixelAt(x, y);
    // ITU-R BT.601 luma, integer arithmetic to stay cheap in the hot loop.
    return (r * 77 + g * 150 + b * 29) >> 8;
  }

  CameraFrame copyWith({
    int? id,
    int? timestampMicros,
    CameraCalibration? calibration,
  }) =>
      CameraFrame(
        id: id ?? this.id,
        timestampMicros: timestampMicros ?? this.timestampMicros,
        width: width,
        height: height,
        bytes: bytes,
        format: format,
        calibration: calibration ?? this.calibration,
        sensorRotationDegrees: sensorRotationDegrees,
        exposureNanos: exposureNanos,
        isoSensitivity: isoSensitivity,
      );

  @override
  String toString() =>
      'CameraFrame(#$id ${width}x$height ${format.name} @ '
      '${timestampSeconds.toStringAsFixed(3)}s)';
}

/// Raw multi-plane YUV_420_888 data exactly as CameraX delivers it.
///
/// Kept separate from [CameraFrame] because the conversion to RGB is the
/// single most expensive preprocessing step and we want the option of doing it
/// natively (see `android/app/src/main/cpp/image_ops.cpp`) or of skipping it
/// entirely when a stage only needs luminance.
class YuvFrame {
  const YuvFrame({
    required this.id,
    required this.timestampMicros,
    required this.width,
    required this.height,
    required this.yPlane,
    required this.uPlane,
    required this.vPlane,
    required this.yRowStride,
    required this.uvRowStride,
    required this.uvPixelStride,
    this.sensorRotationDegrees = 0,
  });

  final int id;
  final int timestampMicros;
  final int width;
  final int height;
  final Uint8List yPlane;
  final Uint8List uPlane;
  final Uint8List vPlane;
  final int yRowStride;
  final int uvRowStride;
  final int uvPixelStride;
  final int sensorRotationDegrees;

  /// The Y plane alone is a valid grayscale image. Several stages (lane edges,
  /// road-edge gradients) need nothing else, so this is the cheap path.
  Uint8List extractLuminance() {
    if (yRowStride == width) {
      return Uint8List.sublistView(yPlane, 0, width * height);
    }
    final Uint8List out = Uint8List(width * height);
    for (int row = 0; row < height; row++) {
      out.setRange(
        row * width,
        (row + 1) * width,
        yPlane,
        row * yRowStride,
      );
    }
    return out;
  }
}
