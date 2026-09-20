import 'dart:typed_data';

import '../camera/camera_calibration.dart';
import '../core/geometry.dart';

/// Inverse-perspective mapping onto the road plane.
///
/// A perspective image makes lane markings converge, vary in width with
/// distance, and curve even when the road is straight. Resampling onto a
/// metric top-down grid removes all three problems at once: in bird's-eye
/// space a lane marking is a near-vertical stripe of *constant* width, which
/// turns lane finding from a fiddly heuristic into a simple matched filter
/// plus a sliding window.
///
/// The mapping depends only on the calibration, so the sampling table is built
/// once and reused for every frame — which is what makes this affordable at
/// 20 FPS on a phone.
class BirdsEyeView {
  BirdsEyeView._({
    required this.calibration,
    required this.width,
    required this.height,
    required this.minLateral,
    required this.maxLateral,
    required this.minForward,
    required this.maxForward,
    required Int32List sourceIndex,
    required Uint8List validMask,
    required this.validRowCount,
  })  : _sourceIndex = sourceIndex,
        _validMask = validMask;

  /// Build (and cache) the sampling table for [calibration].
  ///
  /// [metresPerPixelLateral] of 0.05 m resolves a 10 cm lane marking across
  /// two cells, which is the minimum a matched filter can work with.
  factory BirdsEyeView.build({
    required CameraCalibration calibration,
    double minLateral = -7.0,
    double maxLateral = 7.0,
    double minForward = 4.0,
    double maxForward = 45.0,
    double metresPerPixelLateral = 0.05,
    double metresPerPixelForward = 0.25,
  }) {
    final int width =
        ((maxLateral - minLateral) / metresPerPixelLateral).round();
    final int height =
        ((maxForward - minForward) / metresPerPixelForward).round();

    final Int32List index = Int32List(width * height);
    final Uint8List valid = Uint8List(width * height);
    int validRows = 0;

    for (int row = 0; row < height; row++) {
      // Row 0 is the *far* end of the grid, so the bird's-eye image has the
      // same "far at the top" orientation as the camera image. That keeps
      // every overlay and every debug view mentally consistent.
      final double forward =
          maxForward - (row + 0.5) * metresPerPixelForward;
      bool rowHasAnyValid = false;

      for (int col = 0; col < width; col++) {
        final double lateral =
            minLateral + (col + 0.5) * metresPerPixelLateral;
        final PixelPoint? p =
            calibration.projectGroundToImage(Vec2(lateral, forward));
        final int i = row * width + col;
        if (p == null ||
            p.u < 0 ||
            p.v < 0 ||
            p.u >= calibration.imageWidth ||
            p.v >= calibration.imageHeight) {
          index[i] = -1;
          valid[i] = 0;
          continue;
        }
        index[i] = p.v.round() * calibration.imageWidth + p.u.round();
        valid[i] = 1;
        rowHasAnyValid = true;
      }
      if (rowHasAnyValid) validRows++;
    }

    return BirdsEyeView._(
      calibration: calibration,
      width: width,
      height: height,
      minLateral: minLateral,
      maxLateral: maxLateral,
      minForward: minForward,
      maxForward: maxForward,
      sourceIndex: index,
      validMask: valid,
      validRowCount: validRows,
    );
  }

  final CameraCalibration calibration;
  final int width;
  final int height;
  final double minLateral;
  final double maxLateral;
  final double minForward;
  final double maxForward;

  /// Flat source-pixel index per grid cell, `-1` where the cell falls outside
  /// the image (behind the camera, above the horizon, or off the edge).
  final Int32List _sourceIndex;
  final Uint8List _validMask;

  /// Number of grid rows that have at least one visible cell. A low count
  /// means the camera is pitched such that little road is visible, and lane
  /// confidence is reduced accordingly.
  final int validRowCount;

  double get metresPerPixelLateral => (maxLateral - minLateral) / width;
  double get metresPerPixelForward => (maxForward - minForward) / height;

  /// Lateral position in metres of grid column [col].
  double lateralAtColumn(double col) =>
      minLateral + (col + 0.5) * metresPerPixelLateral;

  /// Grid column for a lateral position, as a double (may be out of range).
  double columnAtLateral(double lateral) =>
      (lateral - minLateral) / metresPerPixelLateral - 0.5;

  /// Distance ahead in metres of grid row [row].
  double forwardAtRow(double row) =>
      maxForward - (row + 0.5) * metresPerPixelForward;

  double rowAtForward(double forward) =>
      (maxForward - forward) / metresPerPixelForward - 0.5;

  bool isValid(int col, int row) =>
      col >= 0 &&
      row >= 0 &&
      col < width &&
      row < height &&
      _validMask[row * width + col] == 1;

  /// Resample a single-channel image into the bird's-eye grid.
  ///
  /// Nearest-neighbour is deliberate here: the table is precomputed as integer
  /// indices, and for a thresholded lane-marking response bilinear sampling
  /// blurs thin markings more than it helps.
  Uint8List warpGray(Uint8List gray, int srcWidth, int srcHeight) {
    final Uint8List out = Uint8List(width * height);
    final int maxIndex = srcWidth * srcHeight;
    for (int i = 0; i < out.length; i++) {
      final int si = _sourceIndex[i];
      if (si < 0 || si >= maxIndex) continue;
      out[i] = gray[si];
    }
    return out;
  }

  /// Resample an interleaved RGB image, returning an RGB grid. Used by the
  /// line-colour classifier, which needs chroma the grayscale path discards.
  Uint8List warpRgb(Uint8List rgb, int srcWidth, int srcHeight) {
    final Uint8List out = Uint8List(width * height * 3);
    final int maxIndex = srcWidth * srcHeight;
    for (int i = 0; i < width * height; i++) {
      final int si = _sourceIndex[i];
      if (si < 0 || si >= maxIndex) continue;
      final int s = si * 3;
      final int d = i * 3;
      out[d] = rgb[s];
      out[d + 1] = rgb[s + 1];
      out[d + 2] = rgb[s + 2];
    }
    return out;
  }

  /// Fraction of grid cells that map to a real pixel. Below ~0.35 the
  /// calibration and the framing simply do not show enough road for a
  /// trustworthy lane fit.
  double get coverage {
    int valid = 0;
    for (final int v in _validMask) {
      valid += v;
    }
    return valid / (width * height);
  }

  /// Cheap identity check so callers can rebuild the table when the
  /// calibration or the processing resolution changes.
  bool matches(CameraCalibration other) =>
      other.imageWidth == calibration.imageWidth &&
      other.imageHeight == calibration.imageHeight &&
      other.cameraHeightMeters == calibration.cameraHeightMeters &&
      other.pitchDegrees == calibration.pitchDegrees &&
      other.rollDegrees == calibration.rollDegrees &&
      other.yawDegrees == calibration.yawDegrees &&
      other.horizontalFovDegrees == calibration.horizontalFovDegrees &&
      other.lateralOffsetMeters == calibration.lateralOffsetMeters;
}
