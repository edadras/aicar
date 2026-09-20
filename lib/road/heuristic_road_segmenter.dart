import 'dart:math' as math;
import 'dart:typed_data';

import '../ai/interfaces/road_segmenter.dart';
import '../ai/model_descriptor.dart';
import '../camera/camera_frame.dart';
import '../camera/image_preprocessing.dart';
import '../core/geometry.dart';
import 'road_segmentation.dart';

/// Drivable-surface segmentation without a neural network.
///
/// The method is seeded region growing on the road plane: the patch of ground
/// immediately in front of the vehicle is, by construction, road, so its
/// appearance statistics define what "road" looks like *in this frame* —
/// under this light, on this surface, with this exposure. Pixels are then
/// accepted as road when they match those statistics and sit in a
/// low-gradient area.
///
/// This is genuinely weaker than a trained segmenter: it cannot tell asphalt
/// from a similarly-coloured pavement, and it degrades in heavy shadow. It
/// reports that honestly through [RoadSegmentation.overallConfidence] instead
/// of pretending otherwise, and it exists so the drivable-area and
/// NO_LANE_MODE features work on a device with no models installed.
class HeuristicRoadSegmenter extends RoadSegmenter {
  HeuristicRoadSegmenter({
    this.gridWidth = 128,
    this.gridHeight = 72,
    this.lumaTolerance = 2.6,
    this.chromaTolerance = 26,
    this.gradientCeiling = 34,
  });

  /// Coarse grid: the planner needs metre-scale resolution, not pixels.
  final int gridWidth;
  final int gridHeight;

  /// How many standard deviations of the seed luma still count as road.
  final double lumaTolerance;

  /// Maximum per-channel chroma deviation from the seed, 0..255.
  final int chromaTolerance;

  /// Sobel magnitude above which a pixel is treated as a boundary, not
  /// surface. Road texture is low-gradient; kerbs, grass and vehicles are not.
  final int gradientCeiling;

  @override
  String get modelId => 'heuristic-region-grow';

  @override
  String get displayName => 'Heuristic road segmenter (seeded region growing)';

  @override
  ModelRole get role => ModelRole.roadSegmentation;

  @override
  ModelDescriptor? get descriptor => null;

  @override
  bool get isReady => true;

  @override
  String? get unavailableReason => null;

  @override
  List<SurfaceClass> get supportedClasses => const <SurfaceClass>[
        SurfaceClass.road,
        SurfaceClass.drivableRoad,
        SurfaceClass.obstacle,
        SurfaceClass.unknown,
      ];

  @override
  Future<void> load() async {}

  @override
  Future<void> close() async {}

  @override
  Future<RoadSegmentation> segment(CameraFrame frame) async {
    final bool hasColor = frame.format == PixelFormat.rgb888;

    // Work on a downscaled copy: full resolution buys nothing at metre scale
    // and costs 20x the time.
    final Uint8List small = ImagePreprocessing.resize(
      frame.bytes,
      frame.width,
      frame.height,
      gridWidth,
      gridHeight,
      channels: frame.bytesPerPixel,
    );
    final Uint8List gray = hasColor
        ? ImagePreprocessing.rgbToGray(small, gridWidth, gridHeight)
        : small;
    final Uint8List gradient =
        ImagePreprocessing.sobelMagnitude(gray, gridWidth, gridHeight);

    final double horizonRow =
        clampDouble(frame.calibration.horizonYNormalized, 0, 1) * gridHeight;

    final _SeedStatistics? seed =
        _sampleSeed(small, gray, hasColor, horizonRow);
    if (seed == null) {
      return RoadSegmentation.unavailable(
        frameId: frame.id,
        timestampMicros: frame.timestampMicros,
        reason: 'no usable road seed region in front of the vehicle',
      );
    }

    final Uint8List classes = Uint8List(gridWidth * gridHeight)
      ..fillRange(0, gridWidth * gridHeight, SurfaceClass.unknown.index);
    final Float32List confidence = Float32List(gridWidth * gridHeight);

    final int firstRow = math.max(0, horizonRow.floor());
    for (int y = firstRow; y < gridHeight; y++) {
      for (int x = 0; x < gridWidth; x++) {
        final int i = y * gridWidth + x;
        final int luma = gray[i];

        final double lumaZ = seed.lumaStd < 1e-3
            ? (luma - seed.lumaMean).abs()
            : (luma - seed.lumaMean).abs() / seed.lumaStd;
        final bool lumaOk = lumaZ <= lumaTolerance;

        bool chromaOk = true;
        if (hasColor) {
          final int r = small[i * 3];
          final int g = small[i * 3 + 1];
          final int b = small[i * 3 + 2];
          // Compare chroma, not absolute colour: a shadow changes brightness
          // but barely changes the red/blue balance of asphalt.
          final int rb = r - b;
          final int gb = g - b;
          chromaOk = (rb - seed.rbMean).abs() <= chromaTolerance &&
              (gb - seed.gbMean).abs() <= chromaTolerance;
        }

        final bool smooth = gradient[i] <= gradientCeiling;

        if (lumaOk && chromaOk && smooth) {
          classes[i] = SurfaceClass.drivableRoad.index;
          // Confidence falls off with how far the pixel is from the seed's
          // appearance and how close it is to the horizon.
          final double appearance =
              clampDouble(1.0 - lumaZ / lumaTolerance, 0, 1);
          final double height =
              clampDouble((y - horizonRow) / (gridHeight - horizonRow), 0, 1);
          confidence[i] = clampDouble(
            0.25 + 0.5 * appearance + 0.25 * math.sqrt(height),
            0,
            1,
          );
        } else {
          classes[i] = smooth
              ? SurfaceClass.unknown.index
              : SurfaceClass.obstacle.index;
          confidence[i] = 0.35;
        }
      }
    }

    _removeFloatingRegions(classes);

    return RoadSegmentation(
      width: gridWidth,
      height: gridHeight,
      classIndices: classes,
      classConfidence: confidence,
      frameId: frame.id,
      timestampMicros: frame.timestampMicros,
      modelName: modelId,
    );
  }

  /// Appearance statistics of the road immediately ahead.
  ///
  /// The seed is a wide, shallow band at the bottom of the frame. If that band
  /// is not internally consistent — a lane marking through it, a car filling
  /// it, a kerb — the statistics are meaningless and we return `null` rather
  /// than segment from a bad reference.
  _SeedStatistics? _sampleSeed(
    Uint8List image,
    Uint8List gray,
    bool hasColor,
    double horizonRow,
  ) {
    final int y0 = (gridHeight * 0.86).round();
    final int y1 = gridHeight;
    final int x0 = (gridWidth * 0.32).round();
    final int x1 = (gridWidth * 0.68).round();
    if (y0 >= y1 || x0 >= x1 || y0 <= horizonRow) return null;

    final List<int> lumas = <int>[];
    double rbSum = 0;
    double gbSum = 0;
    for (int y = y0; y < y1; y++) {
      for (int x = x0; x < x1; x++) {
        final int i = y * gridWidth + x;
        lumas.add(gray[i]);
        if (hasColor) {
          rbSum += image[i * 3] - image[i * 3 + 2];
          gbSum += image[i * 3 + 1] - image[i * 3 + 2];
        }
      }
    }
    if (lumas.length < 32) return null;

    // Median and a robust spread: lane markings inside the seed band would
    // wreck a plain mean/standard deviation.
    lumas.sort();
    final double median = lumas[lumas.length ~/ 2].toDouble();
    final double q1 = lumas[lumas.length ~/ 4].toDouble();
    final double q3 = lumas[(lumas.length * 3) ~/ 4].toDouble();
    final double iqr = q3 - q1;
    // 1.349 converts an IQR into an equivalent Gaussian sigma.
    final double sigma = math.max(3.0, iqr / 1.349);

    // A seed spanning nearly the full dynamic range is not one surface.
    if (iqr > 90) return null;

    return _SeedStatistics(
      lumaMean: median,
      lumaStd: sigma,
      rbMean: hasColor ? rbSum / lumas.length : 0,
      gbMean: hasColor ? gbSum / lumas.length : 0,
    );
  }

  /// Drop road regions that are not connected to the bottom of the frame.
  ///
  /// A patch of "road" floating above the horizon line, or across a barrier,
  /// is a colour coincidence — a grey building, an overcast sky. Only surface
  /// we could actually drive onto counts, so connectivity to the vehicle is
  /// required.
  void _removeFloatingRegions(Uint8List classes) {
    final Uint8List reachable = Uint8List(gridWidth * gridHeight);
    final List<int> stack = <int>[];

    final int bottom = gridHeight - 1;
    for (int x = 0; x < gridWidth; x++) {
      final int i = bottom * gridWidth + x;
      if (classes[i] == SurfaceClass.drivableRoad.index) {
        reachable[i] = 1;
        stack.add(i);
      }
    }

    while (stack.isNotEmpty) {
      final int i = stack.removeLast();
      final int x = i % gridWidth;
      final int y = i ~/ gridWidth;
      for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
          if (dx == 0 && dy == 0) continue;
          final int nx = x + dx;
          final int ny = y + dy;
          if (nx < 0 || ny < 0 || nx >= gridWidth || ny >= gridHeight) continue;
          final int ni = ny * gridWidth + nx;
          if (reachable[ni] == 1) continue;
          if (classes[ni] != SurfaceClass.drivableRoad.index) continue;
          reachable[ni] = 1;
          stack.add(ni);
        }
      }
    }

    for (int i = 0; i < classes.length; i++) {
      if (classes[i] == SurfaceClass.drivableRoad.index && reachable[i] == 0) {
        classes[i] = SurfaceClass.unknown.index;
      }
    }
  }
}

class _SeedStatistics {
  const _SeedStatistics({
    required this.lumaMean,
    required this.lumaStd,
    required this.rbMean,
    required this.gbMean,
  });

  final double lumaMean;
  final double lumaStd;
  final double rbMean;
  final double gbMean;
}
