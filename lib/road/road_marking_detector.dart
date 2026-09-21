import 'dart:math' as math;
import 'dart:typed_data';

import '../camera/camera_frame.dart';
import '../camera/image_preprocessing.dart';
import '../core/confidence.dart';
import '../core/geometry.dart';
import 'birds_eye_view.dart';
import 'lane.dart';
import 'road_marking.dart';

/// Tuning for [RoadMarkingDetector].
///
/// The defaults are metric and come from road-marking standards rather than
/// from pixels, which is the whole benefit of working in bird's-eye space: a
/// stop line is 0.3 m deep on every road in the world, and stays 0.3 m deep
/// at 30 m in this grid.
class RoadMarkingConfig {
  const RoadMarkingConfig({
    this.minForwardMeters = 3.5,
    this.maxForwardMeters = 32.0,
    this.metresPerPixelForward = 0.08,
    this.metresPerPixelLateral = 0.08,
    this.roadHalfWidthMeters = 3.6,
    this.minBandDepthMeters = 0.12,
    this.maxBandDepthMeters = 0.85,
    this.minBumpBandGapMeters = 0.20,
    this.maxBumpBandGapMeters = 1.30,
    this.minCrosswalkDepthMeters = 1.0,
    this.minCrosswalkTransitions = 6,
    this.minStripeWidthMeters = 0.20,
    this.maxStripeWidthMeters = 0.90,
    this.brightRowFraction = 0.55,
    this.minClassGap = 28.0,
  });

  final double minForwardMeters;
  final double maxForwardMeters;

  /// Forward resolution has to resolve a stop line, so it is much finer than
  /// the lane detector's grid: at the lane grid's 0.25 m a 0.3 m line is one
  /// row, and one row is indistinguishable from noise.
  final double metresPerPixelForward;
  final double metresPerPixelLateral;

  /// Half-width of the strip searched across the road.
  final double roadHalfWidthMeters;

  final double minBandDepthMeters;
  final double maxBandDepthMeters;
  final double minBumpBandGapMeters;
  final double maxBumpBandGapMeters;
  final double minCrosswalkDepthMeters;

  /// A zebra needs at least three stripes to be a zebra, and every stripe
  /// contributes two edges.
  final int minCrosswalkTransitions;

  final double minStripeWidthMeters;
  final double maxStripeWidthMeters;

  /// Fraction of the road width that must be bright for a row to count as
  /// part of a solid transverse band.
  final double brightRowFraction;

  /// Minimum luma gap between the dark and bright classes of a row before it
  /// counts as containing paint. Otsu will split any row, including one that
  /// holds nothing but sensor noise; this is what stops clean asphalt from
  /// reading as a marking.
  final double minClassGap;
}

/// Finds transverse road markings — stop lines, crosswalks and speed bumps —
/// in the bird's-eye projection of the road surface.
///
/// The discriminator between the three is the **axis of alternation**, which
/// is a property of how the markings are painted and holds across
/// jurisdictions:
///
///  * A **stop line** is one solid bar across the road: one bright run along
///    the forward axis, no alternation across it.
///  * A **speed bump** is painted with bands or chevrons *across* the road,
///    repeated along the direction of travel: alternation on the **forward**
///    axis with a short period.
///  * A **crosswalk** is painted with bars running *along* the direction of
///    travel, repeated across the road: alternation on the **lateral** axis,
///    sustained over a metre or more of forward extent.
///
/// In a perspective image those three look alike and vary with distance. In
/// bird's-eye space they are three different, distance-invariant signatures,
/// which is why this runs here rather than on the camera frame.
///
/// What it cannot do is worth stating plainly.
///
/// An **unpainted** speed hump is invisible to it. Worn paint at night is
/// unreliable. Wet asphalt reflecting a street lamp can produce a bright
/// transverse band. Everything it returns carries a confidence, and the
/// accumulator downstream requires repeated sightings before any of it
/// changes a decision.
///
/// Range differs sharply between the two patterns, and it is a property of
/// the camera rather than of this code. A crossing's stripes repeat *across*
/// the image, where resolution is good, so crossings come in at 25 m and
/// beyond. A hump's bands repeat *along* it, into the vanishing point, where
/// a whole two-metre hump subtends about two image rows at 22 m on a
/// 640x360 frame — the period is not blurred, it is gone. Measured on
/// synthetic scenes, painted humps resolve from roughly 14 m at 640x360 and
/// 18 m at 960x540, scaling with capture height. Raising the inference
/// resolution is the only thing that extends it.
class RoadMarkingDetector {
  RoadMarkingDetector({this.config = const RoadMarkingConfig()});

  final RoadMarkingConfig config;

  BirdsEyeView? _bev;

  /// Drop the cached sampling table, e.g. after recalibration.
  void invalidate() => _bev = null;

  BirdsEyeView _ensureBev(CameraFrame frame) {
    final BirdsEyeView? existing = _bev;
    if (existing != null && existing.matches(frame.calibration)) {
      return existing;
    }
    final BirdsEyeView built = BirdsEyeView.build(
      calibration: frame.calibration,
      minLateral: -config.roadHalfWidthMeters,
      maxLateral: config.roadHalfWidthMeters,
      minForward: config.minForwardMeters,
      maxForward: config.maxForwardMeters,
      metresPerPixelLateral: config.metresPerPixelLateral,
      metresPerPixelForward: config.metresPerPixelForward,
    );
    _bev = built;
    return built;
  }

  RoadMarkingResult detect(CameraFrame frame, {LaneDetectionResult? lanes}) {
    final BirdsEyeView bev = _ensureBev(frame);
    if (bev.coverage < 0.2) {
      return RoadMarkingResult.unavailable(
        frameId: frame.id,
        timestampMicros: frame.timestampMicros,
        reason: 'camera shows too little road surface '
            '(${(bev.coverage * 100).round()}% of the grid)',
      );
    }

    final Uint8List gray = frame.format == PixelFormat.gray8
        ? frame.bytes
        : ImagePreprocessing.rgbToGray(frame.bytes, frame.width, frame.height);
    final Uint8List warped = bev.warpGray(gray, frame.width, frame.height);

    final _RowProfile profile = _profileRows(warped, bev);
    if (profile.usableRows < 8) {
      return RoadMarkingResult.unavailable(
        frameId: frame.id,
        timestampMicros: frame.timestampMicros,
        reason: 'no usable bird\'s-eye rows',
      );
    }

    final List<RoadMarking> out = <RoadMarking>[
      ..._findCrosswalks(profile, bev, frame),
      ..._findBandMarkings(profile, bev, frame),
    ];

    // A crosswalk and a "stop line" almost always coexist, and the stripes of
    // a zebra can read as bands. Where they overlap in depth the crosswalk is
    // the stronger, more specific claim, so it wins.
    out.removeWhere((RoadMarking m) =>
        m.type != RoadMarkingType.crosswalk &&
        out.any((RoadMarking c) =>
            c.type == RoadMarkingType.crosswalk &&
            m.distanceMeters < c.farEdgeMeters + 0.5 &&
            m.farEdgeMeters > c.distanceMeters - 0.5));

    out.sort((RoadMarking a, RoadMarking b) =>
        a.distanceMeters.compareTo(b.distanceMeters));

    return RoadMarkingResult(
      markings: out,
      frameId: frame.id,
      timestampMicros: frame.timestampMicros,
      searchRangeMeters: profile.farthestUsableForward,
    );
  }

  // --- Row analysis -------------------------------------------------------

  /// Reduce the warped grid to one descriptor per row.
  ///
  /// The threshold is computed per row, by Otsu, rather than once for the
  /// whole grid. Two reasons, and both of them bite:
  ///
  ///  * Brightness in a bird's-eye grid falls off with distance, so one
  ///    global threshold either misses far markings or invents near ones.
  ///  * A row crossing a stop line is *mostly paint*. Any threshold defined
  ///    as "brighter than this row's average" then sits above the paint
  ///    itself and finds nothing — which is exactly how a solid bar becomes
  ///    invisible while a zebra, which is half paint, still shows up.
  ///
  /// Otsu has neither problem: it splits each row into two classes wherever
  /// the split is cleanest, whether that is 2 % of the row or 80 % of it.
  /// What it will happily do is split pure noise, so a row only counts when
  /// its two classes are genuinely far apart.
  _RowProfile _profileRows(Uint8List warped, BirdsEyeView bev) {
    final int w = bev.width;
    final int h = bev.height;

    final Float32List brightFraction = Float32List(h);
    final Int32List transitions = Int32List(h);
    final Float32List contrast = Float32List(h);
    final Float32List medianStripe = Float32List(h);
    final Int32List firstBrightCol = Int32List(h)..fillRange(0, h, -1);
    final Int32List lastBrightCol = Int32List(h)..fillRange(0, h, -1);
    final Uint8List rowUsable = Uint8List(h);

    final int minRunPixels =
        math.max(2, (0.12 / bev.metresPerPixelLateral).round());
    final Uint8List scratch = Uint8List(w);

    int usable = 0;
    double farthest = 0;

    for (int row = 0; row < h; row++) {
      int n = 0;
      for (int col = 0; col < w; col++) {
        if (!bev.isValid(col, row)) continue;
        scratch[n++] = warped[row * w + col];
      }
      if (n < w * 0.5) continue;

      rowUsable[row] = 1;
      usable++;
      farthest = math.max(farthest, bev.forwardAtRow(row.toDouble()));

      final int threshold = ImagePreprocessing.otsuThreshold(
        Uint8List.sublistView(scratch, 0, n),
      );

      // How far apart the two classes actually are. Paint against asphalt is
      // a gap of 60 or more; a threshold through sensor noise is a gap of
      // about one standard deviation, which is where the floor sits.
      double belowSum = 0;
      double aboveSum = 0;
      int belowCount = 0;
      int aboveCount = 0;
      for (int i = 0; i < n; i++) {
        if (scratch[i] > threshold) {
          aboveSum += scratch[i];
          aboveCount++;
        } else {
          belowSum += scratch[i];
          belowCount++;
        }
      }
      if (belowCount == 0 || aboveCount == 0) continue;
      final double classGap = aboveSum / aboveCount - belowSum / belowCount;
      if (classGap < config.minClassGap) {
        // A genuinely flat row. It stays usable so that a run of dark rows
        // can bound a band, but it contributes no marking evidence.
        continue;
      }
      contrast[row] = classGap;

      int bright = 0;
      int edges = 0;
      bool inRun = false;
      int runStart = 0;
      final List<int> runWidths = <int>[];

      for (int col = 0; col < w; col++) {
        final bool isBright =
            bev.isValid(col, row) && warped[row * w + col] > threshold;
        if (isBright) {
          bright++;
          if (firstBrightCol[row] < 0) firstBrightCol[row] = col;
          lastBrightCol[row] = col;
        }
        if (isBright && !inRun) {
          inRun = true;
          runStart = col;
        } else if (!isBright && inRun) {
          inRun = false;
          final int width = col - runStart;
          if (width >= minRunPixels) {
            runWidths.add(width);
            edges += 2;
          }
        }
      }
      if (inRun) {
        final int width = w - runStart;
        if (width >= minRunPixels) {
          runWidths.add(width);
          edges += 2;
        }
      }

      brightFraction[row] = bright / n;
      transitions[row] = edges;
      if (runWidths.isNotEmpty) {
        runWidths.sort();
        medianStripe[row] =
            runWidths[runWidths.length ~/ 2] * bev.metresPerPixelLateral;
      }
    }

    return _RowProfile(
      brightFraction: brightFraction,
      transitions: transitions,
      contrast: contrast,
      medianStripeMeters: medianStripe,
      firstBrightCol: firstBrightCol,
      lastBrightCol: lastBrightCol,
      usable: rowUsable,
      usableRows: usable,
      farthestUsableForward: farthest,
    );
  }

  // --- Crosswalks ---------------------------------------------------------

  /// Lateral alternation sustained over a metre of forward extent.
  List<RoadMarking> _findCrosswalks(
    _RowProfile p,
    BirdsEyeView bev,
    CameraFrame frame,
  ) {
    final List<RoadMarking> out = <RoadMarking>[];
    final int h = bev.height;

    bool isStripedRow(int row) =>
        p.usable[row] == 1 &&
        p.transitions[row] >= config.minCrosswalkTransitions &&
        p.brightFraction[row] > 0.22 &&
        p.brightFraction[row] < 0.80 &&
        p.medianStripeMeters[row] >= config.minStripeWidthMeters &&
        p.medianStripeMeters[row] <= config.maxStripeWidthMeters;

    int row = 0;
    while (row < h) {
      if (!isStripedRow(row)) {
        row++;
        continue;
      }
      final int start = row;
      // Allow a couple of non-matching rows inside the run: a shadow or a
      // worn stripe should not split one crossing into two.
      int gap = 0;
      int end = row;
      while (row < h && gap <= 3) {
        if (isStripedRow(row)) {
          end = row;
          gap = 0;
        } else {
          gap++;
        }
        row++;
      }

      final int span = end - start + 1;
      final double depth = span * bev.metresPerPixelForward;
      if (depth < config.minCrosswalkDepthMeters) continue;

      int agreeing = 0;
      double transitionSum = 0;
      double contrastSum = 0;
      int left = bev.width;
      int right = 0;
      for (int r = start; r <= end; r++) {
        if (!isStripedRow(r)) continue;
        agreeing++;
        transitionSum += p.transitions[r];
        contrastSum += p.contrast[r];
        if (p.firstBrightCol[r] >= 0) {
          left = math.min(left, p.firstBrightCol[r]);
          right = math.max(right, p.lastBrightCol[r]);
        }
      }
      if (agreeing == 0 || right <= left) continue;

      // Rows are ordered far-to-near, so the near edge is the *last* row.
      final double nearEdge = bev.forwardAtRow(end.toDouble());
      final double widthMeters = (right - left) * bev.metresPerPixelLateral;
      final double lateralCenter =
          bev.lateralAtColumn((left + right) / 2);

      // A crossing spans the road. Something a metre wide with stripes is a
      // hatched median or a bus-stop box, not a crossing.
      if (widthMeters < 1.8) continue;

      final double agreement = agreeing / span;
      final double stripeStrength = clampDouble(
        (transitionSum / agreeing) / (config.minCrosswalkTransitions * 2),
        0.3,
        1.0,
      );
      final double contrastTerm =
          clampDouble((contrastSum / agreeing) / 70.0, 0.25, 1.0);

      out.add(RoadMarking(
        type: RoadMarkingType.crosswalk,
        distanceMeters: nearEdge,
        depthMeters: depth,
        lateralCenterMeters: lateralCenter,
        widthMeters: widthMeters,
        confidence: Confidence(
          agreement * stripeStrength * contrastTerm * _rangeTrust(nearEdge),
          source: 'bev-stripe-pattern',
        ),
        frameId: frame.id,
        timestampMicros: frame.timestampMicros,
        firstSeenMicros: frame.timestampMicros,
      ));
    }

    return out;
  }

  // --- Solid transverse bands: stop lines and speed bumps -----------------

  List<RoadMarking> _findBandMarkings(
    _RowProfile p,
    BirdsEyeView bev,
    CameraFrame frame,
  ) {
    final List<_Band> bands = <_Band>[];
    final int h = bev.height;

    bool isBandRow(int row) =>
        p.usable[row] == 1 &&
        p.brightFraction[row] >= config.brightRowFraction &&
        // A solid bar has at most one bright run; more than that is a zebra.
        p.transitions[row] <= 4;

    int row = 0;
    while (row < h) {
      if (!isBandRow(row)) {
        row++;
        continue;
      }
      final int start = row;
      while (row < h && isBandRow(row)) {
        row++;
      }
      final int end = row - 1;
      final double depth = (end - start + 1) * bev.metresPerPixelForward;
      if (depth < config.minBandDepthMeters ||
          depth > config.maxBandDepthMeters) {
        continue;
      }

      int left = bev.width;
      int right = 0;
      double contrastSum = 0;
      for (int r = start; r <= end; r++) {
        if (p.firstBrightCol[r] >= 0) {
          left = math.min(left, p.firstBrightCol[r]);
          right = math.max(right, p.lastBrightCol[r]);
        }
        contrastSum += p.contrast[r];
      }
      if (right <= left) continue;

      final double widthMeters = (right - left) * bev.metresPerPixelLateral;
      if (widthMeters < 1.5) continue;

      bands.add(_Band(
        nearEdgeMeters: bev.forwardAtRow(end.toDouble()),
        farEdgeMeters: bev.forwardAtRow(start.toDouble()),
        depthMeters: depth,
        lateralCenterMeters: bev.lateralAtColumn((left + right) / 2),
        widthMeters: widthMeters,
        contrast: contrastSum / (end - start + 1),
      ));
    }

    if (bands.isEmpty) return const <RoadMarking>[];

    // Bands are far-to-near in scan order; sort near-to-far for grouping.
    bands.sort((_Band a, _Band b) =>
        a.nearEdgeMeters.compareTo(b.nearEdgeMeters));

    final List<RoadMarking> out = <RoadMarking>[];
    int i = 0;
    while (i < bands.length) {
      final List<_Band> group = <_Band>[bands[i]];
      int j = i + 1;
      while (j < bands.length) {
        final double gap =
            bands[j].nearEdgeMeters - group.last.farEdgeMeters;
        if (gap < config.minBumpBandGapMeters ||
            gap > config.maxBumpBandGapMeters) {
          break;
        }
        group.add(bands[j]);
        j++;
      }
      i = j;

      final _Band first = group.first;
      final _Band last = group.last;
      final double contrastTerm = clampDouble(
        group.map((_Band b) => b.contrast).reduce(math.max) / 70.0,
        0.25,
        1.0,
      );
      final double range = _rangeTrust(first.nearEdgeMeters);

      if (group.length >= 2) {
        // Repeated bands across the road, closely spaced along travel: the
        // standard painted hump or chevron pattern.
        final double evenness = _spacingEvenness(group);
        out.add(RoadMarking(
          type: RoadMarkingType.speedBump,
          distanceMeters: first.nearEdgeMeters,
          depthMeters: last.farEdgeMeters - first.nearEdgeMeters,
          lateralCenterMeters: first.lateralCenterMeters,
          widthMeters: group
              .map((_Band b) => b.widthMeters)
              .reduce(math.max),
          confidence: Confidence(
            clampDouble(
                (0.45 + 0.1 * group.length) * evenness * contrastTerm * range,
                0,
                0.95),
            source: 'bev-transverse-bands',
          ),
          frameId: frame.id,
          timestampMicros: frame.timestampMicros,
          firstSeenMicros: frame.timestampMicros,
        ));
      } else {
        // One solid bar. Claiming a hump from a single band would fire on
        // every tar seam and every shadow, so this is only ever a stop line —
        // and a stop line is evidence, not an obligation.
        out.add(RoadMarking(
          type: RoadMarkingType.stopLine,
          distanceMeters: first.nearEdgeMeters,
          depthMeters: first.depthMeters,
          lateralCenterMeters: first.lateralCenterMeters,
          widthMeters: first.widthMeters,
          confidence: Confidence(
            clampDouble(0.55 * contrastTerm * range, 0, 0.9),
            source: 'bev-solid-band',
          ),
          frameId: frame.id,
          timestampMicros: frame.timestampMicros,
          firstSeenMicros: frame.timestampMicros,
        ));
      }
    }

    return out;
  }

  /// How even the spacing of a band group is. Painted humps are regular;
  /// a chance alignment of a shadow and a seam is not.
  double _spacingEvenness(List<_Band> group) {
    if (group.length < 3) return 0.8;
    final List<double> gaps = <double>[
      for (int k = 1; k < group.length; k++)
        group[k].nearEdgeMeters - group[k - 1].farEdgeMeters,
    ];
    final double mean = gaps.reduce((double a, double b) => a + b) / gaps.length;
    if (mean <= 0) return 0.5;
    double spread = 0;
    for (final double g in gaps) {
      spread += (g - mean).abs();
    }
    return clampDouble(1 - (spread / gaps.length) / mean, 0.4, 1.0);
  }

  /// Trust in a measurement at this distance.
  ///
  /// One grid row covers a fixed 8 cm of ground, but the *pixels* feeding it
  /// get scarcer with distance: beyond about 25 m a marking is a handful of
  /// source pixels resampled many times over.
  double _rangeTrust(double distanceMeters) =>
      clampDouble(1 - (distanceMeters - 8) / 40, 0.3, 1.0);
}

class _RowProfile {
  const _RowProfile({
    required this.brightFraction,
    required this.transitions,
    required this.contrast,
    required this.medianStripeMeters,
    required this.firstBrightCol,
    required this.lastBrightCol,
    required this.usable,
    required this.usableRows,
    required this.farthestUsableForward,
  });

  final Float32List brightFraction;
  final Int32List transitions;
  final Float32List contrast;
  final Float32List medianStripeMeters;
  final Int32List firstBrightCol;
  final Int32List lastBrightCol;
  final Uint8List usable;
  final int usableRows;
  final double farthestUsableForward;
}

class _Band {
  const _Band({
    required this.nearEdgeMeters,
    required this.farEdgeMeters,
    required this.depthMeters,
    required this.lateralCenterMeters,
    required this.widthMeters,
    required this.contrast,
  });

  final double nearEdgeMeters;
  final double farEdgeMeters;
  final double depthMeters;
  final double lateralCenterMeters;
  final double widthMeters;
  final double contrast;
}
