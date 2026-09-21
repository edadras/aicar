import 'dart:math' as math;

import '../core/logging.dart';
import 'road_marking.dart';

/// Accumulates per-frame marking detections into the markings actually ahead.
///
/// A single frame's detection is not enough to brake for. Paint is noisy: a
/// wet patch reflecting a street lamp reads as a stop line for one frame, and
/// a real crossing at 25 m is missed for one frame in three. So sightings are
/// associated across frames by *ground position*, which is the thing that
/// stays put while we move towards it, and only a marking seen repeatedly
/// becomes confirmed.
///
/// The ego motion model is the whole trick: a marking 24 m ahead is 22 m
/// ahead a second later at 8 m/s. Predicting that before associating means a
/// real marking accumulates evidence while a reflection, which does not move
/// consistently, does not.
class RoadMarkingTracker {
  RoadMarkingTracker({
    this.minObservationsToConfirm = 3,
    this.minConfidenceToConfirm = 0.35,
    this.associationToleranceMeters = 2.5,
    this.missDecay = 0.72,
    this.dropBehindMeters = -6.0,
    this.bumpJoltThresholdMps2 = 1.6,
  });

  static const String _tag = 'RoadMarkings';

  /// Frames a marking must be seen in before anything acts on it.
  final int minObservationsToConfirm;
  final double minConfidenceToConfirm;

  /// How far a prediction may be off and still be the same marking. Generous,
  /// because the distance estimate itself is the noisy part.
  final double associationToleranceMeters;

  /// Confidence multiplier applied each frame a tracked marking is not seen.
  final double missDecay;

  /// Distance behind the vehicle at which a marking is forgotten.
  final double dropBehindMeters;

  /// Vertical acceleration that counts as having actually crossed a bump.
  final double bumpJoltThresholdMps2;

  final List<RoadMarking> _tracked = <RoadMarking>[];

  int _bumpsPredicted = 0;
  int _bumpsConfirmed = 0;
  double _peakJoltSinceCrossing = 0;
  RoadMarking? _crossingBump;

  /// Everything currently believed to be ahead, confirmed or not.
  List<RoadMarking> get all => List<RoadMarking>.unmodifiable(_tracked);

  /// Markings seen often enough and confidently enough to act on.
  List<RoadMarking> get confirmed => <RoadMarking>[
        for (final RoadMarking m in _tracked)
          if (m.observationCount >= minObservationsToConfirm &&
              m.confidence.value >= minConfidenceToConfirm)
            m,
      ];

  /// How the speed-bump detector has actually performed this session.
  ///
  /// Predicted counts every bump we committed to; confirmed counts the ones
  /// the IMU felt on the way over. The ratio is not used by any decision — it
  /// is here so the claim "there is a speed bump ahead" can be audited
  /// against what the vehicle physically did.
  (int predicted, int confirmed) get bumpScore =>
      (_bumpsPredicted, _bumpsConfirmed);

  void reset() {
    _tracked.clear();
    _bumpsPredicted = 0;
    _bumpsConfirmed = 0;
    _crossingBump = null;
    _peakJoltSinceCrossing = 0;
  }

  /// Fold one frame of detections in.
  ///
  /// [travelledMeters] is how far the vehicle moved since the previous call;
  /// [verticalAccelMps2] is the gravity-free vertical acceleration, used only
  /// to score bump predictions after the fact.
  void update({
    required RoadMarkingResult result,
    required double travelledMeters,
    double verticalAccelMps2 = 0,
  }) {
    // 1. Carry every tracked marking towards us.
    for (int i = 0; i < _tracked.length; i++) {
      _tracked[i] = _tracked[i].copyWith(
        distanceMeters: _tracked[i].distanceMeters - travelledMeters,
      );
    }

    _scoreBumpCrossings(verticalAccelMps2);

    // 2. Associate this frame's detections, nearest first.
    if (!result.isDegraded) {
      // Sized against the tracked list as it stands now; anything unmatched
      // is appended afterwards so the indices stay aligned with it.
      final List<bool> matched = List<bool>.filled(_tracked.length, false);
      final List<RoadMarking> additions = <RoadMarking>[];

      for (final RoadMarking fresh in result.markings) {
        int best = -1;
        double bestGap = associationToleranceMeters;
        for (int i = 0; i < _tracked.length; i++) {
          if (matched[i]) continue;
          if (_tracked[i].type != fresh.type) continue;
          final double gap =
              (_tracked[i].distanceMeters - fresh.distanceMeters).abs();
          if (gap < bestGap) {
            bestGap = gap;
            best = i;
          }
        }

        if (best < 0) {
          additions.add(fresh);
          continue;
        }

        matched[best] = true;
        final RoadMarking prior = _tracked[best];
        // Distance takes the fresh measurement; it is the one that came from
        // this frame's geometry rather than from dead reckoning.
        _tracked[best] = fresh.copyWith(
          observationCount: prior.observationCount + 1,
          confidence: prior.confidence.combineIndependent(fresh.confidence),
          confirmedByMotion: prior.confirmedByMotion,
        );

        final RoadMarking now = _tracked[best];
        if (now.observationCount == minObservationsToConfirm &&
            now.confidence.value >= minConfidenceToConfirm) {
          Log.info(_tag, 'confirmed ${now.type.label} at '
              '${now.distanceMeters.toStringAsFixed(1)} m '
              '(${now.confidence.percent}%)');
          if (now.type == RoadMarkingType.speedBump) _bumpsPredicted++;
        }
      }

      // 3. Decay the ones we expected to see and did not — but only within
      //    the range we could actually look. A crossing beyond the grid's
      //    horizon was not missed, it was simply out of sight.
      for (int i = 0; i < matched.length; i++) {
        if (matched[i]) continue;
        final RoadMarking m = _tracked[i];
        if (result.searchRangeMeters > 0 &&
            m.distanceMeters > result.searchRangeMeters) {
          continue;
        }
        _tracked[i] = m.copyWith(
          confidence: m.confidence.scaled(missDecay, source: 'not re-observed'),
        );
      }

      _tracked.addAll(additions);
    }

    // 4. Retire markings that are behind us or no longer believed.
    _tracked.removeWhere((RoadMarking m) =>
        m.distanceMeters < dropBehindMeters || m.confidence.value < 0.05);
  }

  /// Watch the vehicle cross a bump we predicted, and record whether the
  /// jolt we claimed would be there actually was.
  void _scoreBumpCrossings(double verticalAccelMps2) {
    final RoadMarking? crossing = _crossingBump;
    if (crossing != null) {
      _peakJoltSinceCrossing =
          math.max(_peakJoltSinceCrossing, verticalAccelMps2.abs());
      // Give it a few metres past the far edge before deciding.
      final int idx = _tracked.indexWhere((RoadMarking m) =>
          identical(m.type, crossing.type) &&
          (m.distanceMeters - crossing.distanceMeters).abs() < 6);
      final double now =
          idx >= 0 ? _tracked[idx].distanceMeters : crossing.distanceMeters;
      if (now < -3) {
        final bool felt = _peakJoltSinceCrossing >= bumpJoltThresholdMps2;
        if (felt) _bumpsConfirmed++;
        Log.info(_tag,
            'crossed a predicted speed bump: peak vertical '
            '${_peakJoltSinceCrossing.toStringAsFixed(2)} m/s^2 — '
            '${felt ? 'confirmed' : 'NOT felt'}');
        if (idx >= 0) {
          _tracked[idx] = _tracked[idx].copyWith(confirmedByMotion: felt);
        }
        _crossingBump = null;
        _peakJoltSinceCrossing = 0;
      }
      return;
    }

    for (final RoadMarking m in _tracked) {
      if (m.type != RoadMarkingType.speedBump) continue;
      if (m.observationCount < minObservationsToConfirm) continue;
      if (m.distanceMeters > 1.5 || m.distanceMeters < -1.5) continue;
      _crossingBump = m;
      _peakJoltSinceCrossing = verticalAccelMps2.abs();
      return;
    }
  }

  /// The nearest confirmed marking of a type that still lies ahead.
  RoadMarking? nearestAhead(RoadMarkingType type) {
    RoadMarking? best;
    for (final RoadMarking m in confirmed) {
      if (m.type != type) continue;
      if (m.farEdgeMeters < 0) continue;
      if (m.distanceMeters > type.approachMeters) continue;
      if (best == null || m.distanceMeters < best.distanceMeters) best = m;
    }
    return best;
  }

  /// Speed we should be doing right now, given everything ahead.
  ///
  /// Only markings close enough to matter contribute, and the figure eases in
  /// with distance rather than snapping: braking hard for a bump 40 m away is
  /// as wrong as not braking at all.
  double? advisorySpeedMps(double currentSpeedMps) {
    double? cap;
    for (final RoadMarking m in confirmed) {
      if (!m.type.compelsSlowing) continue;
      if (m.farEdgeMeters < 0) continue;
      if (m.distanceMeters > m.type.approachMeters) continue;

      final double target = m.type.advisorySpeedMps;
      // Comfortable deceleration profile: what speed may we be doing now and
      // still be at [target] by the time we reach the marking?
      const double comfortDecel = 1.6;
      final double allowed = math.sqrt(
        target * target + 2 * comfortDecel * math.max(0, m.distanceMeters),
      );
      final double here = math.min(allowed, math.max(target, currentSpeedMps));
      cap = cap == null ? here : math.min(cap, here);
    }
    return cap;
  }

  /// Confidence-weighted description for the HUD, nearest first.
  List<RoadMarking> get upcoming {
    final List<RoadMarking> out = <RoadMarking>[
      for (final RoadMarking m in confirmed)
        if (m.farEdgeMeters >= 0 && m.distanceMeters <= m.type.approachMeters)
          m,
    ]..sort((RoadMarking a, RoadMarking b) =>
        a.distanceMeters.compareTo(b.distanceMeters));
    return out;
  }
}
