import 'dart:math' as math;

import '../core/geometry.dart';
import '../core/logging.dart';
import 'traffic_light.dart';
import 'traffic_sign.dart';

/// Accumulates traffic signs seen over time into the rules currently in force.
///
/// A speed-limit sign is visible for about a second and then governs the next
/// kilometre. Without this accumulator the stack would forget every rule the
/// instant the sign left the frame, which is the difference between a demo and
/// something that could inform a driver.
///
/// Rules expire on **distance travelled**, not on time: a limit that ends
/// after 800 m ends after 800 m whether that took 30 seconds or 5 minutes in
/// traffic.
class RegulatoryContextTracker {
  RegulatoryContextTracker({
    this.minObservationsToAdopt = 3,
    this.minValueConfidence = 0.70,
    this.stopApproachMeters = 40,
  });

  static const String _tag = 'RegulatoryContext';

  /// How many consistent frames a sign needs before its rule is adopted.
  final int minObservationsToAdopt;

  /// How confidently a speed-limit *number* must be read before it is used.
  final double minValueConfidence;

  /// Within this distance a stop or give-way sign becomes a pending action.
  final double stopApproachMeters;

  RegulatoryContext _context = const RegulatoryContext();
  double _odometerMeters = 0;

  int? _pendingLimitKph;
  int _pendingLimitObservations = 0;
  double _pendingLimitBestConfidence = 0;

  double? _schoolZoneSetAtMeters;
  double? _roadWorksSetAtMeters;
  double? _noOvertakingSetAtMeters;
  double? _stopSeenAtMeters;
  double? _giveWaySeenAtMeters;

  RegulatoryContext get context => _context;
  double get odometerMeters => _odometerMeters;

  /// Advance the odometer. Called once per pipeline cycle with the ego speed.
  void advance(double speedMps, double dtSeconds) {
    if (dtSeconds <= 0 || dtSeconds > 2) return;
    _odometerMeters += math.max(0, speedMps) * dtSeconds;
    _expireRules();
  }

  /// Fold this frame's signs into the running context.
  void observeSigns(List<TrafficSign> signs) {
    for (final TrafficSign sign in signs) {
      if (!sign.appliesToEgoLane) continue;
      if (sign.confidence.value < 0.4) continue;

      switch (sign.type) {
        case TrafficSignType.speedLimit:
          _observeSpeedLimit(sign);
        case TrafficSignType.endOfSpeedLimit:
          if (sign.observationCount >= minObservationsToAdopt) {
            _clearSpeedLimit('end-of-limit sign');
          }
        case TrafficSignType.noOvertaking:
          if (sign.observationCount >= minObservationsToAdopt) {
            _noOvertakingSetAtMeters = _odometerMeters;
            _context = _context.copyWith(overtakingProhibited: true);
          }
        case TrafficSignType.endOfNoOvertaking:
          _noOvertakingSetAtMeters = null;
          _context = _context.copyWith(overtakingProhibited: false);
        case TrafficSignType.schoolZone:
          if (sign.observationCount >= 2) {
            _schoolZoneSetAtMeters = _odometerMeters;
            _context = _context.copyWith(inSchoolZone: true);
          }
        case TrafficSignType.roadWork:
          if (sign.observationCount >= 2) {
            _roadWorksSetAtMeters = _odometerMeters;
            _context = _context.copyWith(inRoadWorks: true);
          }
        case TrafficSignType.stop:
          final double? d = sign.distanceMeters;
          if (d != null && d < stopApproachMeters && sign.observationCount >= 2) {
            _stopSeenAtMeters = _odometerMeters + d;
            _context = _context.copyWith(pendingStop: true);
          }
        case TrafficSignType.giveWay:
          final double? d = sign.distanceMeters;
          if (d != null && d < stopApproachMeters && sign.observationCount >= 2) {
            _giveWaySeenAtMeters = _odometerMeters + d;
            _context = _context.copyWith(pendingGiveWay: true);
          }
        default:
          break;
      }
    }
    _context =
        _context.copyWith(distanceTravelledMeters: _odometerMeters);
  }

  /// A green light at a junction clears a pending stop-line obligation, since
  /// the signal supersedes the sign at a signalised junction.
  void observeLights(List<TrafficLight> lights) {
    for (final TrafficLight light in lights) {
      if (!light.isActionable) continue;
      if (light.color.permitsGo) {
        if (_context.pendingStop || _context.pendingGiveWay) {
          _stopSeenAtMeters = null;
          _giveWaySeenAtMeters = null;
          _context = _context.copyWith(
            pendingStop: false,
            pendingGiveWay: false,
          );
        }
      }
    }
  }

  void _observeSpeedLimit(TrafficSign sign) {
    final int? value = sign.speedLimitKph;
    final double valueConfidence = sign.valueConfidence?.value ?? 0;
    if (value == null) return;

    if (_pendingLimitKph != value) {
      _pendingLimitKph = value;
      _pendingLimitObservations = 1;
      _pendingLimitBestConfidence = valueConfidence;
      return;
    }

    _pendingLimitObservations++;
    _pendingLimitBestConfidence =
        math.max(_pendingLimitBestConfidence, valueConfidence);

    final bool enoughObservations =
        _pendingLimitObservations >= minObservationsToAdopt;
    final bool confidentEnough =
        _pendingLimitBestConfidence >= minValueConfidence;

    if (!enoughObservations || !confidentEnough) return;
    if (_context.speedLimitKph == value) return;

    // A very large jump (50 -> 120) from a single sign is more likely a misread
    // than a real change, so it needs stronger evidence.
    final int? previous = _context.speedLimitKph;
    if (previous != null && (value - previous).abs() > 50) {
      if (_pendingLimitObservations < minObservationsToAdopt * 2 ||
          _pendingLimitBestConfidence < 0.85) {
        return;
      }
    }

    _context = _context.copyWith(
      speedLimitKph: value,
      speedLimitConfidence: _pendingLimitBestConfidence,
      speedLimitSetAtMeters: _odometerMeters,
    );
    Log.info(_tag,
        'adopted speed limit $value km/h at ${_odometerMeters.round()} m '
        '($_pendingLimitObservations observations, '
        '${(_pendingLimitBestConfidence * 100).round()}%)');
  }

  void _clearSpeedLimit(String reason) {
    if (_context.speedLimitKph == null) return;
    Log.info(_tag, 'cleared speed limit: $reason');
    _context = RegulatoryContext(
      speedLimitKph: null,
      speedLimitConfidence: 0,
      overtakingProhibited: _context.overtakingProhibited,
      inSchoolZone: _context.inSchoolZone,
      inRoadWorks: _context.inRoadWorks,
      pendingStop: _context.pendingStop,
      pendingGiveWay: _context.pendingGiveWay,
      distanceTravelledMeters: _odometerMeters,
    );
    _pendingLimitKph = null;
    _pendingLimitObservations = 0;
    _pendingLimitBestConfidence = 0;
  }

  /// Retire rules whose posted persistence distance has elapsed.
  void _expireRules() {
    bool changed = false;
    int? speedLimit = _context.speedLimitKph;
    double speedLimitConfidence = _context.speedLimitConfidence;
    bool noOvertaking = _context.overtakingProhibited;
    bool schoolZone = _context.inSchoolZone;
    bool roadWorks = _context.inRoadWorks;
    bool pendingStop = _context.pendingStop;
    bool pendingGiveWay = _context.pendingGiveWay;

    final double? limitSetAt = _context.speedLimitSetAtMeters;
    if (speedLimit != null && limitSetAt != null) {
      final double elapsed = _odometerMeters - limitSetAt;
      if (elapsed > TrafficSignType.speedLimit.persistenceMeters) {
        speedLimit = null;
        speedLimitConfidence = 0;
        changed = true;
      } else {
        // Confidence decays with distance: the further we are from the sign,
        // the more likely we have passed an unobserved change.
        final double decayed = _context.speedLimitConfidence *
            clampDouble(
              1 - elapsed / TrafficSignType.speedLimit.persistenceMeters * 0.5,
              0.4,
              1,
            );
        if ((decayed - speedLimitConfidence).abs() > 0.01) {
          speedLimitConfidence = decayed;
          changed = true;
        }
      }
    }

    if (noOvertaking &&
        _noOvertakingSetAtMeters != null &&
        _odometerMeters - _noOvertakingSetAtMeters! >
            TrafficSignType.noOvertaking.persistenceMeters) {
      noOvertaking = false;
      _noOvertakingSetAtMeters = null;
      changed = true;
    }

    if (schoolZone &&
        _schoolZoneSetAtMeters != null &&
        _odometerMeters - _schoolZoneSetAtMeters! >
            TrafficSignType.schoolZone.persistenceMeters) {
      schoolZone = false;
      _schoolZoneSetAtMeters = null;
      changed = true;
    }

    if (roadWorks &&
        _roadWorksSetAtMeters != null &&
        _odometerMeters - _roadWorksSetAtMeters! >
            TrafficSignType.roadWork.persistenceMeters) {
      roadWorks = false;
      _roadWorksSetAtMeters = null;
      changed = true;
    }

    // A stop obligation is discharged once we have passed the stop line.
    if (pendingStop &&
        _stopSeenAtMeters != null &&
        _odometerMeters > _stopSeenAtMeters! + 4) {
      pendingStop = false;
      _stopSeenAtMeters = null;
      changed = true;
    }
    if (pendingGiveWay &&
        _giveWaySeenAtMeters != null &&
        _odometerMeters > _giveWaySeenAtMeters! + 4) {
      pendingGiveWay = false;
      _giveWaySeenAtMeters = null;
      changed = true;
    }

    if (!changed) return;

    _context = RegulatoryContext(
      speedLimitKph: speedLimit,
      speedLimitConfidence: speedLimitConfidence,
      speedLimitSetAtMeters: speedLimit == null ? null : limitSetAt,
      overtakingProhibited: noOvertaking,
      inSchoolZone: schoolZone,
      inRoadWorks: roadWorks,
      pendingStop: pendingStop,
      pendingGiveWay: pendingGiveWay,
      distanceTravelledMeters: _odometerMeters,
    );
  }

  void reset() {
    _context = const RegulatoryContext();
    _odometerMeters = 0;
    _pendingLimitKph = null;
    _pendingLimitObservations = 0;
    _pendingLimitBestConfidence = 0;
    _schoolZoneSetAtMeters = null;
    _roadWorksSetAtMeters = null;
    _noOvertakingSetAtMeters = null;
    _stopSeenAtMeters = null;
    _giveWaySeenAtMeters = null;
  }
}
