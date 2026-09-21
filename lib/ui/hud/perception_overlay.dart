import 'dart:math' as math;
import 'dart:ui' as ui;

import 'package:flutter/material.dart';

import '../../camera/camera_calibration.dart';
import '../../core/geometry.dart' as geom;
import '../../perception/traffic_light.dart';
import '../../perception/traffic_sign.dart';
import '../../planning/planned_path.dart';
import '../../road/intersection_detector.dart';
import '../../road/lane.dart';
import '../../road/road_marking.dart';
import '../../road/road_segmentation.dart';
import '../../tracking/object_track.dart';
import '../../world_model/world_state.dart';
import '../theme.dart';

/// Which overlays are drawn. Everything can be turned off independently
/// because a HUD that shows everything at once shows nothing.
class OverlayOptions {
  const OverlayOptions({
    this.boundingBoxes = true,
    this.laneLines = true,
    this.drivableArea = true,
    this.plannedPath = true,
    this.roadEdges = false,
    this.trafficSigns = true,
    this.trafficLights = true,
    this.predictedPaths = false,
    this.distanceLabels = true,
    this.horizonLine = false,
    this.roadMarkings = true,
  });

  final bool boundingBoxes;
  final bool laneLines;
  final bool drivableArea;
  final bool plannedPath;
  final bool roadEdges;
  final bool trafficSigns;
  final bool trafficLights;
  final bool predictedPaths;
  final bool distanceLabels;
  final bool horizonLine;

  /// Stop lines, crossings, speed bumps and the inferred junction line.
  final bool roadMarkings;

  OverlayOptions copyWith({
    bool? boundingBoxes,
    bool? laneLines,
    bool? drivableArea,
    bool? plannedPath,
    bool? roadEdges,
    bool? trafficSigns,
    bool? trafficLights,
    bool? predictedPaths,
    bool? distanceLabels,
    bool? horizonLine,
    bool? roadMarkings,
  }) =>
      OverlayOptions(
        boundingBoxes: boundingBoxes ?? this.boundingBoxes,
        laneLines: laneLines ?? this.laneLines,
        drivableArea: drivableArea ?? this.drivableArea,
        plannedPath: plannedPath ?? this.plannedPath,
        roadEdges: roadEdges ?? this.roadEdges,
        trafficSigns: trafficSigns ?? this.trafficSigns,
        trafficLights: trafficLights ?? this.trafficLights,
        predictedPaths: predictedPaths ?? this.predictedPaths,
        distanceLabels: distanceLabels ?? this.distanceLabels,
        horizonLine: horizonLine ?? this.horizonLine,
        roadMarkings: roadMarkings ?? this.roadMarkings,
      );
}

/// Draws the perception output over the camera preview.
///
/// Everything is drawn from the metric world model projected back through the
/// calibration, rather than from image-space coordinates: if the projection
/// is wrong, the overlay is visibly wrong, which is exactly the feedback a
/// calibration screen needs. Painting in normalised coordinates keeps it
/// correct at any preview size.
class PerceptionOverlayPainter extends CustomPainter {
  PerceptionOverlayPainter({
    required this.world,
    required this.path,
    required this.options,
    this.pulse = 0,
  });

  final WorldState world;
  final PlannedPath? path;
  final OverlayOptions options;

  /// 0..1 animation phase, used to make critical warnings breathe so they are
  /// noticed in peripheral vision.
  final double pulse;

  CameraCalibration get calibration => world.calibration;

  @override
  void paint(Canvas canvas, Size size) {
    // Order matters: fills first, then lines, then boxes, then labels, so
    // nothing important is ever obscured by decoration.
    if (options.drivableArea) _paintDrivableArea(canvas, size);
    if (options.plannedPath) _paintPlannedPath(canvas, size);
    if (options.laneLines) _paintLanes(canvas, size);
    if (options.roadEdges) _paintRoadEdges(canvas, size);
    if (options.horizonLine) _paintHorizon(canvas, size);
    if (options.roadMarkings) _paintRoadMarkings(canvas, size);
    if (options.predictedPaths) _paintPredictedPaths(canvas, size);
    if (options.boundingBoxes) _paintTracks(canvas, size);
    if (options.trafficSigns) _paintSigns(canvas, size);
    if (options.trafficLights) _paintLights(canvas, size);
  }

  // --- helpers ------------------------------------------------------------

  Offset? _project(geom.Vec2 groundPoint, Size size) {
    final geom.PixelPoint? p =
        calibration.projectGroundToImage(groundPoint);
    if (p == null) return null;
    final double x = p.u / calibration.imageWidth * size.width;
    final double y = p.v / calibration.imageHeight * size.height;
    // Allow a generous margin so a line leaving the frame still renders its
    // last visible segment instead of vanishing early.
    if (y < -size.height || y > size.height * 2) return null;
    return Offset(x, y);
  }

  Rect _boxToRect(geom.BoundingBox box, Size size) => Rect.fromLTWH(
        box.left * size.width,
        box.top * size.height,
        box.width * size.width,
        box.height * size.height,
      );

  // --- drivable area ------------------------------------------------------

  void _paintDrivableArea(Canvas canvas, Size size) {
    final DrivableArea area = world.drivableArea;
    if (area.samples.length < 2) return;

    final Path shape = Path();
    final List<Offset> left = <Offset>[];
    final List<Offset> right = <Offset>[];

    for (final DrivableSample s in area.samples) {
      final Offset? l = _project(geom.Vec2(s.leftEdge, s.distanceAhead), size);
      final Offset? r = _project(geom.Vec2(s.rightEdge, s.distanceAhead), size);
      if (l != null) left.add(l);
      if (r != null) right.add(r);
    }
    if (left.length < 2 || right.length < 2) return;

    shape.moveTo(left.first.dx, left.first.dy);
    for (final Offset o in left.skip(1)) {
      shape.lineTo(o.dx, o.dy);
    }
    for (final Offset o in right.reversed) {
      shape.lineTo(o.dx, o.dy);
    }
    shape.close();

    // Fade the fill with the corridor's own confidence, so a doubtful
    // drivable area literally looks fainter.
    final double alpha = (0.10 + 0.22 * area.confidence).clamp(0.0, 0.36);
    canvas.drawPath(
      shape,
      Paint()
        ..style = PaintingStyle.fill
        ..color = HudTheme.accent.withValues(alpha: alpha),
    );
    canvas.drawPath(
      shape,
      Paint()
        ..style = PaintingStyle.stroke
        ..strokeWidth = 1.5
        ..color = HudTheme.accent.withValues(alpha: 0.35),
    );
  }

  // --- planned path -------------------------------------------------------

  void _paintPlannedPath(Canvas canvas, Size size) {
    final PlannedPath? p = path;
    if (p == null || p.points.length < 2) return;

    final Color color =
        p.isBlocked ? HudTheme.plannedPathBlocked : HudTheme.plannedPath;

    // Corridor band.
    final List<Offset> left = <Offset>[];
    final List<Offset> right = <Offset>[];
    for (final PathPoint point in p.points) {
      final double heading = point.headingRadians;
      final geom.Vec2 normal =
          geom.Vec2(math.cos(heading), -math.sin(heading));
      final Offset? l =
          _project(point.position - normal * p.corridorHalfWidth, size);
      final Offset? r =
          _project(point.position + normal * p.corridorHalfWidth, size);
      if (l != null) left.add(l);
      if (r != null) right.add(r);
    }

    if (left.length >= 2 && right.length >= 2) {
      final Path band = Path()..moveTo(left.first.dx, left.first.dy);
      for (final Offset o in left.skip(1)) {
        band.lineTo(o.dx, o.dy);
      }
      for (final Offset o in right.reversed) {
        band.lineTo(o.dx, o.dy);
      }
      band.close();
      canvas.drawPath(
        band,
        Paint()
          ..style = PaintingStyle.fill
          ..shader = ui.Gradient.linear(
            Offset(size.width / 2, size.height),
            Offset(size.width / 2, size.height * 0.35),
            <Color>[
              color.withValues(alpha: 0.32),
              color.withValues(alpha: 0.04),
            ],
          ),
      );
    }

    // Centreline.
    final List<Offset> centre = <Offset>[];
    for (final PathPoint point in p.points) {
      final Offset? o = _project(point.position, size);
      if (o != null) centre.add(o);
    }
    if (centre.length >= 2) {
      final Path line = Path()..moveTo(centre.first.dx, centre.first.dy);
      for (final Offset o in centre.skip(1)) {
        line.lineTo(o.dx, o.dy);
      }
      canvas.drawPath(
        line,
        Paint()
          ..style = PaintingStyle.stroke
          ..strokeWidth = 3
          ..strokeCap = StrokeCap.round
          ..color = color.withValues(alpha: 0.9),
      );
    }

    _drawLabel(
      canvas,
      centre.isEmpty
          ? Offset(size.width / 2, size.height * 0.62)
          : centre[centre.length ~/ 2],
      p.isBlocked ? 'PATH BLOCKED' : p.source.label.toUpperCase(),
      color,
      anchorCentre: true,
    );
  }

  // --- lanes --------------------------------------------------------------

  void _paintLanes(Canvas canvas, Size size) {
    for (final LaneBoundary boundary in world.lanes.boundaries) {
      final bool isEgo = boundary.position.isEgoBoundary;
      final Color color =
          isEgo ? HudTheme.laneLine : HudTheme.laneLineAdjacent;

      final List<Offset> points = <Offset>[];
      for (double d = boundary.minRangeMeters;
          d <= boundary.maxRangeMeters;
          d += 1.5) {
        final Offset? o =
            _project(geom.Vec2(boundary.curve.evaluate(d), d), size);
        if (o != null) points.add(o);
      }
      if (points.length < 2) continue;

      final Paint paint = Paint()
        ..style = PaintingStyle.stroke
        // Line weight carries confidence: a faint, thin line is a lane the
        // system is not sure about.
        ..strokeWidth = (isEgo ? 3.0 : 2.0) *
            (0.5 + 0.5 * boundary.confidence.value)
        ..strokeCap = StrokeCap.round
        ..color = color.withValues(
            alpha: (0.35 + 0.6 * boundary.confidence.value).clamp(0.0, 1.0));

      if (boundary.lineType == LineType.dashed) {
        for (int i = 0; i + 1 < points.length; i += 2) {
          canvas.drawLine(points[i], points[i + 1], paint);
        }
      } else {
        final Path line = Path()..moveTo(points.first.dx, points.first.dy);
        for (final Offset o in points.skip(1)) {
          line.lineTo(o.dx, o.dy);
        }
        canvas.drawPath(line, paint);
      }

      if (isEgo && points.length > 3) {
        _drawLabel(
          canvas,
          points[points.length ~/ 3],
          '${boundary.confidence.percent}%',
          color,
        );
      }
    }
  }

  void _paintRoadEdges(Canvas canvas, Size size) {
    for (final RoadEdge edge in world.roadEdges) {
      final List<Offset> points = <Offset>[];
      for (double d = edge.minRangeMeters;
          d <= edge.maxRangeMeters;
          d += 2) {
        final Offset? o = _project(geom.Vec2(edge.curve.evaluate(d), d), size);
        if (o != null) points.add(o);
      }
      if (points.length < 2) continue;

      final Path line = Path()..moveTo(points.first.dx, points.first.dy);
      for (final Offset o in points.skip(1)) {
        line.lineTo(o.dx, o.dy);
      }
      canvas.drawPath(
        line,
        Paint()
          ..style = PaintingStyle.stroke
          ..strokeWidth = 2
          ..color = HudTheme.roadEdge.withValues(
              alpha: (0.3 + 0.5 * edge.confidence).clamp(0.0, 1.0)),
      );
    }
  }

  void _paintHorizon(Canvas canvas, Size size) {
    final double y = calibration.horizonYNormalized * size.height;
    canvas.drawLine(
      Offset(0, y),
      Offset(size.width, y),
      Paint()
        ..strokeWidth = 1
        ..color = HudTheme.textDim.withValues(alpha: 0.5),
    );
    _drawLabel(canvas, Offset(8, y), 'HORIZON', HudTheme.textDim);
  }

  // --- objects ------------------------------------------------------------

  /// Road-surface markings and the junction they imply.
  ///
  /// Drawn as the metric quadrilateral the marking actually occupies, so a
  /// crossing that lines up with the painted one on screen is direct visual
  /// proof that the bird's-eye geometry and the calibration agree. When they
  /// disagree, the band sits visibly off the paint — which is the point.
  void _paintRoadMarkings(Canvas canvas, Size size) {
    for (final RoadMarking m in world.roadMarkings) {
      if (m.farEdgeMeters < 0) continue;

      final double half = m.widthMeters / 2;
      final List<Offset?> corners = <Offset?>[
        _project(geom.Vec2(m.lateralCenterMeters - half, m.distanceMeters),
            size),
        _project(geom.Vec2(m.lateralCenterMeters + half, m.distanceMeters),
            size),
        _project(
            geom.Vec2(m.lateralCenterMeters + half, m.farEdgeMeters), size),
        _project(
            geom.Vec2(m.lateralCenterMeters - half, m.farEdgeMeters), size),
      ];
      if (corners.any((Offset? o) => o == null)) continue;

      final Color colour = switch (m.type) {
        RoadMarkingType.crosswalk => HudTheme.info,
        RoadMarkingType.speedBump => HudTheme.caution,
        RoadMarkingType.stopLine => HudTheme.textDim,
      };

      final Path quad = Path()..moveTo(corners[0]!.dx, corners[0]!.dy);
      for (int i = 1; i < corners.length; i++) {
        quad.lineTo(corners[i]!.dx, corners[i]!.dy);
      }
      quad.close();

      canvas.drawPath(
        quad,
        Paint()
          ..style = PaintingStyle.fill
          ..color = colour.withValues(alpha: 0.18 * m.confidence.value + 0.06),
      );
      canvas.drawPath(
        quad,
        Paint()
          ..style = PaintingStyle.stroke
          ..strokeWidth = 2
          ..color = colour.withValues(alpha: 0.85),
      );

      if (options.distanceLabels) {
        _drawLabel(
          canvas,
          Offset(corners[0]!.dx, corners[0]!.dy - 6),
          '${m.type.label}  '
          '${m.distanceMeters.toStringAsFixed(0)} m  '
          '${m.confidence.percent}%',
          colour,
        );
      }
    }

    // The junction line: where the stack believes the mouth of the junction
    // is, drawn right across the view because a junction is not a lane-width
    // feature.
    final IntersectionEstimate? junction = world.intersection;
    if (junction == null) return;
    final Offset? left =
        _project(geom.Vec2(-6, junction.distanceMeters), size);
    final Offset? right =
        _project(geom.Vec2(6, junction.distanceMeters), size);
    if (left == null || right == null) return;

    final Color colour = junction.hasCrossingTraffic
        ? HudTheme.warning
        : (junction.control.requiresYield
            ? HudTheme.caution
            : HudTheme.accent);
    final Paint paint = Paint()
      ..style = PaintingStyle.stroke
      ..strokeWidth = 2
      ..color = colour.withValues(alpha: 0.75);

    // Dashed, so it never reads as a lane boundary or a stop line.
    const double dash = 14;
    final double span = (right - left).distance;
    for (double d = 0; d < span; d += dash * 2) {
      final double t0 = d / span;
      final double t1 = math.min(1, (d + dash) / span);
      canvas.drawLine(
        Offset.lerp(left, right, t0)!,
        Offset.lerp(left, right, t1)!,
        paint,
      );
    }

    _drawLabel(
      canvas,
      Offset(left.dx + 8, left.dy - 20),
      '${junction.control.label} JUNCTION  '
      '${junction.distanceMeters.toStringAsFixed(0)} m  '
      '${junction.confidence.percent}%',
      colour,
    );
  }

  void _paintTracks(Canvas canvas, Size size) {
    for (final ObjectTrack track in world.tracks) {
      final Rect rect = _boxToRect(track.box, size);
      final Color color = HudTheme.forRisk(track.collisionRisk);

      final double emphasis =
          track.collisionRisk == CollisionRisk.critical
              ? 1.0 + 0.4 * math.sin(pulse * math.pi * 2)
              : 1.0;

      canvas.drawRRect(
        RRect.fromRectAndRadius(rect, const Radius.circular(4)),
        Paint()
          ..style = PaintingStyle.stroke
          ..strokeWidth = (track.collisionRisk == CollisionRisk.low ? 1.6 : 2.6)
              * emphasis
          // A predicted-only box (the detector missed this frame) is drawn
          // faded, so the driver can tell an observation from an inference.
          ..color = color.withValues(
              alpha: track.isPredictedOnly ? 0.45 : 0.95),
      );

      if (track.isPredictedOnly) {
        _drawLabel(canvas, rect.bottomLeft, 'PREDICTED', color);
      }

      if (options.distanceLabels) {
        _drawMultilineLabel(
          canvas,
          Offset(rect.left, rect.top),
          track.hudText,
          color,
        );
      }

      if (track.isMotorcycleCrossing) {
        _drawLabel(
          canvas,
          Offset(rect.left, rect.bottom + 2),
          'MOTORCYCLE CROSSING',
          HudTheme.critical,
        );
      }
    }
  }

  void _paintPredictedPaths(Canvas canvas, Size size) {
    for (final ObjectTrack track in world.tracks) {
      if (track.predictedPath.length < 2) continue;
      final List<Offset> points = <Offset>[];
      for (final geom.Vec2 p in track.predictedPath) {
        final Offset? o = _project(p, size);
        if (o != null) points.add(o);
      }
      if (points.length < 2) continue;

      final Paint paint = Paint()
        ..style = PaintingStyle.stroke
        ..strokeWidth = 1.5
        ..color = HudTheme.forRisk(track.collisionRisk)
            .withValues(alpha: 0.45);
      for (int i = 0; i + 1 < points.length; i += 2) {
        canvas.drawLine(points[i], points[i + 1], paint);
      }
    }
  }

  void _paintSigns(Canvas canvas, Size size) {
    for (final TrafficSign sign in world.trafficSigns) {
      final Rect rect = _boxToRect(sign.box, size);
      final Color color = sign.appliesToEgoLane
          ? HudTheme.info
          : HudTheme.textDim;
      canvas.drawRRect(
        RRect.fromRectAndRadius(rect, const Radius.circular(3)),
        Paint()
          ..style = PaintingStyle.stroke
          ..strokeWidth = 2
          ..color = color,
      );
      _drawMultilineLabel(
        canvas,
        Offset(rect.left, rect.top),
        '${sign.displayText}\n${sign.confidence.percent}%'
        '${sign.appliesToEgoLane ? '' : '\nOTHER ROAD'}',
        color,
      );
    }
  }

  void _paintLights(Canvas canvas, Size size) {
    for (final TrafficLight light in world.trafficLights) {
      final Rect rect = _boxToRect(light.box, size);
      final Color color = switch (light.color) {
        TrafficLightColor.red || TrafficLightColor.redYellow =>
          HudTheme.critical,
        TrafficLightColor.yellow ||
        TrafficLightColor.flashingYellow =>
          HudTheme.caution,
        TrafficLightColor.green => HudTheme.accent,
        _ => HudTheme.textDim,
      };

      canvas.drawRRect(
        RRect.fromRectAndRadius(rect.inflate(2), const Radius.circular(4)),
        Paint()
          ..style = PaintingStyle.stroke
          ..strokeWidth = light.isActionable ? 2.5 : 1.2
          ..color = color.withValues(alpha: light.isActionable ? 1 : 0.5),
      );

      _drawMultilineLabel(
        canvas,
        Offset(rect.right + 4, rect.top),
        '${light.displayText}\n'
        '${light.relevance == TrafficLightRelevance.egoPath
            ? 'EGO PATH'
            : light.relevance.label}\n'
        '${light.colorConfidence.percent}%',
        color,
      );
    }
  }

  // --- label drawing ------------------------------------------------------

  void _drawLabel(
    Canvas canvas,
    Offset anchor,
    String text,
    Color color, {
    bool anchorCentre = false,
  }) {
    final TextPainter painter = TextPainter(
      text: TextSpan(text: text, style: HudTheme.overlayLabel),
      textDirection: TextDirection.ltr,
    )..layout();

    final Offset position = anchorCentre
        ? Offset(anchor.dx - painter.width / 2, anchor.dy)
        : anchor;

    final Rect background = Rect.fromLTWH(
      position.dx - 3,
      position.dy - 2,
      painter.width + 6,
      painter.height + 4,
    );
    canvas.drawRRect(
      RRect.fromRectAndRadius(background, const Radius.circular(3)),
      Paint()..color = Colors.black.withValues(alpha: 0.62),
    );
    canvas.drawRRect(
      RRect.fromRectAndRadius(background, const Radius.circular(3)),
      Paint()
        ..style = PaintingStyle.stroke
        ..strokeWidth = 1
        ..color = color.withValues(alpha: 0.8),
    );
    painter.paint(canvas, position);
  }

  void _drawMultilineLabel(
    Canvas canvas,
    Offset anchor,
    String text,
    Color color,
  ) {
    final TextPainter painter = TextPainter(
      text: TextSpan(text: text, style: HudTheme.overlayLabel),
      textDirection: TextDirection.ltr,
    )..layout();

    // Keep the label inside the frame: a box near the top edge would
    // otherwise have its label clipped exactly when it matters most.
    double y = anchor.dy - painter.height - 4;
    if (y < 2) y = anchor.dy + 4;

    final Rect background =
        Rect.fromLTWH(anchor.dx - 3, y - 2, painter.width + 6,
            painter.height + 4);
    canvas.drawRRect(
      RRect.fromRectAndRadius(background, const Radius.circular(3)),
      Paint()..color = Colors.black.withValues(alpha: 0.68),
    );
    canvas.drawRRect(
      RRect.fromRectAndRadius(background, const Radius.circular(3)),
      Paint()
        ..style = PaintingStyle.stroke
        ..strokeWidth = 1
        ..color = color.withValues(alpha: 0.85),
    );
    painter.paint(canvas, Offset(anchor.dx, y));
  }

  @override
  bool shouldRepaint(PerceptionOverlayPainter oldDelegate) =>
      oldDelegate.world.frameId != world.frameId ||
      oldDelegate.pulse != pulse ||
      oldDelegate.options != options;
}
