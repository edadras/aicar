import 'dart:math' as math;
import 'dart:typed_data';

import 'camera_frame.dart';

/// CPU image operations shared by every perception stage.
///
/// These are deliberately allocation-light and loop-flat: they run on every
/// frame inside the inference isolate. Where the native library
/// (`libaicar_image_ops.so`) is available the same operations are dispatched
/// through `NativeImageOps`, which uses the same signatures, and these
/// implementations act as the portable fallback that also makes the whole
/// pipeline unit-testable on a desktop VM.
class ImagePreprocessing {
  const ImagePreprocessing._();

  /// YUV_420_888 -> RGB888 using the BT.601 full-range integer approximation.
  ///
  /// Written as a single pass with the chroma index computed once per 2x2
  /// block; the naive per-pixel version is roughly 2x slower and that matters
  /// at 1280x720x30.
  static Uint8List yuv420ToRgb(YuvFrame frame) {
    final int w = frame.width;
    final int h = frame.height;
    final Uint8List out = Uint8List(w * h * 3);
    final Uint8List y = frame.yPlane;
    final Uint8List u = frame.uPlane;
    final Uint8List v = frame.vPlane;
    final int yStride = frame.yRowStride;
    final int uvStride = frame.uvRowStride;
    final int uvPix = frame.uvPixelStride;

    int o = 0;
    for (int row = 0; row < h; row++) {
      final int yBase = row * yStride;
      final int uvBase = (row >> 1) * uvStride;
      for (int col = 0; col < w; col++) {
        final int yv = y[yBase + col];
        final int uvIndex = uvBase + (col >> 1) * uvPix;
        final int uv = (uvIndex < u.length ? u[uvIndex] : 128) - 128;
        final int vv = (uvIndex < v.length ? v[uvIndex] : 128) - 128;

        // 16.16-ish fixed point: R = Y + 1.402V, G = Y - 0.344U - 0.714V,
        // B = Y + 1.772U.
        final int r = yv + ((91881 * vv) >> 16);
        final int g = yv - ((22554 * uv + 46802 * vv) >> 16);
        final int b = yv + ((116130 * uv) >> 16);

        out[o++] = r < 0 ? 0 : (r > 255 ? 255 : r);
        out[o++] = g < 0 ? 0 : (g > 255 ? 255 : g);
        out[o++] = b < 0 ? 0 : (b > 255 ? 255 : b);
      }
    }
    return out;
  }

  /// RGB888 -> 8-bit luminance (BT.601).
  static Uint8List rgbToGray(Uint8List rgb, int width, int height) {
    final Uint8List out = Uint8List(width * height);
    int i = 0;
    for (int p = 0; p < out.length; p++) {
      final int r = rgb[i++];
      final int g = rgb[i++];
      final int b = rgb[i++];
      out[p] = (r * 77 + g * 150 + b * 29) >> 8;
    }
    return out;
  }

  /// Bilinear resize of an interleaved 8-bit image with [channels] channels.
  ///
  /// Bilinear (rather than nearest) matters: detector accuracy on small,
  /// distant objects drops measurably with nearest-neighbour downscaling.
  /// Convert an interleaved buffer between channel counts.
  ///
  /// A model's input tensor has a fixed channel count, and the camera can hand
  /// us a grayscale frame. Feeding a 1-channel buffer to a 3-channel tensor
  /// does not fail loudly: the runtime copies what it gets and leaves the rest
  /// of the tensor holding whatever was there before, so the detector sees a
  /// third of an image stitched onto the previous frame's remains.
  static Uint8List toChannels(Uint8List src, int from, int to) {
    if (from == to) return src;
    final int pixels = src.length ~/ from;
    final Uint8List out = Uint8List(pixels * to);
    if (from == 1) {
      for (int i = 0; i < pixels; i++) {
        final int v = src[i];
        for (int c = 0; c < to; c++) {
          out[i * to + c] = v;
        }
      }
      return out;
    }
    if (to == 1) {
      for (int i = 0; i < pixels; i++) {
        final int j = i * from;
        out[i] = (src[j] * 0.299 + src[j + 1] * 0.587 + src[j + 2] * 0.114)
            .round()
            .clamp(0, 255);
      }
      return out;
    }
    // Widening or narrowing between multi-channel layouts: copy what exists,
    // repeat the last channel for the rest.
    for (int i = 0; i < pixels; i++) {
      for (int c = 0; c < to; c++) {
        out[i * to + c] = src[i * from + math.min(c, from - 1)];
      }
    }
    return out;
  }

  static Uint8List resize(
    Uint8List src,
    int srcWidth,
    int srcHeight,
    int dstWidth,
    int dstHeight, {
    int channels = 3,
  }) {
    if (srcWidth == dstWidth && srcHeight == dstHeight) return src;
    final Uint8List out = Uint8List(dstWidth * dstHeight * channels);
    final double xRatio = srcWidth / dstWidth;
    final double yRatio = srcHeight / dstHeight;

    for (int dy = 0; dy < dstHeight; dy++) {
      final double sy = (dy + 0.5) * yRatio - 0.5;
      final int y0 = sy.floor().clamp(0, srcHeight - 1);
      final int y1 = math.min(y0 + 1, srcHeight - 1);
      final double fy = (sy - y0).clamp(0.0, 1.0);

      for (int dx = 0; dx < dstWidth; dx++) {
        final double sx = (dx + 0.5) * xRatio - 0.5;
        final int x0 = sx.floor().clamp(0, srcWidth - 1);
        final int x1 = math.min(x0 + 1, srcWidth - 1);
        final double fx = (sx - x0).clamp(0.0, 1.0);

        final int i00 = (y0 * srcWidth + x0) * channels;
        final int i01 = (y0 * srcWidth + x1) * channels;
        final int i10 = (y1 * srcWidth + x0) * channels;
        final int i11 = (y1 * srcWidth + x1) * channels;
        final int o = (dy * dstWidth + dx) * channels;

        for (int c = 0; c < channels; c++) {
          final double top = src[i00 + c] + (src[i01 + c] - src[i00 + c]) * fx;
          final double bot = src[i10 + c] + (src[i11 + c] - src[i10 + c]) * fx;
          out[o + c] = (top + (bot - top) * fy).round().clamp(0, 255);
        }
      }
    }
    return out;
  }

  /// Crop then resize, used for letterbox-free region-of-interest inference
  /// (e.g. running the traffic-sign classifier on a detected box).
  static Uint8List cropResize(
    Uint8List src,
    int srcWidth,
    int srcHeight,
    int cropX,
    int cropY,
    int cropW,
    int cropH,
    int dstWidth,
    int dstHeight, {
    int channels = 3,
  }) {
    final int x0 = cropX.clamp(0, srcWidth - 1);
    final int y0 = cropY.clamp(0, srcHeight - 1);
    final int w = cropW.clamp(1, srcWidth - x0);
    final int h = cropH.clamp(1, srcHeight - y0);

    final Uint8List patch = Uint8List(w * h * channels);
    for (int row = 0; row < h; row++) {
      final int srcOffset = ((y0 + row) * srcWidth + x0) * channels;
      patch.setRange(
        row * w * channels,
        (row + 1) * w * channels,
        src,
        srcOffset,
      );
    }
    return resize(patch, w, h, dstWidth, dstHeight, channels: channels);
  }

  /// Letterbox into a square canvas, preserving aspect ratio. Returns the
  /// padded buffer plus the transform needed to map detections back.
  static LetterboxResult letterbox(
    Uint8List src,
    int srcWidth,
    int srcHeight,
    int side, {
    int channels = 3,
    int padValue = 114,
  }) {
    final double scale =
        math.min(side / srcWidth, side / srcHeight);
    final int newW = math.max(1, (srcWidth * scale).round());
    final int newH = math.max(1, (srcHeight * scale).round());
    final int padX = (side - newW) ~/ 2;
    final int padY = (side - newH) ~/ 2;

    final Uint8List resized =
        resize(src, srcWidth, srcHeight, newW, newH, channels: channels);
    final Uint8List out = Uint8List(side * side * channels)
      ..fillRange(0, side * side * channels, padValue);

    for (int row = 0; row < newH; row++) {
      out.setRange(
        ((padY + row) * side + padX) * channels,
        ((padY + row) * side + padX + newW) * channels,
        resized,
        row * newW * channels,
      );
    }
    return LetterboxResult(
      bytes: out,
      side: side,
      scale: scale,
      padX: padX,
      padY: padY,
    );
  }

  /// Convert to the float tensor a network expects.
  ///
  /// [mean] and [std] are per-channel; the defaults give the common `0..1`
  /// normalisation. Set [channelsFirst] for NCHW models.
  static Float32List toFloatTensor(
    Uint8List src,
    int width,
    int height, {
    int channels = 3,
    List<double> mean = const <double>[0, 0, 0],
    List<double> std = const <double>[255, 255, 255],
    bool channelsFirst = false,
  }) {
    final Float32List out = Float32List(width * height * channels);
    final int plane = width * height;
    if (channelsFirst) {
      for (int p = 0; p < plane; p++) {
        final int base = p * channels;
        for (int c = 0; c < channels; c++) {
          out[c * plane + p] = (src[base + c] - mean[c]) / std[c];
        }
      }
    } else {
      for (int p = 0; p < plane; p++) {
        final int base = p * channels;
        for (int c = 0; c < channels; c++) {
          out[base + c] = (src[base + c] - mean[c]) / std[c];
        }
      }
    }
    return out;
  }

  /// 3x3 Sobel magnitude on a grayscale image, clamped to 0..255.
  ///
  /// This is the workhorse of the classical lane and road-edge detectors: it
  /// runs without any model file, which is what keeps the stack useful (and
  /// honest about its confidence) before neural models are installed.
  static Uint8List sobelMagnitude(Uint8List gray, int width, int height) {
    final Uint8List out = Uint8List(width * height);
    for (int y = 1; y < height - 1; y++) {
      final int rowUp = (y - 1) * width;
      final int row = y * width;
      final int rowDn = (y + 1) * width;
      for (int x = 1; x < width - 1; x++) {
        final int tl = gray[rowUp + x - 1];
        final int tc = gray[rowUp + x];
        final int tr = gray[rowUp + x + 1];
        final int ml = gray[row + x - 1];
        final int mr = gray[row + x + 1];
        final int bl = gray[rowDn + x - 1];
        final int bc = gray[rowDn + x];
        final int br = gray[rowDn + x + 1];

        final int gx = (tr + 2 * mr + br) - (tl + 2 * ml + bl);
        final int gy = (bl + 2 * bc + br) - (tl + 2 * tc + tr);
        final int mag = (gx.abs() + gy.abs()) >> 1;
        out[row + x] = mag > 255 ? 255 : mag;
      }
    }
    return out;
  }

  /// Horizontal-gradient-only Sobel. Lane markings are near-vertical
  /// structures, so the `x` response alone is a much cleaner lane cue than the
  /// full magnitude, which lights up every shadow edge across the road.
  static Int16List sobelX(Uint8List gray, int width, int height) {
    final Int16List out = Int16List(width * height);
    for (int y = 1; y < height - 1; y++) {
      final int rowUp = (y - 1) * width;
      final int row = y * width;
      final int rowDn = (y + 1) * width;
      for (int x = 1; x < width - 1; x++) {
        final int gx = (gray[rowUp + x + 1] +
                2 * gray[row + x + 1] +
                gray[rowDn + x + 1]) -
            (gray[rowUp + x - 1] + 2 * gray[row + x - 1] + gray[rowDn + x - 1]);
        out[row + x] = gx.clamp(-32768, 32767);
      }
    }
    return out;
  }

  /// Separable 3x3 box blur. Two passes, O(n) per pixel, used to suppress
  /// asphalt texture before thresholding.
  static Uint8List boxBlur3(Uint8List gray, int width, int height) {
    final Uint8List tmp = Uint8List(width * height);
    for (int y = 0; y < height; y++) {
      final int row = y * width;
      for (int x = 0; x < width; x++) {
        final int a = gray[row + (x > 0 ? x - 1 : 0)];
        final int b = gray[row + x];
        final int c = gray[row + (x < width - 1 ? x + 1 : width - 1)];
        tmp[row + x] = (a + b + c) ~/ 3;
      }
    }
    final Uint8List out = Uint8List(width * height);
    for (int y = 0; y < height; y++) {
      final int up = (y > 0 ? y - 1 : 0) * width;
      final int row = y * width;
      final int dn = (y < height - 1 ? y + 1 : height - 1) * width;
      for (int x = 0; x < width; x++) {
        out[row + x] = (tmp[up + x] + tmp[row + x] + tmp[dn + x]) ~/ 3;
      }
    }
    return out;
  }

  /// Histogram-based threshold (Otsu). Adapts to night driving, tunnels and
  /// low sun without a hand-tuned constant.
  static int otsuThreshold(Uint8List gray, {int stride = 1}) {
    final Int32List hist = Int32List(256);
    int total = 0;
    for (int i = 0; i < gray.length; i += stride) {
      hist[gray[i]]++;
      total++;
    }
    if (total == 0) return 128;

    double sum = 0;
    for (int t = 0; t < 256; t++) {
      sum += t * hist[t];
    }
    double sumB = 0;
    int wB = 0;
    double maxVar = -1;
    // With a strongly bimodal histogram every threshold between the two peaks
    // is equally optimal, and taking the first one puts the threshold exactly
    // on the darker peak — which then classifies none of those pixels as ink.
    // Track the whole plateau and take its midpoint.
    int plateauStart = 128;
    int plateauEnd = 128;
    for (int t = 0; t < 256; t++) {
      wB += hist[t];
      if (wB == 0) continue;
      final int wF = total - wB;
      if (wF == 0) break;
      sumB += t * hist[t];
      final double mB = sumB / wB;
      final double mF = (sum - sumB) / wF;
      final double between = wB * wF * (mB - mF) * (mB - mF);
      if (between > maxVar) {
        maxVar = between;
        plateauStart = t;
        plateauEnd = t;
      } else if (between == maxVar) {
        plateauEnd = t;
      }
    }
    return (plateauStart + plateauEnd) ~/ 2;
  }

  /// Mean luminance over a sub-rectangle — used to detect night conditions and
  /// to down-weight perception confidence accordingly.
  static double meanLuminance(
    Uint8List gray,
    int width,
    int height, {
    int x0 = 0,
    int y0 = 0,
    int? x1,
    int? y1,
    int stride = 4,
  }) {
    final int ex = (x1 ?? width).clamp(0, width);
    final int ey = (y1 ?? height).clamp(0, height);
    int sum = 0;
    int count = 0;
    for (int y = y0; y < ey; y += stride) {
      final int row = y * width;
      for (int x = x0; x < ex; x += stride) {
        sum += gray[row + x];
        count++;
      }
    }
    return count == 0 ? 0 : sum / count;
  }
}

/// Output of [ImagePreprocessing.letterbox] with the inverse mapping needed to
/// place model outputs back onto the original frame.
class LetterboxResult {
  const LetterboxResult({
    required this.bytes,
    required this.side,
    required this.scale,
    required this.padX,
    required this.padY,
  });

  final Uint8List bytes;
  final int side;
  final double scale;
  final int padX;
  final int padY;

  /// Map a coordinate in the letterboxed square (0..1) back to the original
  /// image's normalised coordinates (0..1).
  (double, double) toOriginalNormalized(
    double nx,
    double ny,
    int originalWidth,
    int originalHeight,
  ) {
    final double px = nx * side - padX;
    final double py = ny * side - padY;
    return (
      (px / scale) / originalWidth,
      (py / scale) / originalHeight,
    );
  }
}
