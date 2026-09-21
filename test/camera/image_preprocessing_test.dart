import 'dart:typed_data';

import 'package:aicar/camera/image_preprocessing.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  group('toChannels', () {
    test('is a no-op when the counts already match', () {
      final Uint8List src = Uint8List.fromList(<int>[1, 2, 3, 4, 5, 6]);
      expect(identical(ImagePreprocessing.toChannels(src, 3, 3), src), isTrue);
    });

    test('replicates luminance into every channel', () {
      final Uint8List gray = Uint8List.fromList(<int>[10, 200]);
      expect(ImagePreprocessing.toChannels(gray, 1, 3),
          <int>[10, 10, 10, 200, 200, 200]);
    });

    test('collapses RGB to luminance with the usual weights', () {
      final Uint8List rgb = Uint8List.fromList(<int>[255, 0, 0, 0, 255, 0]);
      final Uint8List gray = ImagePreprocessing.toChannels(rgb, 3, 1);
      expect(gray[0], 76); // 255 * 0.299
      expect(gray[1], 150); // 255 * 0.587
    });

    test('produces exactly the byte count an input tensor expects', () {
      // The reason this helper exists: a buffer that is too short does not
      // fail loudly. The native runtime copies what it is given and leaves
      // the rest of the tensor holding the previous frame, so the detector
      // sees a third of an image stitched onto stale memory.
      const int w = 320;
      const int h = 320;
      expect(ImagePreprocessing.toChannels(Uint8List(w * h), 1, 3),
          hasLength(w * h * 3));
    });
  });

  group('resize', () {
    test('a stretch to a square keeps content at proportional positions', () {
      // 4x2 RGB, left half red, right half blue.
      final Uint8List src = Uint8List(4 * 2 * 3);
      for (int y = 0; y < 2; y++) {
        for (int x = 0; x < 4; x++) {
          final int i = (y * 4 + x) * 3;
          if (x < 2) {
            src[i] = 255;
          } else {
            src[i + 2] = 255;
          }
        }
      }

      final Uint8List out = ImagePreprocessing.resize(src, 4, 2, 8, 8);
      expect(out, hasLength(8 * 8 * 3));

      int px(int x, int y, int c) => out[(y * 8 + x) * 3 + c];
      expect(px(0, 4, 0), greaterThan(200), reason: 'left stays red');
      expect(px(7, 4, 2), greaterThan(200), reason: 'right stays blue');
    });

    test('returns the source untouched when the size already matches', () {
      final Uint8List src = Uint8List(3 * 3 * 3);
      expect(identical(ImagePreprocessing.resize(src, 3, 3, 3, 3), src), isTrue);
    });
  });
}
