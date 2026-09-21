# Bundled models (optional)

This directory is **empty on purpose**. The app ships with no neural weights:
they are large, separately licensed, and upgradable independently of the app,
so the normal way to install one is to copy a `.tflite` file into the app's
runtime model directory and press refresh on the **AI models** screen. See
`docs/MODELS.md`.

If you would rather bake a model into the APK — for a fleet build, or a
reproducible experiment where the weights must travel with the binary — put
the `.tflite` file here and set `isBundledAsset: true` in its descriptor. The
native runtime then loads it through Flutter's asset lookup instead of from
the filesystem.

Note that Gradle is configured not to compress `.tflite` files
(`androidResources { noCompress }` in `android/app/build.gradle.kts`), because
a compressed asset cannot be memory-mapped and the interpreter would have to
copy the whole model into the heap.

The directory itself is tracked (via `.gitkeep`) because `pubspec.yaml`
declares it as an asset directory, and a Flutter build errors on a declared
directory that does not exist.
