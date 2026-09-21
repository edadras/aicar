# TensorFlow Lite resolves delegate and op classes reflectively, so the
# shrinker cannot see the references and will strip them — producing a
# runtime failure that looks like an unsupported model rather than a missing
# class.
-keep class org.tensorflow.lite.** { *; }
-keep class org.tensorflow.lite.gpu.** { *; }
-keep class org.tensorflow.lite.nnapi.** { *; }
-dontwarn org.tensorflow.lite.**

# The GPU delegate's native loader looks its own classes up by name.
-keep class org.tensorflow.lite.gpu.GpuDelegateFactory$Options { *; }

# Flutter plugin registration.
-keep class io.flutter.** { *; }
-dontwarn io.flutter.embedding.**
