plugins {
    id("com.android.application")
    id("kotlin-android")
    id("dev.flutter.flutter-gradle-plugin")
}

android {
    namespace = "com.aicar.aicar"
    compileSdk = flutter.compileSdkVersion
    ndkVersion = flutter.ndkVersion

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_11
        targetCompatibility = JavaVersion.VERSION_11
        // Several plugins in the dependency graph use APIs newer than
        // minSdk; desugaring keeps them working on Android 8.
        isCoreLibraryDesugaringEnabled = true
    }

    kotlinOptions {
        jvmTarget = JavaVersion.VERSION_11.toString()
    }

    defaultConfig {
        applicationId = "com.aicar.aicar"
        // CameraX, the sensor APIs and the TFLite GPU delegate are all
        // comfortably available from API 26.
        minSdk = 26
        targetSdk = flutter.targetSdkVersion
        versionCode = flutter.versionCode
        versionName = flutter.versionName

        // No ndk.abiFilters here on purpose. It does not filter the prebuilt
        // JNI libraries that arrive from plugin AARs (TensorFlow Lite's, in
        // particular), so it does not shrink the APK the way it looks as
        // though it should — and setting it makes Gradle reject Flutter's
        // own `--split-per-abi`, which is the supported way to produce a
        // per-architecture build. Use the build flags in README.md instead.
    }

    // TFLite models are already compressed; letting the packager compress
    // them again breaks memory-mapped loading, which is how the interpreter
    // avoids copying a 12 MB model into the heap.
    androidResources {
        noCompress.add("tflite")
        noCompress.add("bin")
    }

    buildTypes {
        debug {
            isMinifyEnabled = false
        }
        release {
            // Signing with the debug keys so `flutter build apk --release`
            // works out of the box for a research build. Replace with a real
            // signing config before distributing.
            signingConfig = signingConfigs.getByName("debug")
            isMinifyEnabled = true
            isShrinkResources = true
            proguardFiles(
                getDefaultProguardFile("proguard-android-optimize.txt"),
                "proguard-rules.pro",
            )
        }
    }

    packaging {
        jniLibs {
            // The TFLite GPU delegate ships uncompressed .so files it loads
            // directly.
            useLegacyPackaging = false
        }
    }
}

dependencies {
    coreLibraryDesugaring("com.android.tools:desugar_jdk_libs:2.1.4")

    // TensorFlow Lite plus the two delegates that matter on a Snapdragon:
    // the GPU delegate for float models and NNAPI for quantised ones.
    implementation("org.tensorflow:tensorflow-lite:2.16.1")
    implementation("org.tensorflow:tensorflow-lite-gpu:2.16.1")
    implementation("org.tensorflow:tensorflow-lite-gpu-api:2.16.1")

    implementation("androidx.core:core-ktx:1.13.1")
}

flutter {
    source = "../.."
}
