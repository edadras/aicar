package com.aicar.aicar

import android.os.Bundle
import android.util.Log
import android.view.WindowManager
import io.flutter.embedding.android.FlutterActivity
import io.flutter.embedding.engine.FlutterEngine
import io.flutter.plugin.common.MethodCall
import io.flutter.plugin.common.MethodChannel

/**
 * Hosts the Flutter engine and exposes the native inference runtime.
 *
 * The method channel is intentionally thin: it loads models, runs tensors and
 * unloads. All interpretation of what the tensors *mean* stays in Dart, so
 * swapping a model architecture never requires touching Kotlin.
 */
class MainActivity : FlutterActivity() {

    companion object {
        private const val TAG = "MainActivity"
        private const val INFERENCE_CHANNEL = "com.aicar/inference"
        private const val SYSTEM_CHANNEL = "com.aicar/system"
    }

    private var runner: TfLiteRunner? = null

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        // A dashboard HUD must stay lit; the Dart side also holds a wakelock,
        // and this covers the window before Dart is running.
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
    }

    override fun configureFlutterEngine(flutterEngine: FlutterEngine) {
        super.configureFlutterEngine(flutterEngine)

        val inference = TfLiteRunner(applicationContext)
        runner = inference

        MethodChannel(
            flutterEngine.dartExecutor.binaryMessenger,
            INFERENCE_CHANNEL,
        ).setMethodCallHandler { call, result ->
            handleInferenceCall(inference, call, result)
        }

        MethodChannel(
            flutterEngine.dartExecutor.binaryMessenger,
            SYSTEM_CHANNEL,
        ).setMethodCallHandler { call, result ->
            handleSystemCall(call, result)
        }
    }

    private fun handleInferenceCall(
        inference: TfLiteRunner,
        call: MethodCall,
        result: MethodChannel.Result,
    ) {
        try {
            when (call.method) {
                "isAvailable" -> result.success(inference.isAvailable())

                "supportedDelegates" ->
                    result.success(inference.supportedDelegates())

                "loadModel" -> {
                    val handle = inference.loadModel(
                        path = call.argument<String>("path")!!,
                        isAsset = call.argument<Boolean>("isAsset") ?: false,
                        delegate = call.argument<String>("delegate") ?: "cpu",
                        numThreads = call.argument<Int>("numThreads") ?: 4,
                        quantized = call.argument<Boolean>("quantized") ?: false,
                    )
                    result.success(handle)
                }

                "run" -> {
                    val handle = call.argument<Int>("handle")!!
                    // Flutter's codec delivers a Float32List as a FloatArray
                    // with no per-element conversion.
                    val input = call.argument<FloatArray>("input")!!
                    result.success(inference.run(handle, input))
                }

                "runQuantized" -> {
                    val handle = call.argument<Int>("handle")!!
                    val input = call.argument<ByteArray>("input")!!
                    result.success(inference.runQuantized(handle, input))
                }

                "unload" -> {
                    inference.unload(call.argument<Int>("handle")!!)
                    result.success(null)
                }

                "disposeAll" -> {
                    inference.disposeAll()
                    result.success(null)
                }

                else -> result.notImplemented()
            }
        } catch (t: Throwable) {
            // Surfacing the message rather than crashing lets the Dart side
            // mark the stage degraded and carry on with the classical path.
            Log.e(TAG, "inference call ${call.method} failed", t)
            result.error("inference_error", t.message, null)
        }
    }

    private fun handleSystemCall(call: MethodCall, result: MethodChannel.Result) {
        when (call.method) {
            "thermalStatus" -> result.success(readThermalStatus())
            else -> result.notImplemented()
        }
    }

    /**
     * Thermal state, as both a status and a forecast.
     *
     * `currentThermalStatus` says what is happening now. `getThermalHeadroom`
     * (API 30+) predicts where the SoC will be in the requested number of
     * seconds, normalised so 1.0 is the throttling point — which is the more
     * useful of the two here, because it lets the governor shed load *before*
     * the clocks come down rather than after.
     */
    private fun readThermalStatus(): Map<String, Any> {
        return try {
            if (android.os.Build.VERSION.SDK_INT >= 29) {
                val manager = getSystemService(android.os.PowerManager::class.java)
                val out = mutableMapOf<String, Any>(
                    "status" to manager.currentThermalStatus,
                    "available" to true,
                )
                if (android.os.Build.VERSION.SDK_INT >= 30) {
                    // A 60 s forecast: long enough to be actionable, short
                    // enough that the estimate is still meaningful.
                    val headroom = manager.getThermalHeadroom(60)
                    if (!headroom.isNaN()) out["headroom"] = headroom
                }
                out
            } else {
                mapOf("available" to false)
            }
        } catch (t: Throwable) {
            mapOf("available" to false)
        }
    }

    override fun onDestroy() {
        runner?.disposeAll()
        runner = null
        super.onDestroy()
    }
}
