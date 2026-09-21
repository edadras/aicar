package com.aicar.aicar

import android.content.Context
import android.util.Log
import org.tensorflow.lite.Interpreter
import org.tensorflow.lite.gpu.CompatibilityList
import org.tensorflow.lite.gpu.GpuDelegate
import org.tensorflow.lite.nnapi.NnApiDelegate
import java.io.File
import java.io.FileInputStream
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.channels.FileChannel

/**
 * Loads and runs TensorFlow Lite models.
 *
 * The heavy lifting is deliberately on this side of the platform channel
 * rather than in Dart:
 *
 *  * The GPU and NNAPI delegates only exist in the Android TFLite library.
 *    On a Galaxy S23 the Adreno GPU delegate is roughly 4x faster than
 *    multi-threaded CPU for a float detector, which is the difference between
 *    a usable frame rate and a slideshow.
 *  * Input and output tensors are written straight into the interpreter's
 *    direct [ByteBuffer]s. Flutter's standard message codec passes
 *    `Float32List` and `Uint8List` as raw byte buffers with no per-element
 *    boxing, so a 4.9 MB input tensor crosses the channel as one memcpy.
 *
 * Every model is a [Session] holding its own interpreter and buffers, so
 * several models can be resident at once (detector, depth, segmentation)
 * without reallocating per frame.
 */
class TfLiteRunner(private val context: Context) {

    companion object {
        private const val TAG = "TfLiteRunner"
    }

    private class Session(
        val interpreter: Interpreter,
        val gpuDelegate: GpuDelegate?,
        val nnApiDelegate: NnApiDelegate?,
        val inputBuffer: ByteBuffer,
        val outputBuffers: Map<Int, ByteBuffer>,
        val outputShapes: List<IntArray>,
        val quantized: Boolean,
    )

    private val sessions = mutableMapOf<Int, Session>()
    private var nextHandle = 1

    fun isAvailable(): Boolean = true

    /** Delegates this specific device actually accepts. */
    fun supportedDelegates(): List<String> {
        val result = mutableListOf("cpu")
        try {
            if (CompatibilityList().isDelegateSupportedOnThisDevice) {
                result.add("gpu")
            }
        } catch (t: Throwable) {
            Log.w(TAG, "GPU delegate probe failed", t)
        }
        // NNAPI is present from API 27; whether it helps depends on the model
        // being quantised, which is why the descriptor chooses per model.
        if (android.os.Build.VERSION.SDK_INT >= 27) result.add("nnapi")
        return result
    }

    fun loadModel(
        path: String,
        isAsset: Boolean,
        delegate: String,
        numThreads: Int,
        quantized: Boolean,
    ): Int {
        val buffer = if (isAsset) loadAsset(path) else loadFile(path)

        val options = Interpreter.Options().apply {
            setNumThreads(numThreads.coerceIn(1, 8))
            // XNNPACK is the CPU fallback and is a large win on its own.
            setUseXNNPACK(true)
        }

        var gpu: GpuDelegate? = null
        var nnapi: NnApiDelegate? = null

        when (delegate) {
            "gpu" -> {
                try {
                    if (CompatibilityList().isDelegateSupportedOnThisDevice) {
                        gpu = GpuDelegate(
                            CompatibilityList().bestOptionsForThisDevice
                        )
                        options.addDelegate(gpu)
                    } else {
                        Log.w(TAG, "GPU delegate unsupported; using CPU")
                    }
                } catch (t: Throwable) {
                    // A delegate that fails to initialise must not stop the
                    // model loading: falling back to CPU is slower but works.
                    Log.w(TAG, "GPU delegate unavailable; using CPU", t)
                    gpu = null
                }
            }
            "nnapi" -> {
                try {
                    nnapi = NnApiDelegate()
                    options.addDelegate(nnapi)
                } catch (t: Throwable) {
                    Log.w(TAG, "NNAPI delegate unavailable; using CPU", t)
                    nnapi = null
                }
            }
        }

        val interpreter = try {
            Interpreter(buffer, options)
        } catch (t: Throwable) {
            // Some models fail only once a delegate tries to compile them.
            // Retrying on plain CPU turns a hard failure into a slow success.
            Log.w(TAG, "interpreter creation failed with delegate; retrying on CPU", t)
            gpu?.close()
            nnapi?.close()
            gpu = null
            nnapi = null
            Interpreter(buffer, Interpreter.Options().apply {
                setNumThreads(numThreads.coerceIn(1, 8))
                setUseXNNPACK(true)
            })
        }

        val inputTensor = interpreter.getInputTensor(0)
        val inputBuffer = ByteBuffer
            .allocateDirect(inputTensor.numBytes())
            .order(ByteOrder.nativeOrder())

        val outputBuffers = mutableMapOf<Int, ByteBuffer>()
        val outputShapes = mutableListOf<IntArray>()
        for (i in 0 until interpreter.outputTensorCount) {
            val tensor = interpreter.getOutputTensor(i)
            outputBuffers[i] = ByteBuffer
                .allocateDirect(tensor.numBytes())
                .order(ByteOrder.nativeOrder())
            outputShapes.add(tensor.shape())
        }

        val handle = nextHandle++
        sessions[handle] = Session(
            interpreter = interpreter,
            gpuDelegate = gpu,
            nnApiDelegate = nnapi,
            inputBuffer = inputBuffer,
            outputBuffers = outputBuffers,
            outputShapes = outputShapes,
            quantized = quantized,
        )

        Log.i(
            TAG,
            "loaded $path as handle $handle " +
                "(input ${inputTensor.shape().joinToString("x")}, " +
                "${interpreter.outputTensorCount} outputs, " +
                "delegate=${if (gpu != null) "gpu" else if (nnapi != null) "nnapi" else "cpu"})"
        )
        return handle
    }

    /** Run a float model. Returns output tensors as float arrays. */
    fun run(handle: Int, input: FloatArray): Map<String, Any> {
        val session = sessions[handle]
            ?: throw IllegalArgumentException("unknown model handle $handle")

        val started = System.nanoTime()

        session.inputBuffer.rewind()
        val floatView = session.inputBuffer.asFloatBuffer()
        floatView.rewind()
        val count = minOf(input.size, floatView.capacity())
        floatView.put(input, 0, count)
        session.inputBuffer.rewind()

        for (buffer in session.outputBuffers.values) buffer.rewind()
        session.interpreter.runForMultipleInputsOutputs(
            arrayOf<Any>(session.inputBuffer),
            session.outputBuffers as Map<Int, Any>,
        )

        return collectOutputs(session, System.nanoTime() - started)
    }

    /** Run a quantised model whose input is uint8. */
    fun runQuantized(handle: Int, input: ByteArray): Map<String, Any> {
        val session = sessions[handle]
            ?: throw IllegalArgumentException("unknown model handle $handle")

        val started = System.nanoTime()

        session.inputBuffer.rewind()
        val count = minOf(input.size, session.inputBuffer.capacity())
        session.inputBuffer.put(input, 0, count)
        session.inputBuffer.rewind()

        for (buffer in session.outputBuffers.values) buffer.rewind()
        session.interpreter.runForMultipleInputsOutputs(
            arrayOf<Any>(session.inputBuffer),
            session.outputBuffers as Map<Int, Any>,
        )

        return collectOutputs(session, System.nanoTime() - started)
    }

    private fun collectOutputs(session: Session, elapsedNanos: Long): Map<String, Any> {
        val outputs = mutableListOf<FloatArray>()
        val shapes = mutableListOf<List<Int>>()

        for (i in 0 until session.interpreter.outputTensorCount) {
            val tensor = session.interpreter.getOutputTensor(i)
            val buffer = session.outputBuffers[i]!!
            buffer.rewind()

            val values: FloatArray
            if (tensor.dataType() == org.tensorflow.lite.DataType.UINT8) {
                // Dequantise here rather than in Dart: the scale and zero
                // point live on this side, and doing it in the same pass that
                // reads the buffer avoids a second copy.
                val quantization = tensor.quantizationParams()
                val bytes = ByteArray(buffer.remaining())
                buffer.get(bytes)
                values = FloatArray(bytes.size) { index ->
                    (bytes[index].toInt() and 0xFF).let {
                        (it - quantization.zeroPoint) * quantization.scale
                    }
                }
            } else {
                val floatView = buffer.asFloatBuffer()
                values = FloatArray(floatView.remaining())
                floatView.get(values)
            }

            outputs.add(values)
            shapes.add(tensor.shape().toList())
        }

        return mapOf(
            "outputs" to outputs,
            "shapes" to shapes,
            "micros" to (elapsedNanos / 1000),
        )
    }

    fun unload(handle: Int) {
        sessions.remove(handle)?.let { session ->
            session.interpreter.close()
            session.gpuDelegate?.close()
            session.nnApiDelegate?.close()
            Log.i(TAG, "unloaded handle $handle")
        }
    }

    fun disposeAll() {
        for (handle in sessions.keys.toList()) unload(handle)
    }

    private fun loadFile(path: String): ByteBuffer {
        val file = File(path)
        require(file.exists()) { "model file not found: $path" }
        FileInputStream(file).use { stream ->
            return stream.channel.map(
                FileChannel.MapMode.READ_ONLY, 0, file.length()
            )
        }
    }

    private fun loadAsset(assetPath: String): ByteBuffer {
        val key = io.flutter.FlutterInjector.instance()
            .flutterLoader()
            .getLookupKeyForAsset(assetPath)
        context.assets.openFd(key).use { descriptor ->
            FileInputStream(descriptor.fileDescriptor).use { stream ->
                return stream.channel.map(
                    FileChannel.MapMode.READ_ONLY,
                    descriptor.startOffset,
                    descriptor.declaredLength,
                )
            }
        }
    }
}
