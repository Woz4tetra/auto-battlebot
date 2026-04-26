// Probe 2: TensorRT enqueueV3 + cudaMemcpy hang detection.
//
// Loads a serialized .engine and runs h2d -> enqueue -> d2h with each call as
// its own stage so a hang attributes to the specific operation. Optionally uses
// an explicit cudaStream_t to disambiguate default-stream serialization.

#include <NvInferRuntime.h>
#include <NvInferRuntimeBase.h>
#include <cuda_runtime.h>

#include <CLI/CLI.hpp>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#include "diag_common.hpp"
#include "spdlog/spdlog.h"

namespace ab = auto_battlebot::diag;

namespace {

class TrtLogger : public nvinfer1::ILogger {
   public:
    void log(Severity severity, nvinfer1::AsciiChar const* msg) noexcept override {
        if (severity <= Severity::kWARNING) {
            spdlog::error("[TensorRT] {}", msg);
        }
    }
};

int64_t volume(const nvinfer1::Dims& dims) {
    if (dims.nbDims <= 0) return 0;
    int64_t v = 1;
    for (int32_t i = 0; i < dims.nbDims; ++i) {
        if (dims.d[i] <= 0) return 0;
        v *= dims.d[i];
    }
    return v;
}

}  // namespace

int main(int argc, char** argv) {
    ab::install_sigsegv_backtrace();
    ab::configure_logger("diag_trt_loop");

    std::string engine_path;
    int iterations = 0;
    double duration_s = 60.0;
    int watchdog_s = 5;
    bool use_stream = false;
    std::string mem_mode = "pinned";  // pinned|pageable|managed
    int concurrent_cpu_load = 0;

    CLI::App app{"TensorRT enqueue/memcpy hang probe"};
    app.add_option("--engine-path", engine_path, "Serialized TRT engine (.engine)")->required();
    app.add_option("--iterations", iterations, "Stop after N iterations (0 = use --duration-s)");
    app.add_option("--duration-s", duration_s, "Stop after this many seconds");
    app.add_option("--watchdog-s", watchdog_s, "Per-stage hang threshold (seconds)");
    app.add_flag("--use-stream", use_stream, "Use explicit cudaStream_t (vs default stream null)");
    app.add_option("--mem-mode", mem_mode, "pinned|pageable|managed");
    app.add_option("--concurrent-cpu-load", concurrent_cpu_load,
                   "Spawn N busy threads to simulate CPU pressure");
    CLI11_PARSE(app, argc, argv);

    ab::StageWatchdog wd(std::chrono::seconds(watchdog_s), "diag_trt_loop");

    DIAG_STAGE(wd, "read_engine_file");
    std::ifstream f(engine_path, std::ios::binary | std::ios::ate);
    if (!f) {
        spdlog::error("cannot open engine: {}", engine_path);
        return ab::kExitSetupFailure;
    }
    const size_t blob_size = static_cast<size_t>(f.tellg());
    f.seekg(0, std::ios::beg);
    std::vector<char> blob(blob_size);
    f.read(blob.data(), static_cast<std::streamsize>(blob_size));
    f.close();

    DIAG_STAGE(wd, "createInferRuntime");
    TrtLogger trt_logger;
    auto* runtime = nvinfer1::createInferRuntime(trt_logger);
    if (!runtime) {
        spdlog::error("createInferRuntime failed");
        return ab::kExitSetupFailure;
    }
    runtime->setEngineHostCodeAllowed(true);

    DIAG_STAGE(wd, "deserializeCudaEngine");
    auto* engine = runtime->deserializeCudaEngine(blob.data(), blob_size);
    if (!engine) {
        spdlog::error("deserializeCudaEngine failed");
        delete runtime;
        return ab::kExitSetupFailure;
    }

    DIAG_STAGE(wd, "createExecutionContext");
    auto* context = engine->createExecutionContext();
    if (!context) {
        spdlog::error("createExecutionContext failed");
        delete engine;
        delete runtime;
        return ab::kExitSetupFailure;
    }

    const int32_t nb_io = engine->getNbIOTensors();
    const char* input_name = nullptr;
    const char* output_name = nullptr;
    for (int32_t i = 0; i < nb_io; ++i) {
        const char* name = engine->getIOTensorName(i);
        const auto mode = engine->getTensorIOMode(name);
        if (mode == nvinfer1::TensorIOMode::kINPUT && !input_name) input_name = name;
        if (mode == nvinfer1::TensorIOMode::kOUTPUT && !output_name) output_name = name;
    }
    if (!input_name || !output_name) {
        spdlog::error("missing input/output tensor");
        delete context;
        delete engine;
        delete runtime;
        return ab::kExitSetupFailure;
    }

    const int64_t input_vol = volume(engine->getTensorShape(input_name));
    const int64_t output_vol = volume(engine->getTensorShape(output_name));
    const size_t input_bytes = static_cast<size_t>(input_vol) * sizeof(float);
    const size_t output_bytes = static_cast<size_t>(output_vol) * sizeof(float);

    void* d_input = nullptr;
    void* d_output = nullptr;
    cudaMalloc(&d_input, input_bytes);
    cudaMalloc(&d_output, output_bytes);
    context->setTensorAddress(input_name, d_input);
    context->setTensorAddress(output_name, d_output);

    float* h_input = nullptr;
    float* h_output = nullptr;
    if (mem_mode == "pinned") {
        cudaMallocHost(reinterpret_cast<void**>(&h_input), input_bytes);
        cudaMallocHost(reinterpret_cast<void**>(&h_output), output_bytes);
    } else if (mem_mode == "managed") {
        cudaMallocManaged(reinterpret_cast<void**>(&h_input), input_bytes);
        cudaMallocManaged(reinterpret_cast<void**>(&h_output), output_bytes);
    } else {
        h_input = static_cast<float*>(std::malloc(input_bytes));
        h_output = static_cast<float*>(std::malloc(output_bytes));
    }
    std::memset(h_input, 0, input_bytes);

    cudaStream_t stream = nullptr;
    if (use_stream) {
        cudaStreamCreate(&stream);
        context->setInputConsumedEvent(nullptr);
    }

    std::atomic<bool> stop_load{false};
    std::vector<std::thread> load_threads;
    for (int i = 0; i < concurrent_cpu_load; ++i) {
        load_threads.emplace_back([&stop_load]() {
            volatile double x = 1.0;
            while (!stop_load.load(std::memory_order_relaxed)) {
                x = x * 1.0000001 + 1e-9;
            }
        });
    }

    spdlog::info("setup complete input_bytes={} output_bytes={} mem_mode={} use_stream={}",
                 input_bytes, output_bytes, mem_mode, use_stream);

    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(static_cast<int>(duration_s * 1000.0));
    char stage_buf[64];
    int total = 0;
    int errors = 0;

    for (int i = 0; iterations == 0 || i < iterations; ++i) {
        if (iterations == 0 && std::chrono::steady_clock::now() > deadline) break;

        std::snprintf(stage_buf, sizeof(stage_buf), "h2d_%d", i);
        wd.stage(stage_buf);
        cudaError_t err;
        if (use_stream) {
            err = cudaMemcpyAsync(d_input, h_input, input_bytes, cudaMemcpyHostToDevice, stream);
        } else {
            err = cudaMemcpy(d_input, h_input, input_bytes, cudaMemcpyHostToDevice);
        }
        if (err != cudaSuccess) {
            ++errors;
            spdlog::warn("h2d error iter={} err={}", i, cudaGetErrorString(err));
            continue;
        }

        std::snprintf(stage_buf, sizeof(stage_buf), "enqueue_%d", i);
        wd.stage(stage_buf);
        const bool ok =
            use_stream ? context->enqueueV3(stream) : context->enqueueV3(nullptr);
        if (!ok) {
            ++errors;
            spdlog::warn("enqueueV3 failed iter={}", i);
            continue;
        }

        std::snprintf(stage_buf, sizeof(stage_buf), "d2h_%d", i);
        wd.stage(stage_buf);
        if (use_stream) {
            err = cudaMemcpyAsync(h_output, d_output, output_bytes, cudaMemcpyDeviceToHost, stream);
            if (err == cudaSuccess) {
                std::snprintf(stage_buf, sizeof(stage_buf), "stream_sync_%d", i);
                wd.stage(stage_buf);
                err = cudaStreamSynchronize(stream);
            }
        } else {
            err = cudaMemcpy(h_output, d_output, output_bytes, cudaMemcpyDeviceToHost);
        }
        if (err != cudaSuccess) {
            ++errors;
            spdlog::warn("d2h error iter={} err={}", i, cudaGetErrorString(err));
            continue;
        }
        ++total;
    }

    stop_load.store(true);
    for (auto& t : load_threads) t.join();

    wd.stage("teardown");
    if (stream) cudaStreamDestroy(stream);
    if (mem_mode == "pinned") {
        cudaFreeHost(h_input);
        cudaFreeHost(h_output);
    } else if (mem_mode == "managed") {
        cudaFree(h_input);
        cudaFree(h_output);
    } else {
        std::free(h_input);
        std::free(h_output);
    }
    cudaFree(d_input);
    cudaFree(d_output);
    delete context;
    delete engine;
    delete runtime;

    spdlog::info("DONE total={} errors={}", total, errors);
    if (total == 0) return ab::kExitSetupFailure;
    if (errors > 0 && static_cast<double>(errors) / static_cast<double>(total + errors) > 0.05) {
        return ab::kExitErrors;
    }
    return ab::kExitPass;
}
