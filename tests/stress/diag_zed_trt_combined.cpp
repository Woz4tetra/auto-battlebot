// Probe 4: ZED + TRT in the same process, separate threads.
//
// Some hangs only manifest when the live ZED capture loop and TensorRT inference
// share a CUDA context. This probe runs both concurrently and stages each
// thread's calls separately so a hang attributes to which thread/op stalled.

#include <NvInferRuntime.h>
#include <NvInferRuntimeBase.h>
#include <cuda_runtime.h>

#include <CLI/CLI.hpp>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sl/Camera.hpp>
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
    ab::configure_logger("diag_zed_trt_combined");

    std::string engine_path;
    std::string svo_path;
    double duration_s = 120.0;
    int watchdog_s = 8;
    int fps = 30;

    CLI::App app{"ZED+TRT combined hang probe"};
    app.add_option("--engine-path", engine_path, "Serialized TRT engine (.engine)")->required();
    app.add_option("--zed-svo", svo_path, "Replay an SVO file instead of live camera (control)");
    app.add_option("--duration-s", duration_s, "Total run time in seconds");
    app.add_option("--watchdog-s", watchdog_s, "Per-stage hang threshold (seconds)");
    app.add_option("--fps", fps, "Camera FPS");
    CLI11_PARSE(app, argc, argv);

    ab::StageWatchdog wd(std::chrono::seconds(watchdog_s), "diag_zed_trt_combined");

    // Open ZED.
    DIAG_STAGE(wd, "zed_open");
    sl::Camera zed;
    sl::InitParameters params;
    params.camera_fps = fps;
    params.camera_resolution = sl::RESOLUTION::HD720;
    params.depth_mode = sl::DEPTH_MODE::NEURAL;
    params.coordinate_system = sl::COORDINATE_SYSTEM::IMAGE;
    params.coordinate_units = sl::UNIT::METER;
    if (!svo_path.empty()) {
        params.input.setFromSVOFile(svo_path.c_str());
        params.svo_real_time_mode = true;
    }
    if (zed.open(params) != sl::ERROR_CODE::SUCCESS) {
        spdlog::error("zed.open failed");
        return ab::kExitSetupFailure;
    }

    // Load TRT engine.
    DIAG_STAGE(wd, "trt_load");
    std::ifstream f(engine_path, std::ios::binary | std::ios::ate);
    if (!f) {
        spdlog::error("cannot open engine: {}", engine_path);
        zed.close();
        return ab::kExitSetupFailure;
    }
    const size_t blob_size = static_cast<size_t>(f.tellg());
    f.seekg(0, std::ios::beg);
    std::vector<char> blob(blob_size);
    f.read(blob.data(), static_cast<std::streamsize>(blob_size));
    f.close();

    TrtLogger trt_logger;
    auto* runtime = nvinfer1::createInferRuntime(trt_logger);
    runtime->setEngineHostCodeAllowed(true);
    auto* engine = runtime->deserializeCudaEngine(blob.data(), blob_size);
    auto* context = engine->createExecutionContext();
    if (!context) {
        spdlog::error("createExecutionContext failed");
        delete engine;
        delete runtime;
        zed.close();
        return ab::kExitSetupFailure;
    }
    const int32_t nb_io = engine->getNbIOTensors();
    const char* in_name = nullptr;
    const char* out_name = nullptr;
    for (int32_t i = 0; i < nb_io; ++i) {
        const char* name = engine->getIOTensorName(i);
        const auto mode = engine->getTensorIOMode(name);
        if (mode == nvinfer1::TensorIOMode::kINPUT && !in_name) in_name = name;
        if (mode == nvinfer1::TensorIOMode::kOUTPUT && !out_name) out_name = name;
    }
    const size_t in_bytes =
        static_cast<size_t>(volume(engine->getTensorShape(in_name))) * sizeof(float);
    const size_t out_bytes =
        static_cast<size_t>(volume(engine->getTensorShape(out_name))) * sizeof(float);
    void* d_in = nullptr;
    void* d_out = nullptr;
    cudaMalloc(&d_in, in_bytes);
    cudaMalloc(&d_out, out_bytes);
    context->setTensorAddress(in_name, d_in);
    context->setTensorAddress(out_name, d_out);
    float* h_in = nullptr;
    float* h_out = nullptr;
    cudaMallocHost(reinterpret_cast<void**>(&h_in), in_bytes);
    cudaMallocHost(reinterpret_cast<void**>(&h_out), out_bytes);
    std::memset(h_in, 0, in_bytes);

    std::atomic<bool> stop{false};
    std::atomic<int> zed_iters{0};
    std::atomic<int> trt_iters{0};
    std::atomic<int> zed_errors{0};
    std::atomic<int> trt_errors{0};

    // ZED thread.
    std::thread zed_thread([&]() {
        sl::Mat rgb_mat;
        sl::Mat depth_mat;
        sl::RuntimeParameters rt;
        char buf[64];
        int i = 0;
        while (!stop.load(std::memory_order_acquire)) {
            std::snprintf(buf, sizeof(buf), "tid=zed grab_%d", i);
            wd.stage(buf);
            const auto err = zed.grab(rt);
            if (err != sl::ERROR_CODE::SUCCESS) {
                ++zed_errors;
                if (!svo_path.empty()) break;
                continue;
            }
            std::snprintf(buf, sizeof(buf), "tid=zed retrieveImage_%d", i);
            wd.stage(buf);
            zed.retrieveImage(rgb_mat, sl::VIEW::LEFT);
            std::snprintf(buf, sizeof(buf), "tid=zed retrieveMeasure_%d", i);
            wd.stage(buf);
            zed.retrieveMeasure(depth_mat, sl::MEASURE::DEPTH);
            ++zed_iters;
            ++i;
        }
    });

    // TRT thread.
    std::thread trt_thread([&]() {
        char buf[64];
        int i = 0;
        while (!stop.load(std::memory_order_acquire)) {
            std::snprintf(buf, sizeof(buf), "tid=trt h2d_%d", i);
            wd.stage(buf);
            if (cudaMemcpy(d_in, h_in, in_bytes, cudaMemcpyHostToDevice) != cudaSuccess) {
                ++trt_errors;
                continue;
            }
            std::snprintf(buf, sizeof(buf), "tid=trt enqueue_%d", i);
            wd.stage(buf);
            if (!context->enqueueV3(nullptr)) {
                ++trt_errors;
                continue;
            }
            std::snprintf(buf, sizeof(buf), "tid=trt d2h_%d", i);
            wd.stage(buf);
            if (cudaMemcpy(h_out, d_out, out_bytes, cudaMemcpyDeviceToHost) != cudaSuccess) {
                ++trt_errors;
                continue;
            }
            ++trt_iters;
            ++i;
        }
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(duration_s * 1000.0)));
    stop.store(true);
    zed_thread.join();
    trt_thread.join();

    wd.stage("teardown");
    cudaFreeHost(h_in);
    cudaFreeHost(h_out);
    cudaFree(d_in);
    cudaFree(d_out);
    delete context;
    delete engine;
    delete runtime;
    zed.close();

    spdlog::info("DONE zed_iters={} zed_errors={} trt_iters={} trt_errors={}", zed_iters.load(),
                 zed_errors.load(), trt_iters.load(), trt_errors.load());
    if (zed_iters.load() == 0 || trt_iters.load() == 0) return ab::kExitSetupFailure;
    if (zed_errors.load() + trt_errors.load() > 0) return ab::kExitErrors;
    return ab::kExitPass;
}
