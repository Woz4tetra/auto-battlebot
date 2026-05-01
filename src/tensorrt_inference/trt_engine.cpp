#include "tensorrt_inference/trt_engine.hpp"

#include <NvInferRuntime.h>
#include <NvInferRuntimeBase.h>
#include <cuda_runtime.h>
#include <spdlog/spdlog.h>

#include <chrono>
#include <cstring>
#include <fstream>

namespace auto_battlebot {
namespace {
constexpr double kExecuteWarnMs = 120.0;
constexpr double kMemcpyWarnMs = 80.0;
// Minimal TensorRT logger (only log errors/warnings via spdlog).
class TrtLogger : public nvinfer1::ILogger {
   public:
    void log(Severity severity, nvinfer1::AsciiChar const* msg) noexcept override {
        if (severity <= Severity::kWARNING) {
            spdlog::error("[TensorRT] {}", msg);
        }
    }
};

TrtLogger g_trt_logger;

// Compute volume (product of dimensions). Returns 0 if dims are invalid.
int64_t volume(const nvinfer1::Dims& dims) {
    if (dims.nbDims <= 0) return 0;
    int64_t v = 1;
    for (int32_t i = 0; i < dims.nbDims; ++i) {
        if (dims.d[i] <= 0) return 0;
        v *= dims.d[i];
    }
    return v;
}

std::vector<int64_t> dimsToVector(const nvinfer1::Dims& dims) {
    std::vector<int64_t> out;
    out.reserve(static_cast<size_t>(dims.nbDims));
    for (int32_t i = 0; i < dims.nbDims; ++i) out.push_back(dims.d[i]);
    return out;
}
}  // namespace

TrtEngine::~TrtEngine() {
    if (d_input_) {
        cudaFree(d_input_);
        d_input_ = nullptr;
    }
    for (void* ptr : d_outputs_) {
        if (ptr) cudaFree(ptr);
    }
    d_outputs_.clear();
    d_output_ = nullptr;
    auto* ctx = static_cast<nvinfer1::IExecutionContext*>(context_);
    auto* eng = static_cast<nvinfer1::ICudaEngine*>(engine_);
    auto* rt = static_cast<nvinfer1::IRuntime*>(runtime_);
    if (ctx) {
        delete ctx;
        context_ = nullptr;
    }
    if (eng) {
        delete eng;
        engine_ = nullptr;
    }
    if (rt) {
        delete rt;
        runtime_ = nullptr;
    }
}

bool TrtEngine::load(const std::string& engine_path) {
    std::ifstream f(engine_path, std::ios::binary | std::ios::ate);
    if (!f) {
        spdlog::error("TrtEngine: cannot open file: {}", engine_path);
        return false;
    }
    const size_t size = static_cast<size_t>(f.tellg());
    f.seekg(0, std::ios::beg);
    std::vector<char> blob(size);
    if (!f.read(blob.data(), static_cast<std::streamsize>(size))) {
        spdlog::error("TrtEngine: failed to read engine file");
        return false;
    }
    f.close();

    nvinfer1::IRuntime* rt = nvinfer1::createInferRuntime(g_trt_logger);
    if (!rt) {
        spdlog::error("TrtEngine: createInferRuntime failed");
        return false;
    }
    runtime_ = rt;

    // Allow version-compatible engines (built with VERSION_COMPATIBLE) to load.
    rt->setEngineHostCodeAllowed(true);

    nvinfer1::ICudaEngine* eng = rt->deserializeCudaEngine(blob.data(), size);
    if (!eng) {
        spdlog::error("TrtEngine: deserializeCudaEngine failed.");
        delete rt;
        runtime_ = nullptr;
        engine_ = nullptr;
        return false;
    }
    engine_ = eng;

    nvinfer1::IExecutionContext* ctx = eng->createExecutionContext();
    if (!ctx) {
        spdlog::error("TrtEngine: createExecutionContext failed");
        delete eng;
        delete rt;
        engine_ = nullptr;
        runtime_ = nullptr;
        return false;
    }
    context_ = ctx;

    const int32_t nb_io = eng->getNbIOTensors();
    if (nb_io < 2) {
        spdlog::error("TrtEngine: expected at least 2 IO tensors, got {}", nb_io);
        delete ctx;
        delete eng;
        delete rt;
        context_ = nullptr;
        engine_ = nullptr;
        runtime_ = nullptr;
        return false;
    }

    const char* input_name = nullptr;
    std::vector<std::string> output_names;
    std::vector<int32_t> output_io_indices;
    output_infos_.clear();
    d_outputs_.clear();
    d_output_ = nullptr;
    for (int32_t i = 0; i < nb_io; ++i) {
        const char* name = eng->getIOTensorName(i);
        const nvinfer1::TensorIOMode mode = eng->getTensorIOMode(name);
        if (mode == nvinfer1::TensorIOMode::kINPUT) {
            input_name = name;
            input_io_index_ = i;
        } else if (mode == nvinfer1::TensorIOMode::kOUTPUT) {
            output_names.emplace_back(name);
            output_io_indices.push_back(i);
        }
    }
    if (!input_name || output_names.empty()) {
        spdlog::error("TrtEngine: could not identify input/output tensors by mode");
        delete ctx;
        delete eng;
        delete rt;
        context_ = nullptr;
        engine_ = nullptr;
        runtime_ = nullptr;
        return false;
    }

    nvinfer1::Dims input_dims = eng->getTensorShape(input_name);
    const int64_t input_vol = volume(input_dims);
    if (input_vol <= 0) {
        spdlog::error("TrtEngine: invalid input shape");
        delete ctx;
        delete eng;
        delete rt;
        context_ = nullptr;
        engine_ = nullptr;
        runtime_ = nullptr;
        return false;
    }

    cudaError_t err = cudaMalloc(&d_input_, static_cast<size_t>(input_vol) * sizeof(float));
    if (err != cudaSuccess) {
        spdlog::error("TrtEngine: cudaMalloc input failed: {}", cudaGetErrorString(err));
        delete ctx;
        delete eng;
        delete rt;
        context_ = nullptr;
        engine_ = nullptr;
        runtime_ = nullptr;
        return false;
    }
    if (!ctx->setTensorAddress(input_name, d_input_)) {
        spdlog::error("TrtEngine: setTensorAddress failed");
        cudaFree(d_input_);
        d_input_ = nullptr;
        delete ctx;
        delete eng;
        delete rt;
        context_ = nullptr;
        engine_ = nullptr;
        runtime_ = nullptr;
        return false;
    }

    int64_t first_output_vol = 0;
    std::vector<int64_t> first_output_shape;
    for (size_t i = 0; i < output_names.size(); ++i) {
        const std::string& output_name = output_names[i];
        const int32_t io_index = output_io_indices[i];
        nvinfer1::Dims output_dims = eng->getTensorShape(output_name.c_str());
        const int64_t output_vol = volume(output_dims);
        if (output_vol <= 0) {
            spdlog::error("TrtEngine: invalid output shape for tensor {}", output_name);
            cudaFree(d_input_);
            d_input_ = nullptr;
            delete ctx;
            delete eng;
            delete rt;
            context_ = nullptr;
            engine_ = nullptr;
            runtime_ = nullptr;
            return false;
        }

        void* d_output_ptr = nullptr;
        err = cudaMalloc(&d_output_ptr, static_cast<size_t>(output_vol) * sizeof(float));
        if (err != cudaSuccess) {
            spdlog::error("TrtEngine: cudaMalloc output '{}' failed: {}", output_name,
                          cudaGetErrorString(err));
            cudaFree(d_input_);
            d_input_ = nullptr;
            for (void* ptr : d_outputs_) {
                if (ptr) cudaFree(ptr);
            }
            d_outputs_.clear();
            delete ctx;
            delete eng;
            delete rt;
            context_ = nullptr;
            engine_ = nullptr;
            runtime_ = nullptr;
            return false;
        }
        d_outputs_.push_back(d_output_ptr);

        if (!ctx->setTensorAddress(output_name.c_str(), d_output_ptr)) {
            spdlog::error("TrtEngine: setTensorAddress failed for output {}", output_name);
            cudaFree(d_input_);
            d_input_ = nullptr;
            for (void* ptr : d_outputs_) {
                if (ptr) cudaFree(ptr);
            }
            d_outputs_.clear();
            delete ctx;
            delete eng;
            delete rt;
            context_ = nullptr;
            engine_ = nullptr;
            runtime_ = nullptr;
            return false;
        }

        OutputTensorInfo info;
        info.name = output_name;
        info.shape = dimsToVector(output_dims);
        info.io_index = io_index;
        info.num_elements = output_vol;
        output_infos_.push_back(info);

        if (i == 0) {
            first_output_vol = output_vol;
            first_output_shape = info.shape;
            output_io_index_ = io_index;
            d_output_ = d_output_ptr;
        }
    }

    input_shape_ = dimsToVector(input_dims);
    input_num_elements_ = input_vol;
    output_shape_ = first_output_shape;
    output_num_elements_ = first_output_vol;

    {
        std::string in_shape_str, out_shape_str;
        for (size_t i = 0; i < input_shape_.size(); ++i)
            in_shape_str += (i ? ", " : "") + std::to_string(input_shape_[i]);
        for (size_t i = 0; i < output_shape_.size(); ++i) {
            out_shape_str += (i ? ", " : "") + std::to_string(output_shape_[i]);
        }
        spdlog::info("TrtEngine: input \"{}\" shape [{}], outputs [{}], first output shape [{}]",
                     input_name, in_shape_str, output_infos_.size(), out_shape_str);
    }

    return true;
}

std::vector<int64_t> TrtEngine::getInputShape() const { return input_shape_; }

std::vector<int64_t> TrtEngine::getOutputShape() const { return output_shape_; }

bool TrtEngine::execute(const float* host_input, float* host_output) {
    if (!context_ || !d_input_ || !d_output_) return false;

    const auto exec_start = std::chrono::steady_clock::now();
    const auto h2d_start = exec_start;
    cudaError_t err = cudaMemcpy(d_input_, host_input, getInputSizeBytes(), cudaMemcpyHostToDevice);
    const double h2d_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - h2d_start)
            .count();
    if (err != cudaSuccess) {
        spdlog::error("TrtEngine: cudaMemcpy H2D failed: {}", cudaGetErrorString(err));
        return false;
    }

    auto* ctx = static_cast<nvinfer1::IExecutionContext*>(context_);
    const auto enqueue_start = std::chrono::steady_clock::now();
    if (!ctx->enqueueV3(nullptr)) {
        spdlog::error("TrtEngine: executeV2 failed");
        return false;
    }
    const double enqueue_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - enqueue_start)
            .count();

    const auto d2h_start = std::chrono::steady_clock::now();
    err = cudaMemcpy(host_output, d_output_, getOutputSizeBytes(), cudaMemcpyDeviceToHost);
    const double d2h_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - d2h_start)
            .count();
    if (err != cudaSuccess) {
        spdlog::error("TrtEngine: cudaMemcpy D2H failed: {}", cudaGetErrorString(err));
        return false;
    }

    const double total_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - exec_start)
            .count();
    if (total_ms > kExecuteWarnMs || h2d_ms > kMemcpyWarnMs || d2h_ms > kMemcpyWarnMs) {
        spdlog::warn(
            "TrtEngine::execute slow path total_ms={:.2f} h2d_ms={:.2f} enqueue_ms={:.2f} "
            "d2h_ms={:.2f}",
            total_ms, h2d_ms, enqueue_ms, d2h_ms);
    }

    return true;
}

bool TrtEngine::execute_multi(const float* host_input, const std::vector<float*>& host_outputs) {
    if (!context_ || !d_input_ || d_outputs_.empty()) return false;
    if (host_outputs.size() != d_outputs_.size()) {
        spdlog::error("TrtEngine: execute_multi output count mismatch host={} engine={}",
                      host_outputs.size(), d_outputs_.size());
        return false;
    }

    const auto exec_start = std::chrono::steady_clock::now();
    const auto h2d_start = exec_start;
    cudaError_t err = cudaMemcpy(d_input_, host_input, getInputSizeBytes(), cudaMemcpyHostToDevice);
    const double h2d_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - h2d_start)
            .count();
    if (err != cudaSuccess) {
        spdlog::error("TrtEngine: cudaMemcpy H2D failed: {}", cudaGetErrorString(err));
        return false;
    }

    auto* ctx = static_cast<nvinfer1::IExecutionContext*>(context_);
    const auto enqueue_start = std::chrono::steady_clock::now();
    if (!ctx->enqueueV3(nullptr)) {
        spdlog::error("TrtEngine: execute_multi enqueueV3 failed");
        return false;
    }
    const double enqueue_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - enqueue_start)
            .count();

    double d2h_total_ms = 0.0;
    for (size_t i = 0; i < d_outputs_.size(); ++i) {
        const size_t bytes = static_cast<size_t>(output_infos_[i].num_elements) * sizeof(float);
        const auto d2h_start = std::chrono::steady_clock::now();
        err = cudaMemcpy(host_outputs[i], d_outputs_[i], bytes, cudaMemcpyDeviceToHost);
        const double d2h_ms =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - d2h_start)
                .count();
        d2h_total_ms += d2h_ms;
        if (err != cudaSuccess) {
            spdlog::error("TrtEngine: cudaMemcpy D2H failed for output {}: {}",
                          output_infos_[i].name, cudaGetErrorString(err));
            return false;
        }
    }

    const double total_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - exec_start)
            .count();
    if (total_ms > kExecuteWarnMs || h2d_ms > kMemcpyWarnMs || d2h_total_ms > kMemcpyWarnMs) {
        spdlog::warn(
            "TrtEngine::execute_multi slow path total_ms={:.2f} h2d_ms={:.2f} "
            "enqueue_ms={:.2f} d2h_total_ms={:.2f} outputs={}",
            total_ms, h2d_ms, enqueue_ms, d2h_total_ms, d_outputs_.size());
    }
    return true;
}

}  // namespace auto_battlebot
