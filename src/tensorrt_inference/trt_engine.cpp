#include "tensorrt_inference/trt_engine.hpp"

#include <NvInferRuntime.h>
#include <NvInferRuntimeBase.h>
#include <cuda_runtime.h>
#include <spdlog/spdlog.h>

#include <cstring>
#include <fstream>
#include <stdexcept>

namespace auto_battlebot {
namespace {
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
    if (d_output_) {
        cudaFree(d_output_);
        d_output_ = nullptr;
    }
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
    if (nb_io != 2) {
        spdlog::error("TrtEngine: expected 2 IO tensors, got {}", nb_io);
        delete ctx;
        delete eng;
        delete rt;
        context_ = nullptr;
        engine_ = nullptr;
        runtime_ = nullptr;
        return false;
    }

    const char* input_name = nullptr;
    const char* output_name = nullptr;
    for (int32_t i = 0; i < nb_io; ++i) {
        const char* name = eng->getIOTensorName(i);
        const nvinfer1::TensorIOMode mode = eng->getTensorIOMode(name);
        if (mode == nvinfer1::TensorIOMode::kINPUT) {
            input_name = name;
            input_io_index_ = i;
        } else if (mode == nvinfer1::TensorIOMode::kOUTPUT) {
            output_name = name;
            output_io_index_ = i;
        }
    }
    if (!input_name || !output_name) {
        spdlog::error("TrtEngine: could not identify input and output tensors by mode");
        delete ctx;
        delete eng;
        delete rt;
        context_ = nullptr;
        engine_ = nullptr;
        runtime_ = nullptr;
        return false;
    }

    nvinfer1::Dims input_dims = eng->getTensorShape(input_name);
    nvinfer1::Dims output_dims = eng->getTensorShape(output_name);
    const int64_t input_vol = volume(input_dims);
    const int64_t output_vol = volume(output_dims);
    if (input_vol <= 0 || output_vol <= 0) {
        spdlog::error("TrtEngine: invalid input or output shape");
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
    err = cudaMalloc(&d_output_, static_cast<size_t>(output_vol) * sizeof(float));
    if (err != cudaSuccess) {
        spdlog::error("TrtEngine: cudaMalloc output failed: {}", cudaGetErrorString(err));
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

    if (!ctx->setTensorAddress(input_name, d_input_) ||
        !ctx->setTensorAddress(output_name, d_output_)) {
        spdlog::error("TrtEngine: setTensorAddress failed");
        cudaFree(d_input_);
        cudaFree(d_output_);
        d_input_ = nullptr;
        d_output_ = nullptr;
        delete ctx;
        delete eng;
        delete rt;
        context_ = nullptr;
        engine_ = nullptr;
        runtime_ = nullptr;
        return false;
    }

    input_shape_ = dimsToVector(input_dims);
    output_shape_ = dimsToVector(output_dims);
    input_num_elements_ = input_vol;
    output_num_elements_ = output_vol;

    {
        std::string in_shape_str, out_shape_str;
        for (size_t i = 0; i < input_shape_.size(); ++i)
            in_shape_str += (i ? ", " : "") + std::to_string(input_shape_[i]);
        for (size_t i = 0; i < output_shape_.size(); ++i)
            out_shape_str += (i ? ", " : "") + std::to_string(output_shape_[i]);
        spdlog::info("TrtEngine: input \"{}\" shape [{}], output \"{}\" shape [{}]", input_name,
                     in_shape_str, output_name, out_shape_str);
    }

    return true;
}

std::vector<int64_t> TrtEngine::getInputShape() const { return input_shape_; }

std::vector<int64_t> TrtEngine::getOutputShape() const { return output_shape_; }

bool TrtEngine::execute(const float* host_input, float* host_output) {
    if (!context_ || !d_input_ || !d_output_) return false;

    cudaError_t err = cudaMemcpy(d_input_, host_input, getInputSizeBytes(), cudaMemcpyHostToDevice);
    if (err != cudaSuccess) {
        spdlog::error("TrtEngine: cudaMemcpy H2D failed: {}", cudaGetErrorString(err));
        return false;
    }

    // executeV2(bindings): bindings[i] must match getIOTensorName(i).
    void* bindings[2] = {nullptr, nullptr};
    bindings[input_io_index_] = d_input_;
    bindings[output_io_index_] = d_output_;

    auto* ctx = static_cast<nvinfer1::IExecutionContext*>(context_);
    if (!ctx->executeV2(bindings)) {
        spdlog::error("TrtEngine: executeV2 failed");
        return false;
    }

    err = cudaMemcpy(host_output, d_output_, getOutputSizeBytes(), cudaMemcpyDeviceToHost);
    if (err != cudaSuccess) {
        spdlog::error("TrtEngine: cudaMemcpy D2H failed: {}", cudaGetErrorString(err));
        return false;
    }

    return true;
}

}  // namespace auto_battlebot
