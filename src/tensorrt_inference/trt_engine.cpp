#include "tensorrt_inference/trt_engine.hpp"

#include <NvInferRuntime.h>
#include <cuda_runtime.h>
#include <cstring>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace auto_battlebot
{
namespace
{
    // Minimal TensorRT logger (only log errors/warnings to stderr).
    class TrtLogger : public nvinfer1::ILogger
    {
    public:
        void log(Severity severity, nvinfer1::AsciiChar const* msg) noexcept override
        {
            if (severity <= Severity::kWARNING)
            {
                std::cerr << "[TensorRT] " << msg << std::endl;
            }
        }
    };

    TrtLogger g_trt_logger;

    // Compute volume (product of dimensions). Returns 0 if dims are invalid.
    int64_t volume(const nvinfer1::Dims& dims)
    {
        if (dims.nbDims <= 0)
            return 0;
        int64_t v = 1;
        for (int32_t i = 0; i < dims.nbDims; ++i)
        {
            if (dims.d[i] <= 0)
                return 0;
            v *= dims.d[i];
        }
        return v;
    }

    std::vector<int64_t> dimsToVector(const nvinfer1::Dims& dims)
    {
        std::vector<int64_t> out;
        out.reserve(static_cast<size_t>(dims.nbDims));
        for (int32_t i = 0; i < dims.nbDims; ++i)
            out.push_back(dims.d[i]);
        return out;
    }
} // namespace

TrtEngine::~TrtEngine()
{
    if (d_input_)
    {
        cudaFree(d_input_);
        d_input_ = nullptr;
    }
    if (d_output_)
    {
        cudaFree(d_output_);
        d_output_ = nullptr;
    }
    auto* ctx = static_cast<nvinfer1::IExecutionContext*>(context_);
    auto* eng = static_cast<nvinfer1::ICudaEngine*>(engine_);
    auto* rt = static_cast<nvinfer1::IRuntime*>(runtime_);
    if (ctx)
    {
        delete ctx;
        context_ = nullptr;
    }
    if (eng)
    {
        delete eng;
        engine_ = nullptr;
    }
    if (rt)
    {
        delete rt;
        runtime_ = nullptr;
    }
}

bool TrtEngine::load(const std::string& engine_path)
{
    std::ifstream f(engine_path, std::ios::binary | std::ios::ate);
    if (!f)
    {
        std::cerr << "TrtEngine: cannot open file: " << engine_path << std::endl;
        return false;
    }
    const size_t size = static_cast<size_t>(f.tellg());
    f.seekg(0, std::ios::beg);
    std::vector<char> blob(size);
    if (!f.read(blob.data(), static_cast<std::streamsize>(size)))
    {
        std::cerr << "TrtEngine: failed to read engine file" << std::endl;
        return false;
    }
    f.close();

    nvinfer1::IRuntime* rt = nvinfer1::createInferRuntime(g_trt_logger);
    if (!rt)
    {
        std::cerr << "TrtEngine: createInferRuntime failed" << std::endl;
        return false;
    }
    runtime_ = rt;

    nvinfer1::ICudaEngine* eng = rt->deserializeCudaEngine(blob.data(), size);
    if (!eng)
    {
        std::cerr << "TrtEngine: deserializeCudaEngine failed" << std::endl;
        delete rt;
        runtime_ = nullptr;
        engine_ = nullptr;
        return false;
    }
    engine_ = eng;

    nvinfer1::IExecutionContext* ctx = eng->createExecutionContext();
    if (!ctx)
    {
        std::cerr << "TrtEngine: createExecutionContext failed" << std::endl;
        delete eng;
        delete rt;
        engine_ = nullptr;
        runtime_ = nullptr;
        return false;
    }
    context_ = ctx;

    const int32_t nb_io = eng->getNbIOTensors();
    if (nb_io != 2)
    {
        std::cerr << "TrtEngine: expected 2 IO tensors, got " << nb_io << std::endl;
        delete ctx;
        delete eng;
        delete rt;
        context_ = nullptr;
        engine_ = nullptr;
        runtime_ = nullptr;
        return false;
    }

    const char* input_name = eng->getIOTensorName(0);
    const char* output_name = eng->getIOTensorName(1);
    nvinfer1::Dims input_dims = eng->getTensorShape(input_name);
    nvinfer1::Dims output_dims = eng->getTensorShape(output_name);
    const int64_t input_vol = volume(input_dims);
    const int64_t output_vol = volume(output_dims);
    if (input_vol <= 0 || output_vol <= 0)
    {
        std::cerr << "TrtEngine: invalid input or output shape" << std::endl;
        delete ctx;
        delete eng;
        delete rt;
        context_ = nullptr;
        engine_ = nullptr;
        runtime_ = nullptr;
        return false;
    }

    cudaError_t err = cudaMalloc(&d_input_, static_cast<size_t>(input_vol) * sizeof(float));
    if (err != cudaSuccess)
    {
        std::cerr << "TrtEngine: cudaMalloc input failed: " << cudaGetErrorString(err) << std::endl;
        delete ctx;
        delete eng;
        delete rt;
        context_ = nullptr;
        engine_ = nullptr;
        runtime_ = nullptr;
        return false;
    }
    err = cudaMalloc(&d_output_, static_cast<size_t>(output_vol) * sizeof(float));
    if (err != cudaSuccess)
    {
        std::cerr << "TrtEngine: cudaMalloc output failed: " << cudaGetErrorString(err) << std::endl;
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

    if (!ctx->setTensorAddress(input_name, d_input_) || !ctx->setTensorAddress(output_name, d_output_))
    {
        std::cerr << "TrtEngine: setTensorAddress failed" << std::endl;
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
    return true;
}

std::vector<int64_t> TrtEngine::getInputShape() const
{
    return input_shape_;
}

std::vector<int64_t> TrtEngine::getOutputShape() const
{
    return output_shape_;
}

bool TrtEngine::execute(const float* host_input, float* host_output)
{
    if (!context_ || !d_input_ || !d_output_)
        return false;

    cudaError_t err = cudaMemcpy(d_input_, host_input, getInputSizeBytes(), cudaMemcpyHostToDevice);
    if (err != cudaSuccess)
    {
        std::cerr << "TrtEngine: cudaMemcpy H2D failed: " << cudaGetErrorString(err) << std::endl;
        return false;
    }

    void* bindings[2] = {d_input_, d_output_};
    auto* ctx = static_cast<nvinfer1::IExecutionContext*>(context_);
    if (!ctx->executeV2(bindings))
    {
        std::cerr << "TrtEngine: executeV2 failed" << std::endl;
        return false;
    }

    err = cudaMemcpy(host_output, d_output_, getOutputSizeBytes(), cudaMemcpyDeviceToHost);
    if (err != cudaSuccess)
    {
        std::cerr << "TrtEngine: cudaMemcpy D2H failed: " << cudaGetErrorString(err) << std::endl;
        return false;
    }

    return true;
}

} // namespace auto_battlebot
