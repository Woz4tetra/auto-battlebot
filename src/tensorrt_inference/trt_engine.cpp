#include "tensorrt_inference/trt_engine.hpp"

#include <NvInferRuntime.h>
#include <NvInferRuntimeBase.h>
#include <cuda_runtime.h>
#include <spdlog/spdlog.h>

#include <array>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <new>

#include "tensorrt_inference/file_stream_reader.hpp"

namespace auto_battlebot {
namespace {
constexpr double kExecuteWarnMs = 120.0;

thread_local int t_quiet_depth = 0;

class TrtLogger : public nvinfer1::ILogger {
   public:
    void log(Severity severity, nvinfer1::AsciiChar const* msg) noexcept override {
        if (severity > Severity::kWARNING) return;
        if (t_quiet_depth > 0) {
            spdlog::debug("[TensorRT] {}", msg);
            return;
        }
        spdlog::error("[TensorRT] {}", msg);
    }
};

TrtLogger g_trt_logger;

void log_load_failure(const std::string& message) {
    if (t_quiet_depth > 0) {
        spdlog::debug("{}", message);
        return;
    }
    spdlog::error("{}", message);
}

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
    if (graph_exec_) {
        cudaGraphExecDestroy(static_cast<cudaGraphExec_t>(graph_exec_));
        graph_exec_ = nullptr;
    }
    if (stream_) {
        cudaStreamDestroy(static_cast<cudaStream_t>(stream_));
        stream_ = nullptr;
    }
    if (h_input_pinned_) {
        cudaFreeHost(h_input_pinned_);
        h_input_pinned_ = nullptr;
    }
    for (void* ptr : h_outputs_pinned_) {
        if (ptr) cudaFreeHost(ptr);
    }
    h_outputs_pinned_.clear();
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

bool TrtEngine::ScopedQuietLogging::active_on_this_thread() { return t_quiet_depth > 0; }

TrtEngine::ScopedQuietLogging::ScopedQuietLogging() { ++t_quiet_depth; }

TrtEngine::ScopedQuietLogging::~ScopedQuietLogging() { --t_quiet_depth; }

constexpr std::array<char, 4> kEngineMagic{'f', 't', 'r', 't'};
constexpr std::streamoff kEngineSizeOffset = 16;

bool engine_header_matches_file(const std::string& engine_path) {
    std::ifstream file(engine_path, std::ios::binary);
    std::array<char, 4> magic{};
    uint64_t declared_size = 0;
    if (!file.read(magic.data(), magic.size()) || magic != kEngineMagic) return false;
    if (!file.seekg(kEngineSizeOffset) ||
        !file.read(reinterpret_cast<char*>(&declared_size), sizeof(declared_size))) {
        return false;
    }
    std::error_code error;
    const auto actual_size = std::filesystem::file_size(engine_path, error);
    return !error && declared_size == actual_size;
}

bool TrtEngine::load(const std::string& engine_path) {
    FileStreamReader reader(engine_path);
    if (!reader.is_open()) {
        log_load_failure(fmt::format("TrtEngine: cannot open file: {}", engine_path));
        return false;
    }
    if (!engine_header_matches_file(engine_path)) {
        log_load_failure(fmt::format(
            "TrtEngine: {} is not a whole TensorRT engine (bad magic or truncated)", engine_path));
        return false;
    }

    nvinfer1::IRuntime* rt = nvinfer1::createInferRuntime(g_trt_logger);
    if (!rt) {
        log_load_failure("TrtEngine: createInferRuntime failed");
        return false;
    }
    runtime_ = rt;

    rt->setEngineHostCodeAllowed(true);

    nvinfer1::ICudaEngine* eng = rt->deserializeCudaEngine(reader);
    if (!eng) {
        log_load_failure("TrtEngine: deserializeCudaEngine failed.");
        delete rt;
        runtime_ = nullptr;
        engine_ = nullptr;
        return false;
    }
    engine_ = eng;

    nvinfer1::IExecutionContext* ctx = eng->createExecutionContext();
    if (!ctx) {
        log_load_failure("TrtEngine: createExecutionContext failed");
        delete eng;
        delete rt;
        engine_ = nullptr;
        runtime_ = nullptr;
        return false;
    }
    context_ = ctx;

    const int32_t nb_io = eng->getNbIOTensors();
    if (nb_io < 2) {
        log_load_failure(fmt::format("TrtEngine: expected at least 2 IO tensors, got {}", nb_io));
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
        log_load_failure("TrtEngine: could not identify input/output tensors by mode");
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

    auto cleanup_all = [this]() {
        if (stream_) {
            cudaStreamDestroy(static_cast<cudaStream_t>(stream_));
            stream_ = nullptr;
        }
        if (h_input_pinned_) {
            cudaFreeHost(h_input_pinned_);
            h_input_pinned_ = nullptr;
        }
        for (void* ptr : h_outputs_pinned_) {
            if (ptr) cudaFreeHost(ptr);
        }
        h_outputs_pinned_.clear();
        cudaFree(d_input_);
        d_input_ = nullptr;
        for (void* ptr : d_outputs_) {
            if (ptr) cudaFree(ptr);
        }
        d_outputs_.clear();
        d_output_ = nullptr;
        delete static_cast<nvinfer1::IExecutionContext*>(context_);
        delete static_cast<nvinfer1::ICudaEngine*>(engine_);
        delete static_cast<nvinfer1::IRuntime*>(runtime_);
        context_ = nullptr;
        engine_ = nullptr;
        runtime_ = nullptr;
    };

    cudaStream_t stream = nullptr;
    err = cudaStreamCreateWithFlags(&stream, cudaStreamNonBlocking);
    if (err != cudaSuccess) {
        spdlog::error("TrtEngine: cudaStreamCreateWithFlags failed: {}", cudaGetErrorString(err));
        cleanup_all();
        return false;
    }
    stream_ = stream;

    err = cudaMallocHost(&h_input_pinned_, getInputSizeBytes());
    if (err != cudaSuccess) {
        spdlog::error("TrtEngine: cudaMallocHost input failed: {}", cudaGetErrorString(err));
        cleanup_all();
        return false;
    }
    for (const OutputTensorInfo& info : output_infos_) {
        void* pinned = nullptr;
        err = cudaMallocHost(&pinned, static_cast<size_t>(info.num_elements) * sizeof(float));
        if (err != cudaSuccess) {
            spdlog::error("TrtEngine: cudaMallocHost output '{}' failed: {}", info.name,
                          cudaGetErrorString(err));
            cleanup_all();
            return false;
        }
        h_outputs_pinned_.push_back(pinned);
    }

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

bool TrtEngine::enqueue_direct() {
    auto stream = static_cast<cudaStream_t>(stream_);
    auto* ctx = static_cast<nvinfer1::IExecutionContext*>(context_);
    if (!ctx->enqueueV3(stream)) {
        spdlog::error("TrtEngine: enqueueV3 failed");
        return false;
    }
    for (size_t i = 0; i < d_outputs_.size(); ++i) {
        const size_t bytes = static_cast<size_t>(output_infos_[i].num_elements) * sizeof(float);
        const cudaError_t err = cudaMemcpyAsync(h_outputs_pinned_[i], d_outputs_[i], bytes,
                                                cudaMemcpyDeviceToHost, stream);
        if (err != cudaSuccess) {
            spdlog::error("TrtEngine: cudaMemcpyAsync D2H failed for output {}: {}",
                          output_infos_[i].name, cudaGetErrorString(err));
            return false;
        }
    }
    return true;
}

bool TrtEngine::enqueue_network() {
    if (!graph_exec_) return enqueue_direct();
    const cudaError_t err = cudaGraphLaunch(static_cast<cudaGraphExec_t>(graph_exec_),
                                            static_cast<cudaStream_t>(stream_));
    if (err != cudaSuccess) {
        spdlog::error("TrtEngine: cudaGraphLaunch failed: {}", cudaGetErrorString(err));
        return false;
    }
    return true;
}

void TrtEngine::capture_graph() {
    graph_capture_attempted_ = true;
    auto stream = static_cast<cudaStream_t>(stream_);
    cudaError_t err = cudaStreamBeginCapture(stream, cudaStreamCaptureModeThreadLocal);
    if (err != cudaSuccess) {
        spdlog::warn("TrtEngine: cudaStreamBeginCapture failed ({}); running without a CUDA graph",
                     cudaGetErrorString(err));
        cudaGetLastError();
        return;
    }
    const bool enqueued = enqueue_direct();
    cudaGraph_t graph = nullptr;
    err = cudaStreamEndCapture(stream, &graph);
    if (!enqueued || err != cudaSuccess || graph == nullptr) {
        if (graph) cudaGraphDestroy(graph);
        spdlog::warn("TrtEngine: CUDA graph capture failed ({}); running without a CUDA graph",
                     cudaGetErrorString(err));
        cudaGetLastError();
        return;
    }
    cudaGraphExec_t exec = nullptr;
    err = cudaGraphInstantiate(&exec, graph, 0);
    cudaGraphDestroy(graph);
    if (err != cudaSuccess) {
        spdlog::warn("TrtEngine: cudaGraphInstantiate failed ({}); running without a CUDA graph",
                     cudaGetErrorString(err));
        cudaGetLastError();
        return;
    }
    graph_exec_ = exec;
    spdlog::info("TrtEngine: inference replays from a CUDA graph");
}

bool TrtEngine::run_and_wait(std::chrono::steady_clock::time_point exec_start, const char* caller) {
    if (!enqueue_network()) return false;
    const auto sync_start = std::chrono::steady_clock::now();
    const cudaError_t err = cudaStreamSynchronize(static_cast<cudaStream_t>(stream_));
    if (err != cudaSuccess) {
        spdlog::error("TrtEngine: cudaStreamSynchronize failed: {}", cudaGetErrorString(err));
        return false;
    }
    const auto end = std::chrono::steady_clock::now();
    const double sync_ms = std::chrono::duration<double, std::milli>(end - sync_start).count();
    const double total_ms = std::chrono::duration<double, std::milli>(end - exec_start).count();
    if (total_ms > kExecuteWarnMs) {
        spdlog::warn("TrtEngine::{} slow path total_ms={:.2f} sync_ms={:.2f}", caller, total_ms,
                     sync_ms);
    }
    if (!graph_capture_attempted_) capture_graph();
    return true;
}

bool TrtEngine::upload_input(const float* host_input) {
    std::memcpy(h_input_pinned_, host_input, getInputSizeBytes());
    const cudaError_t err =
        cudaMemcpyAsync(d_input_, h_input_pinned_, getInputSizeBytes(), cudaMemcpyHostToDevice,
                        static_cast<cudaStream_t>(stream_));
    if (err != cudaSuccess) {
        spdlog::error("TrtEngine: cudaMemcpyAsync H2D failed: {}", cudaGetErrorString(err));
        return false;
    }
    return true;
}

bool TrtEngine::execute(std::span<const float> host_input, std::span<float> host_output) {
    if (!context_ || !d_input_ || !d_output_ || !stream_) return false;
    if (host_input.size() < static_cast<size_t>(input_num_elements_) ||
        host_output.size() < static_cast<size_t>(output_num_elements_)) {
        spdlog::error("TrtEngine: execute buffers too small input={}/{} output={}/{}",
                      host_input.size(), input_num_elements_, host_output.size(),
                      output_num_elements_);
        return false;
    }
    const auto exec_start = std::chrono::steady_clock::now();
    if (!upload_input(host_input.data())) return false;
    if (!run_and_wait(exec_start, "execute")) return false;
    std::memcpy(host_output.data(), h_outputs_pinned_[0], getOutputSizeBytes());
    return true;
}

bool TrtEngine::execute_multi(std::span<const float> host_input,
                              std::span<const std::span<float>> host_outputs) {
    if (!context_ || !d_input_ || d_outputs_.empty() || !stream_) return false;
    if (host_input.size() < static_cast<size_t>(input_num_elements_)) {
        spdlog::error("TrtEngine: execute_multi input too small host={} engine={}",
                      host_input.size(), input_num_elements_);
        return false;
    }
    for (size_t i = 0; i < host_outputs.size() && i < output_infos_.size(); ++i) {
        if (host_outputs[i].size() < static_cast<size_t>(output_infos_[i].num_elements)) {
            spdlog::error("TrtEngine: execute_multi output {} too small host={} engine={}", i,
                          host_outputs[i].size(), output_infos_[i].num_elements);
            return false;
        }
    }
    if (host_outputs.size() != d_outputs_.size()) {
        spdlog::error("TrtEngine: execute_multi output count mismatch host={} engine={}",
                      host_outputs.size(), d_outputs_.size());
        return false;
    }
    const auto exec_start = std::chrono::steady_clock::now();
    if (!upload_input(host_input.data())) return false;
    if (!run_and_wait(exec_start, "execute_multi")) return false;
    for (size_t i = 0; i < d_outputs_.size(); ++i) {
        const size_t bytes = static_cast<size_t>(output_infos_[i].num_elements) * sizeof(float);
        std::memcpy(host_outputs[i].data(), h_outputs_pinned_[i], bytes);
    }
    return true;
}

std::span<const float> TrtEngine::execute_borrowing_pinned_output_after_device_input() {
    if (!context_ || !d_input_ || !d_output_ || !stream_) return {};
    const auto exec_start = std::chrono::steady_clock::now();
    if (!run_and_wait(exec_start, "execute_device_input")) return {};
    return {static_cast<const float*>(h_outputs_pinned_[0]),
            static_cast<size_t>(output_num_elements_)};
}

}  // namespace auto_battlebot
