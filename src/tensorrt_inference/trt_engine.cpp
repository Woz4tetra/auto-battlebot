#include "tensorrt_inference/trt_engine.hpp"

#include <NvInferRuntime.h>
#include <NvInferRuntimeBase.h>
#include <NvInferVersion.h>
#include <cuda_runtime.h>
#include <spdlog/spdlog.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <fstream>
#include <new>

// IStreamReaderV2 arrived in TensorRT 10.7 and deprecated IStreamReader. The dev machines
// are on 10.14; the Jetson's JetPack TensorRT predates 10.7 and only has the older
// interface. Pick whichever the installed headers provide.
#define AB_TRT_HAS_STREAM_READER_V2 \
    (NV_TENSORRT_MAJOR > 10 || (NV_TENSORRT_MAJOR == 10 && NV_TENSORRT_MINOR >= 7))

namespace auto_battlebot {
namespace {
constexpr double kExecuteWarnMs = 120.0;

// Non-zero while a TrtEngine::ScopedQuietLogging guard is alive. Atomic because models
// are constructed from the same thread today but nothing enforces that.
std::atomic<int> g_quiet_depth{0};

// Minimal TensorRT logger (only log errors/warnings via spdlog).
class TrtLogger : public nvinfer1::ILogger {
   public:
    void log(Severity severity, nvinfer1::AsciiChar const* msg) noexcept override {
        if (severity > Severity::kWARNING) return;
        if (g_quiet_depth.load(std::memory_order_relaxed) > 0) {
            // Probing a candidate engine. A rejection here is an expected outcome, not a
            // fault, so keep it out of the error stream and the mcap /rosout topic.
            spdlog::debug("[TensorRT] {}", msg);
            return;
        }
        spdlog::error("[TensorRT] {}", msg);
    }
};

TrtLogger g_trt_logger;

// A failed load is an expected outcome while EngineSelector probes candidates, so route
// TrtEngine's own load diagnostics through the same switch as TensorRT's. Outside
// probing these stay errors.
void log_load_failure(const std::string& message) {
    if (g_quiet_depth.load(std::memory_order_relaxed) > 0) {
        spdlog::debug("{}", message);
        return;
    }
    spdlog::error("{}", message);
}

#if AB_TRT_HAS_STREAM_READER_V2
using TrtStreamReaderBase = nvinfer1::IStreamReaderV2;
#else
using TrtStreamReaderBase = nvinfer1::IStreamReader;
#endif

// Feeds a file to TensorRT incrementally so deserialization can abort early.
//
// The V2 interface may hand us a device pointer as the destination, so every read is
// staged through a host buffer and routed with cudaMemcpyDefault. Unified addressing
// makes that correct whether the target is host or device memory. V1 destinations are
// always host memory, and V1 has no seek.
class FileStreamReader final : public TrtStreamReaderBase {
   public:
    explicit FileStreamReader(const std::string& path) : file_(path, std::ios::binary) {}

    bool is_open() const { return file_.is_open(); }

#if AB_TRT_HAS_STREAM_READER_V2
    int64_t read(void* destination, int64_t nbBytes, cudaStream_t stream) noexcept override {
        if (nbBytes <= 0) return 0;
        try {
            staging_.resize(static_cast<size_t>(nbBytes));
        } catch (const std::bad_alloc&) {
            return -1;
        }
        file_.read(staging_.data(), static_cast<std::streamsize>(nbBytes));
        const int64_t bytes_read = static_cast<int64_t>(file_.gcount());
        if (bytes_read <= 0) return bytes_read;

        if (cudaMemcpyAsync(destination, staging_.data(), static_cast<size_t>(bytes_read),
                            cudaMemcpyDefault, stream) != cudaSuccess) {
            return -1;
        }
        // staging_ is reused by the next call, so the copy has to land before returning.
        if (cudaStreamSynchronize(stream) != cudaSuccess) return -1;
        return bytes_read;
    }

    bool seek(int64_t offset, nvinfer1::SeekPosition where) noexcept override {
        std::ios_base::seekdir direction = std::ios_base::beg;
        switch (where) {
            case nvinfer1::SeekPosition::kSET:
                direction = std::ios_base::beg;
                break;
            case nvinfer1::SeekPosition::kCUR:
                direction = std::ios_base::cur;
                break;
            case nvinfer1::SeekPosition::kEND:
                direction = std::ios_base::end;
                break;
        }
        // Clear first: a prior read that hit EOF leaves failbit set, which would make
        // seekg a no-op.
        file_.clear();
        file_.seekg(static_cast<std::streamoff>(offset), direction);
        return file_.good();
    }
#else
    int64_t read(void* destination, int64_t nbBytes) noexcept override {
        if (nbBytes <= 0) return 0;
        file_.read(static_cast<char*>(destination), static_cast<std::streamsize>(nbBytes));
        return static_cast<int64_t>(file_.gcount());
    }
#endif

   private:
    std::ifstream file_;
#if AB_TRT_HAS_STREAM_READER_V2
    std::vector<char> staging_;
#endif
};

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
    // execute() synchronizes the stream before returning, so no work is in flight here.
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

TrtEngine::ScopedQuietLogging::ScopedQuietLogging() {
    g_quiet_depth.fetch_add(1, std::memory_order_relaxed);
}

TrtEngine::ScopedQuietLogging::~ScopedQuietLogging() {
    g_quiet_depth.fetch_sub(1, std::memory_order_relaxed);
}

bool TrtEngine::load(const std::string& engine_path) {
    FileStreamReader reader(engine_path);
    if (!reader.is_open()) {
        log_load_failure(fmt::format("TrtEngine: cannot open file: {}", engine_path));
        return false;
    }

    nvinfer1::IRuntime* rt = nvinfer1::createInferRuntime(g_trt_logger);
    if (!rt) {
        log_load_failure("TrtEngine: createInferRuntime failed");
        return false;
    }
    runtime_ = rt;

    // Allow version-compatible engines (built with VERSION_COMPATIBLE) to load.
    rt->setEngineHostCodeAllowed(true);

    // Streaming rather than whole-file: an engine built for another GPU architecture or
    // TensorRT version is rejected once the header has been read, so EngineSelector can
    // try a candidate list without paying a full read per miss.
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

    // Final step: per-engine non-blocking stream plus pinned staging buffers. Created
    // last so every earlier error path stays untouched; on failure the destructor-style
    // cleanup below releases everything allocated so far.
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

bool TrtEngine::execute(const float* host_input, float* host_output) {
    if (!context_ || !d_input_ || !d_output_ || !stream_) return false;

    const auto exec_start = std::chrono::steady_clock::now();
    auto stream = static_cast<cudaStream_t>(stream_);
    auto* ctx = static_cast<nvinfer1::IExecutionContext*>(context_);

    // Stage through pinned memory so both copies are async on this engine's stream; the
    // single synchronize at the end is the only blocking point.
    std::memcpy(h_input_pinned_, host_input, getInputSizeBytes());
    cudaError_t err = cudaMemcpyAsync(d_input_, h_input_pinned_, getInputSizeBytes(),
                                      cudaMemcpyHostToDevice, stream);
    if (err != cudaSuccess) {
        spdlog::error("TrtEngine: cudaMemcpyAsync H2D failed: {}", cudaGetErrorString(err));
        return false;
    }
    if (!ctx->enqueueV3(stream)) {
        spdlog::error("TrtEngine: enqueueV3 failed");
        return false;
    }
    err = cudaMemcpyAsync(h_outputs_pinned_[0], d_output_, getOutputSizeBytes(),
                          cudaMemcpyDeviceToHost, stream);
    if (err != cudaSuccess) {
        spdlog::error("TrtEngine: cudaMemcpyAsync D2H failed: {}", cudaGetErrorString(err));
        return false;
    }

    const auto sync_start = std::chrono::steady_clock::now();
    err = cudaStreamSynchronize(stream);
    if (err != cudaSuccess) {
        spdlog::error("TrtEngine: cudaStreamSynchronize failed: {}", cudaGetErrorString(err));
        return false;
    }
    const double sync_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - sync_start)
            .count();
    std::memcpy(host_output, h_outputs_pinned_[0], getOutputSizeBytes());

    const double total_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - exec_start)
            .count();
    if (total_ms > kExecuteWarnMs) {
        spdlog::warn("TrtEngine::execute slow path total_ms={:.2f} sync_ms={:.2f}", total_ms,
                     sync_ms);
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

    if (!stream_) return false;

    const auto exec_start = std::chrono::steady_clock::now();
    auto stream = static_cast<cudaStream_t>(stream_);
    auto* ctx = static_cast<nvinfer1::IExecutionContext*>(context_);

    // Stage through pinned memory so all copies are async on this engine's stream; the
    // single synchronize below is the only blocking point.
    std::memcpy(h_input_pinned_, host_input, getInputSizeBytes());
    cudaError_t err = cudaMemcpyAsync(d_input_, h_input_pinned_, getInputSizeBytes(),
                                      cudaMemcpyHostToDevice, stream);
    if (err != cudaSuccess) {
        spdlog::error("TrtEngine: cudaMemcpyAsync H2D failed: {}", cudaGetErrorString(err));
        return false;
    }
    if (!ctx->enqueueV3(stream)) {
        spdlog::error("TrtEngine: execute_multi enqueueV3 failed");
        return false;
    }
    for (size_t i = 0; i < d_outputs_.size(); ++i) {
        const size_t bytes = static_cast<size_t>(output_infos_[i].num_elements) * sizeof(float);
        err = cudaMemcpyAsync(h_outputs_pinned_[i], d_outputs_[i], bytes, cudaMemcpyDeviceToHost,
                              stream);
        if (err != cudaSuccess) {
            spdlog::error("TrtEngine: cudaMemcpyAsync D2H failed for output {}: {}",
                          output_infos_[i].name, cudaGetErrorString(err));
            return false;
        }
    }

    const auto sync_start = std::chrono::steady_clock::now();
    err = cudaStreamSynchronize(stream);
    if (err != cudaSuccess) {
        spdlog::error("TrtEngine: cudaStreamSynchronize failed: {}", cudaGetErrorString(err));
        return false;
    }
    const double sync_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - sync_start)
            .count();
    for (size_t i = 0; i < d_outputs_.size(); ++i) {
        const size_t bytes = static_cast<size_t>(output_infos_[i].num_elements) * sizeof(float);
        std::memcpy(host_outputs[i], h_outputs_pinned_[i], bytes);
    }

    const double total_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - exec_start)
            .count();
    if (total_ms > kExecuteWarnMs) {
        spdlog::warn("TrtEngine::execute_multi slow path total_ms={:.2f} sync_ms={:.2f} outputs={}",
                     total_ms, sync_ms, d_outputs_.size());
    }
    return true;
}

}  // namespace auto_battlebot
