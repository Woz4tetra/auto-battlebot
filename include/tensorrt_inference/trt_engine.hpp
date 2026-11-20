#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include "tensorrt_inference/cuda_handles.hpp"

namespace auto_battlebot {
class TrtEngine {
   public:
    struct OutputTensorInfo {
        std::string name;
        std::vector<int64_t> shape;
        int32_t io_index = -1;
        int64_t num_elements = 0;
    };

    class ScopedQuietLogging {
       public:
        ScopedQuietLogging();
        ~ScopedQuietLogging();

        ScopedQuietLogging(const ScopedQuietLogging&) = delete;
        ScopedQuietLogging& operator=(const ScopedQuietLogging&) = delete;
        ScopedQuietLogging(ScopedQuietLogging&&) = delete;
        ScopedQuietLogging& operator=(ScopedQuietLogging&&) = delete;

        static bool active_on_this_thread();
    };

    TrtEngine() = default;
    ~TrtEngine();

    TrtEngine(const TrtEngine&) = delete;
    TrtEngine& operator=(const TrtEngine&) = delete;
    TrtEngine(TrtEngine&&) = delete;
    TrtEngine& operator=(TrtEngine&&) = delete;

    bool load(const std::string& engine_path);

    bool isLoaded() const { return context_ != nullptr; }

    std::vector<int64_t> getInputShape() const;

    std::vector<int64_t> getOutputShape() const;

    int64_t getInputNumElements() const { return input_num_elements_; }
    int64_t getOutputNumElements() const { return output_num_elements_; }
    const std::vector<OutputTensorInfo>& getOutputTensorInfos() const { return output_infos_; }

    size_t getInputSizeBytes() const {
        return static_cast<size_t>(input_num_elements_) * sizeof(float);
    }
    size_t getOutputSizeBytes() const {
        return static_cast<size_t>(output_num_elements_) * sizeof(float);
    }

    bool execute(std::span<const float> host_input, std::span<float> host_output);

    bool execute_multi(std::span<const float> host_input,
                       std::span<const std::span<float>> host_outputs);

    std::span<const float> execute_borrowing_pinned_output_after_device_input();

    float* device_input() const { return static_cast<float*>(d_input_); }
    cudaStream_t stream() const { return stream_; }

    bool uses_cuda_graph() const { return graph_exec_ != nullptr; }

   private:
    bool upload_input(const float* host_input);
    bool enqueue_direct();
    bool enqueue_network();
    bool run_and_wait(std::chrono::steady_clock::time_point exec_start, const char* caller);
    void capture_graph();

    cudaGraphExec_t graph_exec_{nullptr};
    bool graph_capture_attempted_{false};
    cudaStream_t stream_{nullptr};
    void* h_input_pinned_{nullptr};
    std::vector<void*> h_outputs_pinned_;
    void* d_input_{nullptr};
    void* d_output_{nullptr};
    std::vector<void*> d_outputs_;
    void* context_{nullptr};
    void* engine_{nullptr};
    void* runtime_{nullptr};
    int64_t input_num_elements_{0};
    int64_t output_num_elements_{0};
    int32_t input_io_index_{-1};
    int32_t output_io_index_{-1};
    std::vector<int64_t> input_shape_;
    std::vector<int64_t> output_shape_;
    std::vector<OutputTensorInfo> output_infos_;
};

}  // namespace auto_battlebot
