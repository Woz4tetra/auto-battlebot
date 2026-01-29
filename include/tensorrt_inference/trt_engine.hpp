#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace auto_battlebot
{
// Shared TensorRT inference helper for loading a serialized .engine and running
// inference. Supports exactly one input and one output tensor (float32).
// Used by DeepLab and YOLO model implementations.
class TrtEngine
{
public:
    TrtEngine() = default;
    ~TrtEngine();

    TrtEngine(const TrtEngine&) = delete;
    TrtEngine& operator=(const TrtEngine&) = delete;
    TrtEngine(TrtEngine&&) = delete;
    TrtEngine& operator=(TrtEngine&&) = delete;

    // Load engine from a serialized .engine file. Returns true on success.
    bool load(const std::string& engine_path);

    // Return true if an engine is loaded and ready for inference.
    bool isLoaded() const { return context_ != nullptr; }

    // Shape of the input tensor (e.g. [1, 3, H, W] for NCHW).
    std::vector<int64_t> getInputShape() const;

    // Shape of the output tensor.
    std::vector<int64_t> getOutputShape() const;

    // Number of float elements for input/output.
    int64_t getInputNumElements() const { return input_num_elements_; }
    int64_t getOutputNumElements() const { return output_num_elements_; }

    // Size in bytes (input/output are float32).
    size_t getInputSizeBytes() const { return static_cast<size_t>(input_num_elements_) * sizeof(float); }
    size_t getOutputSizeBytes() const { return static_cast<size_t>(output_num_elements_) * sizeof(float); }

    // Run inference: copy host_input to GPU, execute, copy output to host_output.
    // Both buffers must be at least getInputSizeBytes() / getOutputSizeBytes().
    // Returns true on success.
    bool execute(const float* host_input, float* host_output);

private:
    void* d_input_{nullptr};
    void* d_output_{nullptr};
    void* context_{nullptr};
    void* engine_{nullptr};
    void* runtime_{nullptr};
    int64_t input_num_elements_{0};
    int64_t output_num_elements_{0};
    int32_t input_io_index_{-1};
    int32_t output_io_index_{-1};
    std::vector<int64_t> input_shape_;
    std::vector<int64_t> output_shape_;
};

} // namespace auto_battlebot
