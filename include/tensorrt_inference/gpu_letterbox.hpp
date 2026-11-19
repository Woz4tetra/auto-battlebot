#pragma once

#include <cstddef>
#include <opencv2/core.hpp>

#include "tensorrt_inference/cuda_handles.hpp"

namespace auto_battlebot {

class GpuLetterbox {
   public:
    GpuLetterbox() = default;
    ~GpuLetterbox();

    GpuLetterbox(const GpuLetterbox &) = delete;
    GpuLetterbox &operator=(const GpuLetterbox &) = delete;
    GpuLetterbox(GpuLetterbox &&) = delete;
    GpuLetterbox &operator=(GpuLetterbox &&) = delete;

    bool enqueue(const cv::Mat &image, DeviceTensorView tensor, float padding, cudaStream_t stream);

    bool last_was_zero_copy() const { return last_was_zero_copy_; }

   private:
    void *staging_{nullptr};
    size_t staging_bytes_{0};
    bool last_was_zero_copy_{false};
};

}  // namespace auto_battlebot
