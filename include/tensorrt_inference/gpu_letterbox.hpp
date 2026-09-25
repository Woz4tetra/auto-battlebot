#pragma once

#include <cstddef>
#include <opencv2/core.hpp>

namespace auto_battlebot {

/**
 * Letterboxes a camera frame straight into a TensorRT input buffer on the GPU, replacing the CPU
 * resize, pad, normalize and CHW pack.
 *
 * A frame in pinned host memory (PinnedMatAllocator) is read in place: the Orin's GPU and CPU share
 * DRAM, so the kernel fetches only the source pixels it samples. Anything else (a playback
 * frame, a camera without the pinned allocator) is copied to a device staging buffer first.
 */
class GpuLetterbox {
   public:
    GpuLetterbox() = default;
    ~GpuLetterbox();

    GpuLetterbox(const GpuLetterbox &) = delete;
    GpuLetterbox &operator=(const GpuLetterbox &) = delete;
    GpuLetterbox(GpuLetterbox &&) = delete;
    GpuLetterbox &operator=(GpuLetterbox &&) = delete;

    /**
     * Enqueues the letterbox of `image` (8-bit BGR or BGRA) into `tensor`, a device buffer of
     * 3 * tensor_height * tensor_width floats, on `stream` (a cudaStream_t). The work is
     * asynchronous: `image` must stay alive until the stream is synchronized. Returns false for
     * an unsupported image or a failed copy or launch.
     */
    bool enqueue(const cv::Mat &image, int tensor_width, int tensor_height, float padding,
                 float *tensor, void *stream);

    /** True if the last enqueue read the frame in place rather than copying it. */
    bool last_was_zero_copy() const { return last_was_zero_copy_; }

   private:
    void *staging_{nullptr};
    size_t staging_bytes_{0};
    bool last_was_zero_copy_{false};
};

}  // namespace auto_battlebot
