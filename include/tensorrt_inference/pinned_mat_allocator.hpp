#pragma once

#include <cstddef>
#include <map>
#include <mutex>
#include <opencv2/core.hpp>
#include <vector>

namespace auto_battlebot {

/**
 * cv::Mat allocator backed by pinned, device-mapped host memory, so GpuLetterbox reads a frame in
 * place instead of copying it to the GPU. Set it on the destination Mat before the conversion
 * that fills it:
 *
 *     cv::Mat frame;
 *     frame.allocator = PinnedMatAllocator::instance();
 *     cv::cvtColor(sdk_image, frame, cv::COLOR_BGRA2BGR);
 *
 * Pinning is slow (it locks pages), so released buffers go back to a pool keyed by size and the
 * next frame reuses one. On Orin, pinned memory stays CPU-cached and I/O coherent, so the CPU
 * writes into it at full speed.
 *
 * With no CUDA device, or when a pinned allocation fails, it falls back to OpenCV's standard
 * allocator and the frame takes the copy path.
 */
class PinnedMatAllocator : public cv::MatAllocator {
   public:
    /** Process-wide instance. Never destroyed: Mats can outlive static destruction order. */
    static PinnedMatAllocator *instance();

    cv::UMatData *allocate(int dims, const int *sizes, int type, void *data, size_t *step,
                           cv::AccessFlag flags, cv::UMatUsageFlags usage_flags) const override;
    bool allocate(cv::UMatData *data, cv::AccessFlag access_flags,
                  cv::UMatUsageFlags usage_flags) const override;
    void deallocate(cv::UMatData *data) const override;

    /** Buffers held for reuse across all sizes. */
    size_t pooled_buffers() const;

   private:
    PinnedMatAllocator() = default;

    /** Free buffers kept per size. A frame lives about ten ticks between the pipeline and the
     *  publisher queue, so this covers steady state without pinning memory forever. */
    static constexpr size_t kMaxPooledPerSize = 16;

    mutable std::mutex mutex_;
    mutable std::map<size_t, std::vector<void *>> pool_;
};

}  // namespace auto_battlebot
