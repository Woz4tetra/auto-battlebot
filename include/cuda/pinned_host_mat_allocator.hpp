#pragma once

#include <cstddef>
#include <mutex>
#include <opencv2/core/mat.hpp>
#include <vector>

namespace auto_battlebot {

/**
 * @brief OpenCV cv::MatAllocator backed by cudaHostAlloc(cudaHostAllocMapped |
 *        cudaHostAllocPortable).
 *
 * Why pinned + mapped:
 *   - cudaMemcpy(H2D) from page-locked memory bypasses the runtime's per-call pinning step,
 *     runs at full bus bandwidth, and unlocks true async cudaMemcpyAsync.
 *   - On Tegra (Orin), GPU and CPU share LPDDR5; mapped pinned memory is device-accessible at
 *     the same physical address (cudaHostGetDevicePointer), so the H2D becomes a pointer alias.
 *
 * Buffers are pooled internally to amortize the ~1 ms cost of each cudaHostAlloc across frames.
 * Lifetime of each buffer is tied to OpenCV's UMatData refcount as usual; a freed buffer is
 * returned to the pool and reused on the next allocate() of compatible size.
 *
 * If cudaHostAlloc fails or the pool is at capacity, allocate() falls back to cv::fastMalloc
 * (pageable host memory) so the camera path keeps working without CUDA.
 *
 * Thread-safe.
 *
 * Usage:
 *   PinnedHostMatAllocator allocator;
 *   cv::Mat dst;
 *   dst.allocator = &allocator;
 *   cv::cvtColor(src, dst, cv::COLOR_BGRA2BGR);   // dst now backed by pinned memory.
 */
class PinnedHostMatAllocator : public cv::MatAllocator {
   public:
    /**
     * @param max_pool_entries Cap on retained pool buffers. New allocate() calls beyond this
     *                         fall back to pageable memory with a warning. 0 = unbounded.
     */
    explicit PinnedHostMatAllocator(std::size_t max_pool_entries = 8);
    ~PinnedHostMatAllocator() override;

    PinnedHostMatAllocator(const PinnedHostMatAllocator&) = delete;
    PinnedHostMatAllocator& operator=(const PinnedHostMatAllocator&) = delete;

    cv::UMatData* allocate(int dims, const int* sizes, int type, void* data, size_t* step,
                           cv::AccessFlag flags, cv::UMatUsageFlags usageFlags) const override;
    bool allocate(cv::UMatData* data, cv::AccessFlag accessFlags,
                  cv::UMatUsageFlags usageFlags) const override;
    void deallocate(cv::UMatData* data) const override;

   private:
    struct Slot {
        void* ptr = nullptr;
        std::size_t bytes = 0;
        bool in_use = false;
    };

    std::size_t max_pool_entries_;
    mutable std::mutex mu_;
    mutable std::vector<Slot> pool_;
};

}  // namespace auto_battlebot
