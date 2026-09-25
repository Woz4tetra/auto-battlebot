#include "tensorrt_inference/pinned_mat_allocator.hpp"

#include <cuda_runtime.h>
#include <spdlog/spdlog.h>

namespace auto_battlebot {

PinnedMatAllocator *PinnedMatAllocator::instance() {
    static PinnedMatAllocator *allocator = new PinnedMatAllocator();
    return allocator;
}

cv::UMatData *PinnedMatAllocator::allocate(int dims, const int *sizes, int type, void *data,
                                           size_t *step, cv::AccessFlag flags,
                                           cv::UMatUsageFlags usage_flags) const {
    // Wrapping caller-owned memory: nothing to pin, so let the standard allocator track it.
    if (data) {
        return cv::Mat::getStdAllocator()->allocate(dims, sizes, type, data, step, flags,
                                                    usage_flags);
    }

    // Dense layout, the same as OpenCV's standard allocator.
    size_t total = CV_ELEM_SIZE(type);
    for (int i = dims - 1; i >= 0; --i) {
        if (step) step[i] = total;
        total *= static_cast<size_t>(sizes[i]);
    }

    void *buffer = nullptr;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = pool_.find(total);
        if (it != pool_.end() && !it->second.empty()) {
            buffer = it->second.back();
            it->second.pop_back();
        }
    }
    if (!buffer) {
        // Portable and mapped: readable from the device at a pointer cudaPointerGetAttributes
        // reports, on any context.
        const cudaError_t err =
            cudaHostAlloc(&buffer, total, cudaHostAllocPortable | cudaHostAllocMapped);
        if (err != cudaSuccess) {
            cudaGetLastError();
            static std::once_flag warned;
            std::call_once(warned, [err] {
                spdlog::warn("PinnedMatAllocator: cudaHostAlloc failed ({}); using pageable frames",
                             cudaGetErrorString(err));
            });
            return cv::Mat::getStdAllocator()->allocate(dims, sizes, type, nullptr, step, flags,
                                                        usage_flags);
        }
    }

    auto *u = new cv::UMatData(this);
    u->data = u->origdata = static_cast<uchar *>(buffer);
    u->size = total;
    return u;
}

bool PinnedMatAllocator::allocate(cv::UMatData *data, cv::AccessFlag /*access_flags*/,
                                  cv::UMatUsageFlags /*usage_flags*/) const {
    return data != nullptr;
}

void PinnedMatAllocator::deallocate(cv::UMatData *data) const {
    if (!data) return;
    CV_Assert(data->urefcount == 0 && data->refcount == 0);
    void *buffer = data->origdata;
    const size_t size = data->size;
    delete data;
    if (!buffer) return;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto &free_list = pool_[size];
        if (free_list.size() < kMaxPooledPerSize) {
            free_list.push_back(buffer);
            return;
        }
    }
    cudaFreeHost(buffer);
}

size_t PinnedMatAllocator::pooled_buffers() const {
    std::lock_guard<std::mutex> lock(mutex_);
    size_t count = 0;
    for (const auto &[size, free_list] : pool_) count += free_list.size();
    return count;
}

}  // namespace auto_battlebot
