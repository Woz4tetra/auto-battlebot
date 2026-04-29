#include "cuda/pinned_host_mat_allocator.hpp"

#include <cuda_runtime.h>
#include <opencv2/core/core_c.h>  // CV_AUTOSTEP
#include <spdlog/spdlog.h>

namespace auto_battlebot {

PinnedHostMatAllocator::PinnedHostMatAllocator(std::size_t max_pool_entries)
    : max_pool_entries_(max_pool_entries) {}

PinnedHostMatAllocator::~PinnedHostMatAllocator() {
    std::lock_guard<std::mutex> lock(mu_);
    for (auto& slot : pool_) {
        if (!slot.ptr) continue;
        if (slot.in_use) {
            // A cv::Mat header outlived the allocator. Free anyway to avoid a leak; the dangling
            // header will read freed memory if it is ever touched again, but that bug is on the
            // caller (allocator must outlive every cv::Mat it backs).
            spdlog::warn(
                "PinnedHostMatAllocator: destroying with {} bytes still in use; outlived a "
                "cv::Mat header.",
                slot.bytes);
        }
        cudaError_t err = cudaFreeHost(slot.ptr);
        if (err != cudaSuccess) {
            spdlog::warn("PinnedHostMatAllocator: cudaFreeHost failed: {}",
                         cudaGetErrorString(err));
        }
    }
}

cv::UMatData* PinnedHostMatAllocator::allocate(int dims, const int* sizes, int type, void* data,
                                               size_t* step, cv::AccessFlag /*flags*/,
                                               cv::UMatUsageFlags /*usageFlags*/) const {
    // Compute total byte count. Mirrors cv::StdMatAllocator::allocate so step/dims semantics
    // match what the rest of OpenCV expects.
    size_t total = CV_ELEM_SIZE(type);
    for (int i = dims - 1; i >= 0; --i) {
        if (step) {
            if (data && step[i] != CV_AUTOSTEP) {
                CV_Assert(total <= step[i]);
                total = step[i];
            } else {
                step[i] = total;
            }
        }
        total *= static_cast<size_t>(sizes[i]);
    }

    uchar* data_ptr = static_cast<uchar*>(data);

    if (!data_ptr) {
        std::lock_guard<std::mutex> lock(mu_);

        // Best-fit on bytes among free slots, to minimize wasted capacity when shapes vary.
        std::size_t best_idx = pool_.size();
        for (std::size_t i = 0; i < pool_.size(); ++i) {
            if (pool_[i].in_use) continue;
            if (pool_[i].bytes < total) continue;
            if (best_idx == pool_.size() || pool_[i].bytes < pool_[best_idx].bytes) {
                best_idx = i;
            }
        }

        if (best_idx < pool_.size()) {
            data_ptr = static_cast<uchar*>(pool_[best_idx].ptr);
            pool_[best_idx].in_use = true;
        } else if (max_pool_entries_ == 0 || pool_.size() < max_pool_entries_) {
            void* new_ptr = nullptr;
            cudaError_t err =
                cudaHostAlloc(&new_ptr, total, cudaHostAllocMapped | cudaHostAllocPortable);
            if (err == cudaSuccess) {
                pool_.push_back(Slot{new_ptr, total, true});
                data_ptr = static_cast<uchar*>(new_ptr);
            } else {
                spdlog::warn(
                    "PinnedHostMatAllocator: cudaHostAlloc({} bytes) failed: {}; falling back to "
                    "pageable heap.",
                    total, cudaGetErrorString(err));
            }
        } else {
            spdlog::warn(
                "PinnedHostMatAllocator: pool exhausted ({} entries, all in use); falling back "
                "to pageable heap for {} bytes.",
                pool_.size(), total);
        }

        if (!data_ptr) {
            data_ptr = static_cast<uchar*>(cv::fastMalloc(total));
        }
    }

    cv::UMatData* u = new cv::UMatData(this);
    u->data = u->origdata = data_ptr;
    u->size = total;
    if (data) {
        u->flags = static_cast<cv::UMatData::MemoryFlag>(u->flags | cv::UMatData::USER_ALLOCATED);
    }
    return u;
}

bool PinnedHostMatAllocator::allocate(cv::UMatData* data, cv::AccessFlag /*accessFlags*/,
                                      cv::UMatUsageFlags /*usageFlags*/) const {
    return data != nullptr;
}

void PinnedHostMatAllocator::deallocate(cv::UMatData* u) const {
    if (!u) return;
    CV_Assert(u->urefcount == 0);
    CV_Assert(u->refcount == 0);

    if (!(u->flags & cv::UMatData::USER_ALLOCATED) && u->origdata) {
        bool returned_to_pool = false;
        {
            std::lock_guard<std::mutex> lock(mu_);
            for (auto& slot : pool_) {
                if (slot.ptr == u->origdata) {
                    slot.in_use = false;
                    returned_to_pool = true;
                    break;
                }
            }
        }
        if (!returned_to_pool) {
            // Pageable-fallback path from allocate().
            cv::fastFree(u->origdata);
        }
        u->origdata = nullptr;
    }

    delete u;
}

}  // namespace auto_battlebot
