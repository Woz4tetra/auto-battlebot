#pragma once

#include <atomic>
#include <chrono>
#include <opencv2/core.hpp>
#include <thread>

namespace auto_battlebot {

/** cv::Mat allocator whose free takes `delay`, like cudaFreeHost on a pinned frame. `freeing` goes
 *  true when a free starts, so a test can act while one is in progress. Must outlive its Mats. */
class SlowFreeAllocator : public cv::MatAllocator {
   public:
    explicit SlowFreeAllocator(std::chrono::milliseconds delay) : delay_(delay) {}

    cv::UMatData *allocate(int dims, const int *sizes, int type, void *data, size_t *step,
                           cv::AccessFlag flags, cv::UMatUsageFlags usage_flags) const override {
        cv::UMatData *u = std_->allocate(dims, sizes, type, data, step, flags, usage_flags);
        if (u != nullptr) u->currAllocator = this;
        return u;
    }
    bool allocate(cv::UMatData *data, cv::AccessFlag access_flags,
                  cv::UMatUsageFlags usage_flags) const override {
        return std_->allocate(data, access_flags, usage_flags);
    }
    void deallocate(cv::UMatData *data) const override {
        freeing.store(true);
        std::this_thread::sleep_for(delay_);
        data->currAllocator = std_;
        std_->deallocate(data);
    }

    mutable std::atomic<bool> freeing{false};

   private:
    std::chrono::milliseconds delay_;
    cv::MatAllocator *std_ = cv::Mat::getStdAllocator();
};

/** A Mat whose buffer comes from `allocator`. */
inline cv::Mat slow_free_mat(SlowFreeAllocator &allocator) {
    cv::Mat mat;
    mat.allocator = &allocator;
    mat.create(8, 8, CV_8UC3);
    return mat;
}

}  // namespace auto_battlebot
