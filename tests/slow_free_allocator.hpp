#pragma once

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <opencv2/core.hpp>
#include <thread>

namespace auto_battlebot {

class SlowFreeAllocator : public cv::MatAllocator {
   public:
    explicit SlowFreeAllocator(std::chrono::milliseconds delay) : delay_(delay) {}

    ~SlowFreeAllocator() override {
        if (live_.load() != 0) {
            std::cerr << "SlowFreeAllocator destroyed with " << live_.load() << " live Mats\n";
            std::abort();
        }
    }

    cv::UMatData *allocate(int dims, const int *sizes, int type, void *data, size_t *step,
                           cv::AccessFlag flags, cv::UMatUsageFlags usage_flags) const override {
        cv::UMatData *u = std_->allocate(dims, sizes, type, data, step, flags, usage_flags);
        if (u != nullptr) {
            u->currAllocator = this;
            live_.fetch_add(1);
        }
        return u;
    }
    bool allocate(cv::UMatData *data, cv::AccessFlag access_flags,
                  cv::UMatUsageFlags usage_flags) const override {
        return std_->allocate(data, access_flags, usage_flags);
    }
    void deallocate(cv::UMatData *data) const override {
        freeing.store(true);
        live_.fetch_sub(1);
        std::this_thread::sleep_for(delay_);
        data->currAllocator = std_;
        std_->deallocate(data);
    }

    mutable std::atomic<bool> freeing{false};

   private:
    mutable std::atomic<int> live_{0};
    std::chrono::milliseconds delay_;
    cv::MatAllocator *std_ = cv::Mat::getStdAllocator();
};

inline cv::Mat slow_free_mat(SlowFreeAllocator &allocator) {
    cv::Mat mat;
    mat.allocator = &allocator;
    mat.create(8, 8, CV_8UC3);
    return mat;
}

}  // namespace auto_battlebot
