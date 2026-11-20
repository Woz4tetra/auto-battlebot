#pragma once

#include <cstddef>
#include <map>
#include <mutex>
#include <opencv2/core.hpp>
#include <vector>

namespace auto_battlebot {

class PinnedMatAllocator : public cv::MatAllocator {
   public:
    static PinnedMatAllocator *instance();

    cv::UMatData *allocate(int dims, const int *sizes, int type, void *data, size_t *step,
                           cv::AccessFlag flags, cv::UMatUsageFlags usage_flags) const override;
    bool allocate(cv::UMatData *data, cv::AccessFlag access_flags,
                  cv::UMatUsageFlags usage_flags) const override;
    void deallocate(cv::UMatData *data) const override;

    size_t pooled_buffers() const;

   private:
    PinnedMatAllocator() = default;

    static constexpr size_t kMaxPooledPerSize = 16;

    mutable std::mutex mutex_;
    mutable std::map<size_t, std::vector<void *>> pool_;
};

}  // namespace auto_battlebot
