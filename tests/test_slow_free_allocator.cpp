#include <gtest/gtest.h>

#include <chrono>
#include <memory>

#include "slow_free_allocator.hpp"

namespace auto_battlebot {

TEST(SlowFreeAllocatorTest, DestroyingItBeforeItsMatsAborts) {
    EXPECT_DEATH(
        {
            auto allocator = std::make_unique<SlowFreeAllocator>(std::chrono::milliseconds(0));
            cv::Mat mat = slow_free_mat(*allocator);
            allocator.reset();
        },
        "live Mats");
}

TEST(SlowFreeAllocatorTest, DestroyingItAfterItsMatsAreFreedIsSilent) {
    SlowFreeAllocator allocator(std::chrono::milliseconds(0));
    { cv::Mat mat = slow_free_mat(allocator); }
    EXPECT_TRUE(allocator.freeing.load());
}

}  // namespace auto_battlebot
