#include <gtest/gtest.h>

#include <cstdlib>
#include <opencv2/core.hpp>

#include "tensorrt_inference/pinned_mat_allocator.hpp"

namespace auto_battlebot {
namespace {

struct FrameHolder {
    cv::Mat frame;
};

void hold_a_frame_past_the_allocators_first_use() {
    static FrameHolder holder;
    PinnedMatAllocator *allocator = PinnedMatAllocator::instance();
    {
        cv::Mat pooled;
        pooled.allocator = allocator;
        pooled.create(64, 64, CV_8UC3);
    }
    if (allocator->pooled_buffers() == 0) std::exit(2);
    holder.frame.allocator = allocator;
    holder.frame.create(64, 64, CV_8UC3);
    std::exit(0);
}

TEST(PinnedMatAllocatorTest, InstanceOutlivesFramesReleasedDuringStaticDestruction) {
    GTEST_FLAG_SET(death_test_style, "threadsafe");
    EXPECT_EXIT(hold_a_frame_past_the_allocators_first_use(), ::testing::ExitedWithCode(0), "");
}

}  // namespace
}  // namespace auto_battlebot
