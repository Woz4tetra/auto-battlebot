#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <thread>

#include "viz/image_encoder_worker.hpp"

namespace auto_battlebot {
namespace {

bool wait_for(const std::function<bool()> &done) {
    for (int i = 0; i < 300; ++i) {
        if (done()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return false;
}

TEST(ImageEncoderWorkerTest, SubmitDoesNotWaitForASlowEncoder) {
    std::atomic<int> processed{0};
    ImageEncoderWorker worker("slow", [&](const ImageEncoderWorker::Job &) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        ++processed;
    });
    const auto start = std::chrono::steady_clock::now();
    worker.submit({cv::Mat(4, 4, CV_8UC3), {}, 1});
    worker.submit({cv::Mat(4, 4, CV_8UC3), {}, 2});
    EXPECT_LT(std::chrono::steady_clock::now() - start, std::chrono::milliseconds(20));
    EXPECT_TRUE(wait_for([&] { return processed.load() >= 1; }));
}

TEST(ImageEncoderWorkerTest, BurstEncodesTheNewestAndCountsTheRestReplaced) {
    std::atomic<uint64_t> last_seen{0};
    std::atomic<bool> started{false};
    std::atomic<bool> release{false};
    ImageEncoderWorker worker("burst", [&](const ImageEncoderWorker::Job &job) {
        started = true;
        while (!release.load()) std::this_thread::sleep_for(std::chrono::milliseconds(1));
        last_seen = job.log_time_ns;
    });
    // The first frame is picked up and blocks the worker; the next five pile into one slot.
    worker.submit({cv::Mat(4, 4, CV_8UC3), {}, 1});
    const bool picked_up = wait_for([&] { return started.load(); });
    for (uint64_t t = 2; t <= 6; ++t) worker.submit({cv::Mat(4, 4, CV_8UC3), {}, t});
    release = true;
    ASSERT_TRUE(picked_up);
    ASSERT_TRUE(wait_for([&] { return worker.stats().encoded_frames == 2; }));
    EXPECT_EQ(last_seen.load(), 6u);
    EXPECT_EQ(worker.stats().replaced_frames, 4u);
}

}  // namespace
}  // namespace auto_battlebot
