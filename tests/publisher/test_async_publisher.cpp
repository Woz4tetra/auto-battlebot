#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "../slow_free_allocator.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"
#include "publisher/async_publisher.hpp"

namespace auto_battlebot {
namespace {

// Records the stamp of every call it receives and the thread it ran on.
class RecordingPublisher : public PublisherInterface {
   public:
    void publish_camera_data(const CameraData &data) override { record(data.rgb.header.stamp); }
    void publish_field_mask(const MaskStamped &, const RgbImage &, const CameraInfo &) override {}
    void publish_initial_field_description(const FieldDescriptionWithInlierPoints &) override {}
    void publish_field_description(const FieldDescription &,
                                   const FieldDescriptionWithInlierPoints &) override {}
    void publish_hazards(const FieldDescription &) override {}
    void publish_robots(const RobotDescriptionsStamped &robots) override {
        record(robots.header.stamp);
    }
    void publish_blob_detections(const DetectionsStamped &) override {}
    void publish_keypoint_detections(const DetectionsStamped &) override {}
    void publish_navigation(const NavigationVisualization &) override {}

    void record(double stamp) {
        if (delay.count() > 0) std::this_thread::sleep_for(delay);
        std::lock_guard<std::mutex> lock(mutex);
        stamps.push_back(stamp);
        threads.push_back(std::this_thread::get_id());
    }

    std::chrono::milliseconds delay{0};
    std::mutex mutex;
    std::vector<double> stamps;
    std::vector<std::thread::id> threads;
};

class AsyncPublisherTest : public ::testing::Test {
   protected:
    void SetUp() override {
        if (!DiagnosticsLogger::is_initialized()) DiagnosticsLogger::initialize({});
    }
};

// The worker frees each call's frame when it is done with it. A pinned frame's free can be slow,
// and the tick's next enqueue must not wait behind it.
TEST_F(AsyncPublisherTest, EnqueueDoesNotWaitForAFrameBeingFreed) {
    SlowFreeAllocator allocator(std::chrono::milliseconds(300));
    auto inner = std::make_shared<RecordingPublisher>();
    AsyncPublisher publisher(inner);
    {
        CameraData camera;
        camera.rgb.image = slow_free_mat(allocator);
        publisher.publish_camera_data(camera);
    }  // The queued call now holds the frame's only reference.
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!allocator.freeing.load() && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    ASSERT_TRUE(allocator.freeing.load());

    const auto start = std::chrono::steady_clock::now();
    publisher.publish_robots(RobotDescriptionsStamped{});
    EXPECT_LT(std::chrono::steady_clock::now() - start, std::chrono::milliseconds(100));
    publisher.flush();
}

TEST_F(AsyncPublisherTest, RunsCallsInOrderOnAnotherThread) {
    auto inner = std::make_shared<RecordingPublisher>();
    AsyncPublisher publisher(inner);

    CameraData camera;
    RobotDescriptionsStamped robots;
    for (int i = 0; i < 20; ++i) {
        camera.rgb.header.stamp = 2.0 * i;
        robots.header.stamp = 2.0 * i + 1.0;
        publisher.publish_camera_data(camera);
        publisher.publish_robots(robots);
    }
    publisher.flush();

    std::lock_guard<std::mutex> lock(inner->mutex);
    ASSERT_EQ(inner->stamps.size(), 40U);
    for (size_t i = 0; i < inner->stamps.size(); ++i) {
        EXPECT_DOUBLE_EQ(inner->stamps[i], static_cast<double>(i));
        EXPECT_NE(inner->threads[i], std::this_thread::get_id());
    }
}

// A full queue waits for the worker instead of dropping calls, so recordings stay complete.
TEST_F(AsyncPublisherTest, FullQueueBlocksInsteadOfDropping) {
    auto inner = std::make_shared<RecordingPublisher>();
    inner->delay = std::chrono::milliseconds(1);
    AsyncPublisher publisher(inner);

    constexpr int kCalls = static_cast<int>(AsyncPublisher::kMaxPendingCalls) * 2;
    CameraData camera;
    for (int i = 0; i < kCalls; ++i) {
        camera.rgb.header.stamp = i;
        publisher.publish_camera_data(camera);
    }
    publisher.flush();

    std::lock_guard<std::mutex> lock(inner->mutex);
    EXPECT_EQ(inner->stamps.size(), static_cast<size_t>(kCalls));
}

// The recording toggle flushes from its own thread while the tick keeps publishing. A flush
// that waited for an empty queue could wait as long as the tick kept up.
TEST_F(AsyncPublisherTest, FlushFromAnotherThreadReturnsWhileCallsKeepArriving) {
    auto inner = std::make_shared<RecordingPublisher>();
    inner->delay = std::chrono::milliseconds(1);
    AsyncPublisher publisher(inner);

    std::atomic<bool> stop{false};
    std::thread producer([&] {
        CameraData camera;
        while (!stop.load()) publisher.publish_camera_data(camera);
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    const auto start = std::chrono::steady_clock::now();
    publisher.flush();
    const double flush_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    stop.store(true);
    producer.join();
    // At most a full queue of 1 ms calls ahead of it.
    EXPECT_LT(flush_ms, 5.0 * static_cast<double>(AsyncPublisher::kMaxPendingCalls));
}

TEST_F(AsyncPublisherTest, DestructorDrainsQueue) {
    auto inner = std::make_shared<RecordingPublisher>();
    inner->delay = std::chrono::milliseconds(1);
    {
        AsyncPublisher publisher(inner);
        CameraData camera;
        for (int i = 0; i < 10; ++i) publisher.publish_camera_data(camera);
    }
    std::lock_guard<std::mutex> lock(inner->mutex);
    EXPECT_EQ(inner->stamps.size(), 10U);
}

}  // namespace
}  // namespace auto_battlebot
