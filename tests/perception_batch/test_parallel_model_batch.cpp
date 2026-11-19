#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include "../slow_free_allocator.hpp"
#include "perception_batch/parallel_model_batch.hpp"

namespace auto_battlebot {
namespace {

ModelResultStamped make_tagged_result(double tag) {
    ModelResultStamped result;
    Keypoint keypoint;
    keypoint.x = tag;
    result.keypoints.push_back(keypoint);
    return result;
}

class FakeKeypointModel : public KeypointModelInterface {
   public:
    bool initialize() override { return true; }
    ModelResultStamped update(RgbImage image, const CameraInfo &camera_info) override {
        ++call_count;
        last_camera_width = camera_info.width;
        {
            std::lock_guard<std::mutex> lock(pairs_mutex);
            seen_pairs.emplace_back(image.header.stamp, camera_info.width);
        }
        if (on_update) on_update();
        return make_tagged_result(kKeypointTag);
    }

    static constexpr double kKeypointTag = 1.0;
    std::atomic<int> call_count{0};
    std::atomic<int> last_camera_width{0};
    std::mutex pairs_mutex;
    std::vector<std::pair<double, int>> seen_pairs;
    std::function<void()> on_update;
};

class FakeRobotBlobModel : public RobotBlobModelInterface {
   public:
    bool initialize() override { return true; }
    ModelResultStamped update(RgbImage) override {
        ++call_count;
        if (on_update) on_update();
        return make_tagged_result(kBlobTag);
    }

    static constexpr double kBlobTag = 2.0;
    std::atomic<int> call_count{0};
    std::function<void()> on_update;
};

class ParallelModelBatchTest : public ::testing::Test {
   protected:
    void SetUp() override {
        keypoint_model_ = std::make_shared<FakeKeypointModel>();
        blob_model_ = std::make_shared<FakeRobotBlobModel>();
    }

    ParallelModelBatch make_batch() { return {keypoint_model_, blob_model_}; }

    std::shared_ptr<FakeKeypointModel> keypoint_model_;
    std::shared_ptr<FakeRobotBlobModel> blob_model_;
    RgbImage image_;
    CameraInfo camera_info_;
};

TEST_F(ParallelModelBatchTest, RoutesEachModelResultToItsField) {
    ParallelModelBatch batch = make_batch();
    BatchResult result = batch.update(image_, camera_info_);

    ASSERT_EQ(result.keypoints.keypoints.size(), 1u);
    EXPECT_EQ(result.keypoints.keypoints[0].x, FakeKeypointModel::kKeypointTag);
    ASSERT_EQ(result.robot_blob_keypoints.keypoints.size(), 1u);
    EXPECT_EQ(result.robot_blob_keypoints.keypoints[0].x, FakeRobotBlobModel::kBlobTag);
    EXPECT_EQ(keypoint_model_->call_count, 1);
    EXPECT_EQ(blob_model_->call_count, 1);
}

TEST_F(ParallelModelBatchTest, PassesCameraInfoToKeypointModel) {
    camera_info_.width = 1920;
    ParallelModelBatch batch = make_batch();
    batch.update(image_, camera_info_);
    EXPECT_EQ(keypoint_model_->last_camera_width, 1920);
}

TEST_F(ParallelModelBatchTest, ImageAndCameraInfoStayPaired) {
    constexpr int kWidthOffset = 1000;
    std::atomic<bool> slow_once{true};
    keypoint_model_->on_update = [&slow_once] {
        if (slow_once.exchange(false)) std::this_thread::sleep_for(std::chrono::milliseconds(1100));
    };
    ParallelModelBatch batch = make_batch();

    for (int request = 1; request <= 3; ++request) {
        image_.header.stamp = request;
        camera_info_.width = kWidthOffset + request;
        batch.update(image_, camera_info_);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    std::lock_guard<std::mutex> lock(keypoint_model_->pairs_mutex);
    ASSERT_FALSE(keypoint_model_->seen_pairs.empty());
    for (const auto &[stamp, width] : keypoint_model_->seen_pairs) {
        EXPECT_EQ(width, kWidthOffset + static_cast<int>(stamp));
    }
}

TEST_F(ParallelModelBatchTest, WorkerKeepsImageAliveAfterTimeout) {
    SlowFreeAllocator allocator(std::chrono::milliseconds(0));
    std::atomic<bool> freed_while_model_ran{false};
    blob_model_->on_update = [&] {
        std::this_thread::sleep_for(std::chrono::milliseconds(1300));
        freed_while_model_ran = allocator.freeing.load();
    };
    ParallelModelBatch batch = make_batch();
    {
        RgbImage image;
        image.image = slow_free_mat(allocator);
        batch.update(image, camera_info_);
    }
    EXPECT_FALSE(allocator.freeing.load());

    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    EXPECT_FALSE(freed_while_model_ran.load());
}

TEST_F(ParallelModelBatchTest, ReusesWorkersAcrossTicks) {
    ParallelModelBatch batch = make_batch();
    for (int i = 0; i < 5; ++i) {
        BatchResult result = batch.update(image_, camera_info_);
        ASSERT_EQ(result.keypoints.keypoints.size(), 1u);
        ASSERT_EQ(result.robot_blob_keypoints.keypoints.size(), 1u);
    }
    EXPECT_EQ(keypoint_model_->call_count, 5);
    EXPECT_EQ(blob_model_->call_count, 5);
}

TEST_F(ParallelModelBatchTest, RunsModelsConcurrently) {
    std::atomic<int> entered{0};
    std::atomic<bool> keypoint_saw_both{false};
    std::atomic<bool> blob_saw_both{false};
    auto rendezvous = [&entered](std::atomic<bool> &saw_both) {
        entered.fetch_add(1);
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
        while (entered.load() < 2 && std::chrono::steady_clock::now() < deadline) {
            std::this_thread::yield();
        }
        saw_both = entered.load() >= 2;
    };
    keypoint_model_->on_update = [&] { rendezvous(keypoint_saw_both); };
    blob_model_->on_update = [&] { rendezvous(blob_saw_both); };

    ParallelModelBatch batch = make_batch();
    batch.update(image_, camera_info_);

    EXPECT_TRUE(keypoint_saw_both);
    EXPECT_TRUE(blob_saw_both);
}

TEST_F(ParallelModelBatchTest, ReportsPerModelElapsedTime) {
    keypoint_model_->on_update = [] { std::this_thread::sleep_for(std::chrono::milliseconds(20)); };
    ParallelModelBatch batch = make_batch();
    BatchResult result = batch.update(image_, camera_info_);

    EXPECT_GE(result.keypoint_model_elapsed_ms, 20.0);
    EXPECT_GE(result.robot_blob_model_elapsed_ms, 0.0);
}

TEST_F(ParallelModelBatchTest, TimedOutModelYieldsEmptyResultThenRecovers) {
    constexpr auto kOverrunsBatchDeadline = std::chrono::milliseconds(1300);
    std::atomic<bool> slow_once{true};
    blob_model_->on_update = [&slow_once, kOverrunsBatchDeadline] {
        if (slow_once.exchange(false)) {
            std::this_thread::sleep_for(kOverrunsBatchDeadline);
        }
    };
    ParallelModelBatch batch = make_batch();

    BatchResult timed_out = batch.update(image_, camera_info_);
    EXPECT_EQ(timed_out.keypoints.keypoints.size(), 1u);
    EXPECT_TRUE(timed_out.robot_blob_keypoints.keypoints.empty());

    // Let the straggler finish its stale request before issuing the next one.
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    BatchResult recovered = batch.update(image_, camera_info_);
    EXPECT_EQ(recovered.keypoints.keypoints.size(), 1u);
    EXPECT_EQ(recovered.robot_blob_keypoints.keypoints.size(), 1u);
}

TEST_F(ParallelModelBatchTest, LateResultAfterTimeoutIsDropped) {
    std::atomic<int> slow_calls{2};
    blob_model_->on_update = [&slow_calls] {
        if (slow_calls.fetch_sub(1) > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1300));
        }
    };
    ParallelModelBatch batch = make_batch();

    batch.update(image_, camera_info_);
    BatchResult second = batch.update(image_, camera_info_);

    EXPECT_TRUE(second.robot_blob_keypoints.keypoints.empty())
        << "result of the timed-out first request landed in the second";
}

TEST_F(ParallelModelBatchTest, ThrowingModelYieldsEmptyResultWithoutWaitingOrDying) {
    std::atomic<bool> throw_once{true};
    keypoint_model_->on_update = [&throw_once] {
        if (throw_once.exchange(false)) throw std::runtime_error("CUDA error");
    };
    ParallelModelBatch batch = make_batch();

    const auto start = std::chrono::steady_clock::now();
    BatchResult threw = batch.update(image_, camera_info_);
    constexpr auto kWellUnderBatchTimeout = std::chrono::milliseconds(500);
    EXPECT_LT(std::chrono::steady_clock::now() - start, kWellUnderBatchTimeout)
        << "the worker reports the frame done, so update() does not wait out its timeout";
    EXPECT_TRUE(threw.keypoints.keypoints.empty());
    EXPECT_EQ(threw.robot_blob_keypoints.keypoints.size(), 1u);

    BatchResult recovered = batch.update(image_, camera_info_);
    EXPECT_EQ(recovered.keypoints.keypoints.size(), 1u);
}

}  // namespace
}  // namespace auto_battlebot
