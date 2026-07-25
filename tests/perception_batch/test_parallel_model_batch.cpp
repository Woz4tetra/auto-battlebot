#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <thread>

#include "perception_batch/parallel_model_batch.hpp"

namespace auto_battlebot {
namespace {

KeypointsStamped make_tagged_result(double tag) {
    KeypointsStamped result;
    Keypoint keypoint;
    keypoint.x = tag;
    result.keypoints.push_back(keypoint);
    return result;
}

class FakeKeypointModel : public KeypointModelInterface {
   public:
    bool initialize() override { return true; }
    KeypointsStamped update(RgbImage /*image*/) override {
        ++call_count;
        if (on_update) on_update();
        return make_tagged_result(kKeypointTag);
    }

    static constexpr double kKeypointTag = 1.0;
    std::atomic<int> call_count{0};
    std::function<void()> on_update;
};

class FakeRobotBlobModel : public RobotBlobModelInterface {
   public:
    bool initialize() override { return true; }
    KeypointsStamped update(RgbImage /*image*/) override {
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
};

TEST_F(ParallelModelBatchTest, RoutesEachModelResultToItsField) {
    ParallelModelBatch batch = make_batch();
    BatchResult result = batch.update(image_);

    ASSERT_EQ(result.keypoints.keypoints.size(), 1u);
    EXPECT_EQ(result.keypoints.keypoints[0].x, FakeKeypointModel::kKeypointTag);
    ASSERT_EQ(result.robot_blob_keypoints.keypoints.size(), 1u);
    EXPECT_EQ(result.robot_blob_keypoints.keypoints[0].x, FakeRobotBlobModel::kBlobTag);
    EXPECT_EQ(keypoint_model_->call_count, 1);
    EXPECT_EQ(blob_model_->call_count, 1);
}

TEST_F(ParallelModelBatchTest, ReusesWorkersAcrossTicks) {
    ParallelModelBatch batch = make_batch();
    for (int i = 0; i < 5; ++i) {
        BatchResult result = batch.update(image_);
        ASSERT_EQ(result.keypoints.keypoints.size(), 1u);
        ASSERT_EQ(result.robot_blob_keypoints.keypoints.size(), 1u);
    }
    EXPECT_EQ(keypoint_model_->call_count, 5);
    EXPECT_EQ(blob_model_->call_count, 5);
}

TEST_F(ParallelModelBatchTest, RunsModelsConcurrently) {
    // Each model waits until the other has entered update(). If the batch ran them
    // sequentially, the first model would time out waiting and saw_both stays false.
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
    batch.update(image_);

    EXPECT_TRUE(keypoint_saw_both);
    EXPECT_TRUE(blob_saw_both);
}

TEST_F(ParallelModelBatchTest, ReportsPerModelElapsedTime) {
    keypoint_model_->on_update = [] { std::this_thread::sleep_for(std::chrono::milliseconds(20)); };
    ParallelModelBatch batch = make_batch();
    BatchResult result = batch.update(image_);

    EXPECT_GE(result.keypoint_model_elapsed_ms, 20.0);
    EXPECT_GE(result.robot_blob_model_elapsed_ms, 0.0);
}

TEST_F(ParallelModelBatchTest, TimedOutModelYieldsEmptyResultThenRecovers) {
    // First blob update overruns the batch wait deadline; that tick must still return the
    // keypoint result with an empty blob result, and the next tick must work normally.
    std::atomic<bool> slow_once{true};
    blob_model_->on_update = [&slow_once] {
        if (slow_once.exchange(false)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1300));
        }
    };
    ParallelModelBatch batch = make_batch();

    BatchResult timed_out = batch.update(image_);
    EXPECT_EQ(timed_out.keypoints.keypoints.size(), 1u);
    EXPECT_TRUE(timed_out.robot_blob_keypoints.keypoints.empty());

    // Let the straggler finish its stale request before issuing the next one.
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    BatchResult recovered = batch.update(image_);
    EXPECT_EQ(recovered.keypoints.keypoints.size(), 1u);
    EXPECT_EQ(recovered.robot_blob_keypoints.keypoints.size(), 1u);
}

}  // namespace
}  // namespace auto_battlebot
