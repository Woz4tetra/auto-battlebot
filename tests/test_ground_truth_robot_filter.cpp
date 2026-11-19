#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>
#include <optional>

#include "robot_filter/ground_truth_robot_filter.hpp"
#include "simulation/sim_connection.hpp"
#include "time/manual_clock.hpp"

namespace auto_battlebot {
namespace {

constexpr double kFrameIntervalS = 0.1;

class GroundTruthRobotFilterTest : public ::testing::Test {
   protected:
    std::shared_ptr<ManualClock> clock_ = std::make_shared<ManualClock>();

    void feed(double stamp, std::vector<std::optional<Pose2D>> poses,
              GroundTruthRobotFilter &filter) {
        SimMeasurementFrame frame;
        frame.delivered = true;
        for (const auto &pose : poses) {
            frame.robots.push_back({pose.has_value(), pose.value_or(Pose2D{0.0, 0.0, 0.0})});
        }
        SimConnection::instance()->set_last_measurements(frame);
        clock_->set(stamp);
        filter.correct({}, {}, {}, {});
    }

    static std::optional<RobotDescription> find(const GroundTruthRobotFilter &filter,
                                                FrameId frame_id) {
        for (const RobotDescription &desc : filter.state().descriptions) {
            if (desc.frame_id == frame_id) return desc;
        }
        return std::nullopt;
    }
};

TEST_F(GroundTruthRobotFilterTest, NeutralSlotsFollowOpponents) {
    GroundTruthRobotFilter filter(clock_, 1);
    ASSERT_TRUE(filter.initialize(1));
    feed(1.0, {Pose2D{1.0, 0.0, 0.0}, Pose2D{2.0, 0.0, 0.0}, Pose2D{3.0, 0.0, 0.0}}, filter);

    const auto ours = find(filter, FrameId::OUR_ROBOT_1);
    const auto opponent = find(filter, FrameId::THEIR_ROBOT_1);
    const auto neutral = find(filter, FrameId::NEUTRAL_ROBOT_1);
    ASSERT_TRUE(ours && opponent && neutral);
    EXPECT_EQ(ours->group, Group::OURS);
    EXPECT_DOUBLE_EQ(ours->pose.position.x, 1.0);
    EXPECT_EQ(opponent->group, Group::THEIRS);
    EXPECT_DOUBLE_EQ(opponent->pose.position.x, 2.0);
    EXPECT_EQ(neutral->group, Group::NEUTRAL);
    EXPECT_DOUBLE_EQ(neutral->pose.position.x, 3.0);
}

TEST_F(GroundTruthRobotFilterTest, VelocityFromConsecutivePoses) {
    GroundTruthRobotFilter filter(clock_);
    ASSERT_TRUE(filter.initialize(1));
    feed(1.0, {Pose2D{0.0, 0.0, 0.0}, Pose2D{0.0, 0.0, 0.0}}, filter);
    feed(1.0 + kFrameIntervalS, {Pose2D{0.0, 0.0, 0.0}, Pose2D{0.1, 0.0, 0.0}}, filter);

    const auto opponent = find(filter, FrameId::THEIR_ROBOT_1);
    ASSERT_TRUE(opponent);
    EXPECT_NEAR(opponent->velocity.vx_mps, 1.0, 1e-9);
}

TEST_F(GroundTruthRobotFilterTest, NaNSlotCoastsAndIsStale) {
    GroundTruthRobotFilter filter(clock_);
    ASSERT_TRUE(filter.initialize(1));
    feed(1.0, {Pose2D{0.0, 0.0, 0.0}, Pose2D{0.0, 0.0, 0.0}}, filter);
    feed(1.1, {Pose2D{0.0, 0.0, 0.0}, Pose2D{0.1, 0.0, 0.0}}, filter);
    feed(1.2, {Pose2D{0.0, 0.0, 0.0}, std::nullopt}, filter);

    const auto opponent = find(filter, FrameId::THEIR_ROBOT_1);
    ASSERT_TRUE(opponent);
    EXPECT_TRUE(opponent->is_stale);
    EXPECT_NEAR(opponent->pose.position.x, 0.2, 1e-9);
}

TEST_F(GroundTruthRobotFilterTest, FirstMeasurementHasNoSpuriousVelocity) {
    GroundTruthRobotFilter filter(clock_);
    ASSERT_TRUE(filter.initialize(1));
    feed(1.0, {Pose2D{0.0, 0.0, 0.0}, std::nullopt}, filter);
    EXPECT_FALSE(find(filter, FrameId::THEIR_ROBOT_1));

    feed(1.0 + kFrameIntervalS, {Pose2D{0.0, 0.0, 0.0}, Pose2D{2.0, 1.0, 0.0}}, filter);
    const auto opponent = find(filter, FrameId::THEIR_ROBOT_1);
    ASSERT_TRUE(opponent);
    EXPECT_DOUBLE_EQ(opponent->velocity.vx_mps, 0.0);
    EXPECT_DOUBLE_EQ(opponent->velocity.vy_mps, 0.0);
}

}  // namespace
}  // namespace auto_battlebot
