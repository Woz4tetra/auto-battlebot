#include <gtest/gtest.h>

#include <cmath>

#include "navigation/pursuit_navigation.hpp"
#include "time/manual_clock.hpp"
#include "transform_utils.hpp"

namespace auto_battlebot {
RobotDescription make_robot(Label label, double x, double y, double yaw) {
    RobotDescription robot;
    robot.label = label;
    robot.pose.position.x = x;
    robot.pose.position.y = y;
    robot.pose.position.z = 0.0;
    robot.pose.rotation = euler_to_quaternion(0.0, 0.0, yaw);
    robot.velocity = Velocity2D{0.0, 0.0, 0.0};
    return robot;
}

RobotDescription make_robot_with_frame(Label label, FrameId frame_id, Group group, double x,
                                       double y, double yaw) {
    RobotDescription robot = make_robot(label, x, y, yaw);
    robot.frame_id = frame_id;
    robot.group = group;
    return robot;
}

FieldDescription make_field(double width, double height) {
    FieldDescription field;
    field.size.size.x = width;
    field.size.size.y = height;
    return field;
}

TargetSelection make_target(double x, double y, Label label = Label::OPPONENT) {
    TargetSelection target;
    target.pose.x = x;
    target.pose.y = y;
    target.pose.yaw = 0.0;
    target.label = label;
    return target;
}

PursuitNavigationConfiguration make_default_config() { return PursuitNavigationConfiguration{}; }

RobotDescription make_our_robot_at_origin_facing_x() {
    return make_robot_with_frame(Label::MR_STABS_MK1, FrameId::OUR_ROBOT_1, Group::OURS, 0.0, 0.0,
                                 0.0);
}

void isolate_derivative_term(PursuitNavigationConfiguration& config) {
    config.angular_kp = 0.0;
    config.angle_threshold = M_PI;
    config.max_angular_command = std::nullopt;
}

constexpr double kFarOpponentX = 2.0;
constexpr double kCloseOpponentX = 0.5;
constexpr double kOutsideFieldX = 10.0;
constexpr double kSmallFieldSize = 2.0;
constexpr double kWallReverseDistanceM = 0.3;
constexpr double kNearWallX = kSmallFieldSize / 2.0 - 0.1;
constexpr double kTargetAheadOfWallX = 0.5;

class PursuitNavigationTest : public ::testing::Test {
   protected:
    void SetUp() override {
        config_ = make_default_config();
        clock_ = std::make_shared<ManualClock>();
        nav_ = std::make_unique<PursuitNavigation>(config_, clock_);
        nav_->initialize();
    }

    PursuitNavigationConfiguration config_;
    std::shared_ptr<ManualClock> clock_;
    std::unique_ptr<PursuitNavigation> nav_;
};

TEST(PursuitNavigationConfigTest, DefaultValues) {
    PursuitNavigationConfiguration config;

    EXPECT_EQ(config.type, "PursuitNavigation");
    EXPECT_DOUBLE_EQ(config.stop_distance_m, 0.0);
    EXPECT_DOUBLE_EQ(config.velocity_ramp_far_distance, 2.5);
    EXPECT_DOUBLE_EQ(config.velocity_ramp_near_distance, 1.0);
    EXPECT_DOUBLE_EQ(config.velocity_ramp_min_scale, 0.5);
    EXPECT_DOUBLE_EQ(config.angular_kp, 3.0);
    EXPECT_DOUBLE_EQ(config.angle_threshold, 0.5);
    EXPECT_DOUBLE_EQ(config.lookahead_time, 0.1);
    EXPECT_DOUBLE_EQ(config.boundary_margin, 0.1);
}

TEST(PursuitNavigationConfigTest, CustomValues) {
    PursuitNavigationConfiguration config;
    config.velocity_ramp_far_distance = 2.0;
    config.velocity_ramp_near_distance = 0.5;
    config.stop_distance_m = 0.2;
    config.angular_kp = 3.0;
    config.angle_threshold = 0.5;
    config.lookahead_time = 0.2;
    config.boundary_margin = 0.15;

    PursuitNavigation nav(config, std::make_shared<ManualClock>());
    EXPECT_TRUE(nav.initialize());
}

TEST_F(PursuitNavigationTest, InitializeReturnsTrue) {
    PursuitNavigationConfiguration config;
    PursuitNavigation nav(config, std::make_shared<ManualClock>());
    EXPECT_TRUE(nav.initialize());
}

TEST_F(PursuitNavigationTest, NoRobotsReturnsZeroCommand) {
    RobotDescriptionsStamped robots;
    FieldDescription field = make_field(2.0, 2.0);

    VelocityCommand cmd = nav_->update(robots, field, make_target(0.0, 0.0, Label::EMPTY));

    EXPECT_DOUBLE_EQ(cmd.forward_mps, 0.0);
    EXPECT_DOUBLE_EQ(cmd.lateral_mps, 0.0);
    EXPECT_DOUBLE_EQ(cmd.yaw_rate_radps, 0.0);
}

TEST_F(PursuitNavigationTest, DerivativeTermUsesLogicalClockDt) {
    PursuitNavigationConfiguration config = make_default_config();
    isolate_derivative_term(config);
    config.angular_kd = 0.1;
    config.enable_hysteresis = false;
    auto clock = std::make_shared<ManualClock>();
    PursuitNavigation nav(config, clock);
    nav.initialize();

    FieldDescription field = make_field(4.0, 4.0);
    RobotDescriptionsStamped robots;
    robots.descriptions.push_back(make_robot_with_frame(Label::MR_STABS_MK1, FrameId::OUR_ROBOT_1,
                                                        Group::OURS, 0.0, 0.0, 0.0));

    constexpr double kFirstTickS = 1.0;
    constexpr double kDtS = 0.1;
    constexpr double kAngleErrorAt45DegRad = M_PI / 4.0;
    const TargetSelection target_ahead = make_target(1.0, 0.0);
    const TargetSelection target_at_45_deg = make_target(1.0, 1.0);

    clock->set(kFirstTickS);
    VelocityCommand c1 = nav.update(robots, field, target_ahead);
    EXPECT_NEAR(c1.yaw_rate_radps, 0.0, 1e-9);

    clock->set(kFirstTickS + kDtS);
    VelocityCommand c2 = nav.update(robots, field, target_at_45_deg);
    const double expected = config.angular_kd * kAngleErrorAt45DegRad / kDtS;
    EXPECT_NEAR(c2.yaw_rate_radps, expected, 1e-6);
}

TEST_F(PursuitNavigationTest, NoOurRobotReturnsZeroCommand) {
    RobotDescriptionsStamped robots;
    robots.descriptions.push_back(make_robot(Label::OPPONENT, 1.0, 0.0, 0.0));
    FieldDescription field = make_field(2.0, 2.0);

    VelocityCommand cmd = nav_->update(robots, field, make_target(1.0, 0.0));

    EXPECT_DOUBLE_EQ(cmd.forward_mps, 0.0);
    EXPECT_DOUBLE_EQ(cmd.lateral_mps, 0.0);
    EXPECT_DOUBLE_EQ(cmd.yaw_rate_radps, 0.0);
}

TEST_F(PursuitNavigationTest, UsesCachedOurRobotPoseWhenTemporarilyMissing) {
    FieldDescription field = make_field(4.0, 4.0);
    TargetSelection target = make_target(1.0, 0.0);

    RobotDescriptionsStamped first_tick_robots;
    first_tick_robots.descriptions.push_back(make_robot_with_frame(
        Label::MR_STABS_MK1, FrameId::OUR_ROBOT_1, Group::OURS, 0.0, 0.0, 0.0));
    first_tick_robots.descriptions.push_back(make_robot_with_frame(
        Label::OPPONENT, FrameId::THEIR_ROBOT_1, Group::THEIRS, 1.0, 0.0, 0.0));

    VelocityCommand first_cmd = nav_->update(first_tick_robots, field, target);
    EXPECT_GT(first_cmd.forward_mps, 0.0);

    RobotDescriptionsStamped second_tick_robots;
    second_tick_robots.descriptions.push_back(make_robot_with_frame(
        Label::OPPONENT, FrameId::THEIR_ROBOT_1, Group::THEIRS, 1.0, 0.0, 0.0));

    VelocityCommand second_cmd = nav_->update(second_tick_robots, field, target);
    EXPECT_GT(second_cmd.forward_mps, 0.0);
}

TEST_F(PursuitNavigationTest, MissingOurRobotWithoutCacheStillReturnsZeroCommand) {
    FieldDescription field = make_field(4.0, 4.0);
    TargetSelection target = make_target(1.0, 0.0);

    RobotDescriptionsStamped robots;
    robots.descriptions.push_back(make_robot_with_frame(Label::OPPONENT, FrameId::THEIR_ROBOT_1,
                                                        Group::THEIRS, 1.0, 0.0, 0.0));

    VelocityCommand cmd = nav_->update(robots, field, target);
    EXPECT_DOUBLE_EQ(cmd.forward_mps, 0.0);
    EXPECT_DOUBLE_EQ(cmd.lateral_mps, 0.0);
    EXPECT_DOUBLE_EQ(cmd.yaw_rate_radps, 0.0);
}

TEST_F(PursuitNavigationTest, NoTargetReturnsZeroCommand) {
    RobotDescriptionsStamped robots;
    robots.descriptions.push_back(make_robot(Label::MR_STABS_MK1, 0.0, 0.0, 0.0));
    FieldDescription field = make_field(2.0, 2.0);

    VelocityCommand cmd = nav_->update(robots, field, make_target(0.0, 0.0, Label::EMPTY));

    EXPECT_DOUBLE_EQ(cmd.forward_mps, 0.0);
    EXPECT_DOUBLE_EQ(cmd.lateral_mps, 0.0);
    EXPECT_DOUBLE_EQ(cmd.yaw_rate_radps, 0.0);
}

TEST_F(PursuitNavigationTest, FindsOurRobotMrStabsMk1) {
    RobotDescriptionsStamped robots;
    robots.descriptions.push_back(make_robot_with_frame(Label::MR_STABS_MK1, FrameId::OUR_ROBOT_1,
                                                        Group::OURS, 0.0, 0.0, 0.0));
    robots.descriptions.push_back(make_robot(Label::OPPONENT, 1.0, 0.0, 0.0));
    FieldDescription field = make_field(2.0, 2.0);

    VelocityCommand cmd = nav_->update(robots, field, make_target(1.0, 0.0));

    EXPECT_TRUE(cmd.forward_mps != 0.0 || cmd.yaw_rate_radps != 0.0);
}

TEST_F(PursuitNavigationTest, FindsOurRobotMrStabsMk2) {
    RobotDescriptionsStamped robots;
    robots.descriptions.push_back(make_robot_with_frame(Label::MR_STABS_MK2, FrameId::OUR_ROBOT_1,
                                                        Group::OURS, 0.0, 0.0, 0.0));
    robots.descriptions.push_back(make_robot(Label::OPPONENT, 1.0, 0.0, 0.0));
    FieldDescription field = make_field(2.0, 2.0);

    VelocityCommand cmd = nav_->update(robots, field, make_target(1.0, 0.0));
    EXPECT_TRUE(cmd.forward_mps != 0.0 || cmd.yaw_rate_radps != 0.0);
}

TEST_F(PursuitNavigationTest, FindsOurRobotMrsBuffMk1) {
    RobotDescriptionsStamped robots;
    robots.descriptions.push_back(make_robot_with_frame(Label::MRS_BUFF_MK1, FrameId::OUR_ROBOT_1,
                                                        Group::OURS, 0.0, 0.0, 0.0));
    robots.descriptions.push_back(make_robot(Label::OPPONENT, 1.0, 0.0, 0.0));
    FieldDescription field = make_field(2.0, 2.0);

    VelocityCommand cmd = nav_->update(robots, field, make_target(1.0, 0.0));
    EXPECT_TRUE(cmd.forward_mps != 0.0 || cmd.yaw_rate_radps != 0.0);
}

TEST_F(PursuitNavigationTest, FindsOurRobotMrsBuffMk2) {
    RobotDescriptionsStamped robots;
    robots.descriptions.push_back(make_robot_with_frame(Label::MRS_BUFF_MK2, FrameId::OUR_ROBOT_1,
                                                        Group::OURS, 0.0, 0.0, 0.0));
    robots.descriptions.push_back(make_robot(Label::OPPONENT, 1.0, 0.0, 0.0));
    FieldDescription field = make_field(2.0, 2.0);

    VelocityCommand cmd = nav_->update(robots, field, make_target(1.0, 0.0));
    EXPECT_TRUE(cmd.forward_mps != 0.0 || cmd.yaw_rate_radps != 0.0);
}

TEST_F(PursuitNavigationTest, TargetsOpponent) {
    RobotDescriptionsStamped robots;
    robots.descriptions.push_back(make_robot_with_frame(Label::MR_STABS_MK1, FrameId::OUR_ROBOT_1,
                                                        Group::OURS, 0.0, 0.0, 0.0));
    robots.descriptions.push_back(make_robot(Label::OPPONENT, 1.0, 0.0, 0.0));
    FieldDescription field = make_field(4.0, 4.0);

    VelocityCommand cmd = nav_->update(robots, field, make_target(1.0, 0.0));
    EXPECT_TRUE(cmd.forward_mps != 0.0 || cmd.yaw_rate_radps != 0.0);
}

TEST_F(PursuitNavigationTest, TargetsHouseBot) {
    RobotDescriptionsStamped robots;
    robots.descriptions.push_back(make_robot_with_frame(Label::MR_STABS_MK1, FrameId::OUR_ROBOT_1,
                                                        Group::OURS, 0.0, 0.0, 0.0));
    robots.descriptions.push_back(make_robot(Label::HOUSE_BOT, 1.0, 0.0, 0.0));
    FieldDescription field = make_field(4.0, 4.0);

    VelocityCommand cmd = nav_->update(robots, field, make_target(1.0, 0.0, Label::HOUSE_BOT));
    EXPECT_TRUE(cmd.forward_mps != 0.0 || cmd.yaw_rate_radps != 0.0);
}

TEST_F(PursuitNavigationTest, TargetsClosestOpponent) {
    RobotDescriptionsStamped robots;
    robots.descriptions.push_back(make_our_robot_at_origin_facing_x());
    robots.descriptions.push_back(make_robot(Label::OPPONENT, kFarOpponentX, 0.0, 0.0));
    robots.descriptions.push_back(make_robot(Label::OPPONENT, kCloseOpponentX, 0.0, 0.0));
    FieldDescription field = make_field(4.0, 4.0);

    VelocityCommand cmd = nav_->update(robots, field, make_target(kCloseOpponentX, 0.0));

    EXPECT_GT(cmd.forward_mps, 0.0);
}

TEST_F(PursuitNavigationTest, MovesForwardWhenFacingTarget) {
    RobotDescriptionsStamped robots;
    robots.descriptions.push_back(make_our_robot_at_origin_facing_x());
    robots.descriptions.push_back(make_robot(Label::OPPONENT, 1.0, 0.0, 0.0));
    FieldDescription field = make_field(4.0, 4.0);

    VelocityCommand cmd = nav_->update(robots, field, make_target(1.0, 0.0));

    EXPECT_GT(cmd.forward_mps, 0.0);
    EXPECT_NEAR(cmd.yaw_rate_radps, 0.0, 0.1);
}

TEST_F(PursuitNavigationTest, TurnsWhenTargetToLeft) {
    RobotDescriptionsStamped robots;
    robots.descriptions.push_back(make_our_robot_at_origin_facing_x());
    robots.descriptions.push_back(make_robot(Label::OPPONENT, 0.0, 1.0, 0.0));
    FieldDescription field = make_field(4.0, 4.0);

    VelocityCommand cmd = nav_->update(robots, field, make_target(0.0, 1.0));

    EXPECT_GT(cmd.yaw_rate_radps, 0.0);
}

TEST_F(PursuitNavigationTest, TurnsWhenTargetToRight) {
    RobotDescriptionsStamped robots;
    robots.descriptions.push_back(make_our_robot_at_origin_facing_x());
    robots.descriptions.push_back(make_robot(Label::OPPONENT, 0.0, -1.0, 0.0));
    FieldDescription field = make_field(4.0, 4.0);

    VelocityCommand cmd = nav_->update(robots, field, make_target(0.0, -1.0));

    EXPECT_LT(cmd.yaw_rate_radps, 0.0);
}

TEST_F(PursuitNavigationTest, TurnsInPlaceWhenTargetBehind) {
    RobotDescriptionsStamped robots;
    robots.descriptions.push_back(make_our_robot_at_origin_facing_x());
    robots.descriptions.push_back(make_robot(Label::OPPONENT, -1.0, 0.0, 0.0));
    FieldDescription field = make_field(4.0, 4.0);

    VelocityCommand cmd = nav_->update(robots, field, make_target(-1.0, 0.0));

    EXPECT_NEAR(cmd.forward_mps, 0.0, 0.01) << "large angle error turns in place";
    EXPECT_NE(cmd.yaw_rate_radps, 0.0);
}

TEST_F(PursuitNavigationTest, TurnLatchHoldsBehindTarget) {
    RobotDescriptionsStamped robots;
    robots.descriptions.push_back(make_robot_with_frame(Label::MR_STABS_MK1, FrameId::OUR_ROBOT_1,
                                                        Group::OURS, 0.0, 0.0, 0.0));
    FieldDescription field = make_field(4.0, 4.0);
    const double behind_left = 170.0 * M_PI / 180.0;
    const double behind_right = -170.0 * M_PI / 180.0;

    VelocityCommand first =
        nav_->update(robots, field, make_target(std::cos(behind_left), std::sin(behind_left)));
    VelocityCommand second =
        nav_->update(robots, field, make_target(std::cos(behind_right), std::sin(behind_right)));

    ASSERT_GT(first.yaw_rate_radps, 0.0);
    EXPECT_GT(second.yaw_rate_radps, 0.0) << "the turn flipped sign while the target sat behind";
}

TEST_F(PursuitNavigationTest, NoForwardSpeedUnlessFacingTarget) {
    RobotDescriptionsStamped robots;
    robots.descriptions.push_back(make_robot_with_frame(Label::MR_STABS_MK1, FrameId::OUR_ROBOT_1,
                                                        Group::OURS, 0.0, 0.0, 0.0));
    FieldDescription field = make_field(4.0, 4.0);
    const double beyond_threshold = config_.angle_threshold + 0.02;

    VelocityCommand cmd = nav_->update(
        robots, field, make_target(std::cos(beyond_threshold), std::sin(beyond_threshold)));

    EXPECT_DOUBLE_EQ(cmd.forward_mps, 0.0) << "drove forward while not facing the target";
}

TEST_F(PursuitNavigationTest, StopsAtTarget) {
    RobotDescriptionsStamped robots;
    robots.descriptions.push_back(make_robot(Label::MR_STABS_MK1, 0.0, 0.0, 0.0));
    robots.descriptions.push_back(make_robot(Label::OPPONENT, 0.05, 0.0, 0.0));
    FieldDescription field = make_field(4.0, 4.0);

    VelocityCommand cmd = nav_->update(robots, field, make_target(0.05, 0.0));

    EXPECT_DOUBLE_EQ(cmd.forward_mps, 0.0);
    EXPECT_DOUBLE_EQ(cmd.yaw_rate_radps, 0.0);
}

TEST_F(PursuitNavigationTest, VelocityRampReducesSpeedAtFarDistance) {
    PursuitNavigationConfiguration config;
    config.velocity_ramp_far_distance = 1.0;
    config.velocity_ramp_near_distance = 0.3;
    config.velocity_ramp_min_scale = 0.5;
    config.stop_distance_m = 0.05;
    PursuitNavigation nav(config, std::make_shared<ManualClock>());
    nav.initialize();

    RobotDescriptionsStamped robots;
    robots.descriptions.push_back(make_robot_with_frame(Label::MR_STABS_MK1, FrameId::OUR_ROBOT_1,
                                                        Group::OURS, 0.0, 0.0, 0.0));
    const double target_x = 2.0 * config.velocity_ramp_far_distance;
    robots.descriptions.push_back(make_robot(Label::OPPONENT, target_x, 0.0, 0.0));
    FieldDescription field = make_field(10.0, 10.0);

    VelocityCommand cmd = nav.update(robots, field, make_target(target_x, 0.0));

    EXPECT_GT(cmd.forward_mps, 0.0);
    EXPECT_LE(cmd.forward_mps, config.velocity_ramp_min_scale);
}

TEST_F(PursuitNavigationTest, VelocityRampFullSpeedBelowNearDistance) {
    PursuitNavigationConfiguration config;
    config.velocity_ramp_far_distance = 2.0;
    config.velocity_ramp_near_distance = 1.0;
    config.velocity_ramp_min_scale = 0.5;
    config.stop_distance_m = 0.05;
    PursuitNavigation nav(config, std::make_shared<ManualClock>());
    nav.initialize();

    RobotDescriptionsStamped robots;
    robots.descriptions.push_back(make_robot_with_frame(Label::MR_STABS_MK1, FrameId::OUR_ROBOT_1,
                                                        Group::OURS, 0.0, 0.0, 0.0));
    const double inside_near_distance_x = 0.5 * config.velocity_ramp_near_distance;
    robots.descriptions.push_back(make_robot(Label::OPPONENT, inside_near_distance_x, 0.0, 0.0));
    FieldDescription field = make_field(10.0, 10.0);

    VelocityCommand cmd = nav.update(robots, field, make_target(inside_near_distance_x, 0.0));

    EXPECT_GT(cmd.forward_mps, 0.9) << "angle error ~0, so turn scaling ~1 and speed_scale 1";
}

TEST_F(PursuitNavigationTest, ForwardVelocityIsBoundedByControllerScaling) {
    PursuitNavigationConfiguration config;
    PursuitNavigation nav(config, std::make_shared<ManualClock>());
    nav.initialize();

    RobotDescriptionsStamped robots;
    robots.descriptions.push_back(make_robot(Label::MR_STABS_MK1, 0.0, 0.0, 0.0));
    robots.descriptions.push_back(make_robot(Label::OPPONENT, 10.0, 0.0, 0.0));
    FieldDescription field = make_field(30.0, 30.0);

    VelocityCommand cmd = nav.update(robots, field, make_target(10.0, 0.0));

    EXPECT_LE(cmd.forward_mps, 1.0);
}

TEST_F(PursuitNavigationTest, AngularVelocityUsesRealUnits) {
    PursuitNavigationConfiguration config;
    config.angular_kp = 2.0;
    PursuitNavigation nav(config, std::make_shared<ManualClock>());
    nav.initialize();

    RobotDescriptionsStamped robots;
    robots.descriptions.push_back(make_robot_with_frame(Label::MR_STABS_MK1, FrameId::OUR_ROBOT_1,
                                                        Group::OURS, 0.0, 0.0, 0.0));
    constexpr double kBehindX = -1.0;
    constexpr double kBehindY = 0.1;
    robots.descriptions.push_back(make_robot(Label::OPPONENT, kBehindX, kBehindY, 0.0));
    FieldDescription field = make_field(4.0, 4.0);

    VelocityCommand cmd = nav.update(robots, field, make_target(kBehindX, kBehindY));

    EXPECT_GT(std::abs(cmd.yaw_rate_radps), 1.0);
}

TEST_F(PursuitNavigationTest, NavigatesWithLookaheadTimeConfigured) {
    PursuitNavigationConfiguration config;
    config.lookahead_time = 1.0;
    PursuitNavigation nav(config, std::make_shared<ManualClock>());
    nav.initialize();

    RobotDescriptionsStamped robots;
    robots.descriptions.push_back(make_our_robot_at_origin_facing_x());

    RobotDescription target = make_robot(Label::OPPONENT, 1.0, 0.0, 0.0);
    target.velocity.vy_mps = 1.0;
    robots.descriptions.push_back(target);

    FieldDescription field = make_field(4.0, 4.0);

    VelocityCommand cmd = nav.update(robots, field, make_target(1.0, 0.0));

    EXPECT_GT(cmd.forward_mps, 0.0);
}

TEST_F(PursuitNavigationTest, ClampsTargetToFieldBoundary) {
    PursuitNavigationConfiguration config;
    config.boundary_margin = 0.1;
    PursuitNavigation nav(config, std::make_shared<ManualClock>());
    nav.initialize();

    RobotDescriptionsStamped robots;
    robots.descriptions.push_back(make_robot(Label::MR_STABS_MK1, 0.0, 0.0, 0.0));
    robots.descriptions.push_back(make_robot(Label::OPPONENT, kOutsideFieldX, 0.0, 0.0));

    FieldDescription field = make_field(kSmallFieldSize, kSmallFieldSize);

    VelocityCommand cmd = nav.update(robots, field, make_target(kOutsideFieldX, 0.0));

    EXPECT_TRUE(cmd.forward_mps >= 0.0 || cmd.yaw_rate_radps != 0.0);
}

TEST_F(PursuitNavigationTest, WallReverseDisabledByDefault) {
    RobotDescriptionsStamped robots;
    robots.descriptions.push_back(make_robot_with_frame(Label::MR_STABS_MK1, FrameId::OUR_ROBOT_1,
                                                        Group::OURS, kNearWallX, 0.0, 0.0));
    robots.descriptions.push_back(make_robot(Label::OPPONENT, -0.5, 0.0, M_PI));
    FieldDescription field = make_field(kSmallFieldSize, kSmallFieldSize);

    VelocityCommand cmd = nav_->update(robots, field, make_target(-0.5, 0.0));
    EXPECT_GE(cmd.forward_mps, 0.0);
}

TEST_F(PursuitNavigationTest, WallReverseEngagesWhenTooClose) {
    PursuitNavigationConfiguration config;
    config.wall_reverse_distance_m = kWallReverseDistanceM;
    PursuitNavigation nav(config, std::make_shared<ManualClock>());
    nav.initialize();

    RobotDescriptionsStamped robots;
    robots.descriptions.push_back(make_robot_with_frame(Label::MR_STABS_MK1, FrameId::OUR_ROBOT_1,
                                                        Group::OURS, kNearWallX, 0.0, 0.0));
    robots.descriptions.push_back(make_robot(Label::OPPONENT, kTargetAheadOfWallX, 0.0, 0.0));
    FieldDescription field = make_field(kSmallFieldSize, kSmallFieldSize);

    VelocityCommand cmd = nav.update(robots, field, make_target(kTargetAheadOfWallX, 0.0));

    EXPECT_LT(cmd.forward_mps, 0.0) << "wall reverse overrides a target ahead";
}

TEST_F(PursuitNavigationTest, WallReverseDoesNotEngageWhenFarFromWall) {
    PursuitNavigationConfiguration config;
    config.wall_reverse_distance_m = kWallReverseDistanceM;
    PursuitNavigation nav(config, std::make_shared<ManualClock>());
    nav.initialize();

    RobotDescriptionsStamped robots;
    robots.descriptions.push_back(make_robot_with_frame(Label::MR_STABS_MK1, FrameId::OUR_ROBOT_1,
                                                        Group::OURS, 0.0, 0.0, 0.0));
    robots.descriptions.push_back(make_robot(Label::OPPONENT, 0.5, 0.0, 0.0));
    FieldDescription field = make_field(kSmallFieldSize, kSmallFieldSize);

    VelocityCommand cmd = nav.update(robots, field, make_target(0.5, 0.0));

    EXPECT_GT(cmd.forward_mps, 0.0);
}

TEST_F(PursuitNavigationTest, LinearYIsAlwaysZero) {
    RobotDescriptionsStamped robots;
    robots.descriptions.push_back(make_robot(Label::MR_STABS_MK1, 0.0, 0.0, 0.0));
    robots.descriptions.push_back(make_robot(Label::OPPONENT, 1.0, 1.0, 0.0));
    FieldDescription field = make_field(4.0, 4.0);

    VelocityCommand cmd = nav_->update(robots, field, make_target(1.0, 1.0));

    EXPECT_DOUBLE_EQ(cmd.lateral_mps, 0.0);
}

TEST_F(PursuitNavigationTest, TurnCommitHeldNear180Deg) {
    PursuitNavigationConfiguration config = make_default_config();
    config.angular_kd = 0.0;
    config.max_angular_command = std::nullopt;
    config.enable_hysteresis = true;
    PursuitNavigation nav(config, std::make_shared<ManualClock>());
    nav.initialize();
    FieldDescription field = make_field(4.0, 4.0);
    RobotDescriptionsStamped robots;
    robots.descriptions.push_back(make_robot_with_frame(Label::MR_STABS_MK1, FrameId::OUR_ROBOT_1,
                                                        Group::OURS, 0.0, 0.0, 0.0));

    VelocityCommand first = nav.update(robots, field, make_target(-1.0, 0.01));
    VelocityCommand second = nav.update(robots, field, make_target(-1.0, -0.01));

    EXPECT_GT(first.yaw_rate_radps, 0.0);
    EXPECT_GT(second.yaw_rate_radps, 0.0) << "turn direction flipped on a tiny perturbation";
}

TEST_F(PursuitNavigationTest, SteerBrakeNeverReversesLinear) {
    PursuitNavigationConfiguration config = make_default_config();
    config.angular_kp = 10.0;
    config.angular_kd = 0.0;
    config.angle_threshold = M_PI;
    config.max_angular_command = std::nullopt;
    config.max_linear_command = std::nullopt;
    config.enable_hysteresis = false;
    PursuitNavigation nav(config, std::make_shared<ManualClock>());
    nav.initialize();
    FieldDescription field = make_field(4.0, 4.0);
    RobotDescriptionsStamped robots;
    robots.descriptions.push_back(make_robot_with_frame(Label::MR_STABS_MK1, FrameId::OUR_ROBOT_1,
                                                        Group::OURS, 0.0, 0.0, 0.0));

    VelocityCommand cmd = nav.update(robots, field, make_target(1.0, 1.0));

    ASSERT_GT(std::abs(cmd.yaw_rate_radps), 2.0);
    EXPECT_GE(cmd.forward_mps, 0.0);
}

}  // namespace auto_battlebot
