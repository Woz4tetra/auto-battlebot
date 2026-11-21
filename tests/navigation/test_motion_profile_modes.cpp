#include <gtest/gtest.h>

#include <cmath>
#include <map>
#include <memory>

#include "command_shaper/plant_command_shaper.hpp"
#include "enums/behavior_mode.hpp"
#include "navigation/motion_profile_navigation.hpp"
#include "plant/mrs_buff_mk3_params.hpp"
#include "time/manual_clock.hpp"
#include "transform_utils.hpp"

namespace auto_battlebot {
namespace {

RobotDescriptionsStamped make_our_robot(double x, double y, double yaw, double vx = 0.0) {
    RobotDescription robot;
    robot.label = Label::MRS_BUFF_MK3;
    robot.frame_id = FrameId::OUR_ROBOT_1;
    robot.group = Group::OURS;
    robot.pose.position.x = x;
    robot.pose.position.y = y;
    robot.pose.rotation = euler_to_quaternion(0.0, 0.0, yaw);
    robot.velocity = Velocity2D{vx, 0.0, 0.0};

    RobotDescriptionsStamped robots;
    robots.descriptions.push_back(robot);
    return robots;
}

FieldDescription make_field() {
    FieldDescription field;
    field.size.size.x = 6.0;
    field.size.size.y = 6.0;
    return field;
}

FieldHazard make_hazard(double x, double y, double hard_radius, double inflated_radius) {
    FieldHazard hazard;
    hazard.center.x = x;
    hazard.center.y = y;
    hazard.hard_radius = hard_radius;
    hazard.inflated_radius_m = inflated_radius;
    hazard.object_radius = hard_radius;
    return hazard;
}

TargetSelection make_target(double x, double y, BehaviorMode mode) {
    TargetSelection target;
    target.pose.x = x;
    target.pose.y = y;
    target.label = Label::OPPONENT;
    target.mode = mode;
    return target;
}

constexpr double kDriveThroughFraction = 0.4;

class MotionProfileModeTest : public ::testing::Test {
   protected:
    std::unique_ptr<MotionProfileNavigation> make_nav(
        const MotionProfileNavigationConfiguration &config) {
        auto nav = std::make_unique<MotionProfileNavigation>(config, clock_);
        nav->initialize();
        plants_[nav.get()] = config.plant;
        return nav;
    }

    StickCommand update_sticks(MotionProfileNavigation &nav, RobotDescriptionsStamped robots,
                               const FieldDescription &field, const TargetSelection &target) {
        return plant_body_velocity_to_stick(nav.update(std::move(robots), field, target),
                                            plants_.at(&nav));
    }

    StickCommand tick_target_straight_ahead(MotionProfileNavigation &nav, double distance,
                                            BehaviorMode mode, double time_s, double vx = 0.0) {
        clock_->set(time_s);
        return update_sticks(nav, make_our_robot(0.0, 0.0, 0.0, vx), make_field(),
                             make_target(distance, 0.0, mode));
    }

    MotionProfileNavigationConfiguration make_config_with_plant_fit() {
        MotionProfileNavigationConfiguration config;
        config.plant = mrs_buff_mk3_plant();
        return config;
    }

    MotionProfileNavigationConfiguration config_ = make_config_with_plant_fit();
    std::shared_ptr<ManualClock> clock_ = std::make_shared<ManualClock>();
    std::map<const MotionProfileNavigation *, JigPlantParams> plants_;
};

TEST_F(MotionProfileModeTest, RunAwayCutsTheCommandInsideStopDistance) {
    auto nav = make_nav(config_);
    const StickCommand cmd = tick_target_straight_ahead(*nav, config_.stop_distance_m * 0.5,
                                                        BehaviorMode::RUN_AWAY, 1.0);

    EXPECT_DOUBLE_EQ(cmd.forward, 0.0);
    EXPECT_DOUBLE_EQ(cmd.yaw, 0.0);
}

TEST_F(MotionProfileModeTest, AttackKeepsDrivingInsideStopDistance) {
    auto nav = make_nav(config_);
    const StickCommand cmd =
        tick_target_straight_ahead(*nav, config_.stop_distance_m * 0.5, BehaviorMode::ATTACK, 1.0);

    EXPECT_GT(cmd.forward, 0.0);
}

TEST_F(MotionProfileModeTest, AttackDoesNotBrakeWhereRunAwayDoes) {
    auto attack = make_nav(config_);
    auto run_away = make_nav(config_);
    const double distance = 0.3;

    const double attack_cmd =
        tick_target_straight_ahead(*attack, distance, BehaviorMode::ATTACK, 1.0).forward;
    const double run_away_cmd =
        tick_target_straight_ahead(*run_away, distance, BehaviorMode::RUN_AWAY, 1.0).forward;

    EXPECT_GT(attack_cmd, run_away_cmd);
    EXPECT_GT(run_away_cmd, 0.0) << "run away still approaches the safe point, just slower";
}

TEST_F(MotionProfileModeTest, BrakeScheduleDoesNotSubtractALatencyLead) {
    config_.speed_kp = 0.0;
    config_.speed_ki = 0.0;

    auto stationary = make_nav(config_);
    auto moving = make_nav(config_);
    const double distance = 0.5;

    const double stationary_cmd =
        tick_target_straight_ahead(*stationary, distance, BehaviorMode::RUN_AWAY, 1.0, 0.0).forward;
    const double moving_cmd =
        tick_target_straight_ahead(*moving, distance, BehaviorMode::RUN_AWAY, 1.0, 2.0).forward;

    EXPECT_GT(stationary_cmd, 0.0);
    EXPECT_DOUBLE_EQ(moving_cmd, stationary_cmd);
}

TEST_F(MotionProfileModeTest, TerminalSpeedFractionScalesWithTheTopSpeed) {
    MotionProfileNavigationConfiguration fast = config_;
    fast.attack_terminal_speed_fraction = 0.4;
    MotionProfileNavigationConfiguration slow = config_;
    slow.plant.k_fwd = config_.plant.k_fwd * 0.5;
    slow.attack_terminal_speed_fraction = 0.8;

    ASSERT_DOUBLE_EQ(fast.attack_terminal_speed_fraction * fast.plant.k_fwd,
                     slow.attack_terminal_speed_fraction * slow.plant.k_fwd);

    auto fast_nav = make_nav(fast);
    auto slow_nav = make_nav(slow);
    const double fast_cmd =
        tick_target_straight_ahead(*fast_nav, 0.1, BehaviorMode::ATTACK, 1.0).forward;
    const double slow_cmd =
        tick_target_straight_ahead(*slow_nav, 0.1, BehaviorMode::ATTACK, 1.0).forward;
    EXPECT_GT(fast_cmd, 0.0);
    EXPECT_GT(slow_cmd, fast_cmd) << "the same m/s asks half the drivetrain";
}

TEST_F(MotionProfileModeTest, TerminalSpeedFractionIsClampedToTheUnitInterval) {
    MotionProfileNavigationConfiguration over = config_;
    over.attack_terminal_speed_fraction = 3.0;
    MotionProfileNavigationConfiguration full = config_;
    full.attack_terminal_speed_fraction = 1.0;

    auto over_nav = make_nav(over);
    auto full_nav = make_nav(full);
    EXPECT_DOUBLE_EQ(tick_target_straight_ahead(*over_nav, 0.5, BehaviorMode::ATTACK, 1.0).forward,
                     tick_target_straight_ahead(*full_nav, 0.5, BehaviorMode::ATTACK, 1.0).forward);

    MotionProfileNavigationConfiguration negative = config_;
    negative.run_away_terminal_speed_fraction = -2.0;
    auto negative_nav = make_nav(negative);
    const StickCommand cmd = tick_target_straight_ahead(
        *negative_nav, config_.stop_distance_m * 0.5, BehaviorMode::RUN_AWAY, 1.0);
    EXPECT_DOUBLE_EQ(cmd.forward, 0.0);
    EXPECT_DOUBLE_EQ(cmd.yaw, 0.0);
}

TEST_F(MotionProfileModeTest, RunAwayTerminalSpeedFractionIsConfigurable) {
    MotionProfileNavigationConfiguration driving_config = config_;
    driving_config.run_away_terminal_speed_fraction = kDriveThroughFraction;

    auto stopping = make_nav(config_);
    auto driving = make_nav(driving_config);
    const double distance = 0.3;

    const double stopping_cmd =
        tick_target_straight_ahead(*stopping, distance, BehaviorMode::RUN_AWAY, 1.0).forward;
    const double driving_cmd =
        tick_target_straight_ahead(*driving, distance, BehaviorMode::RUN_AWAY, 1.0).forward;

    EXPECT_GT(driving_cmd, stopping_cmd);
}

TEST_F(MotionProfileModeTest, DriveThroughRunAwayHasNoStopCut) {
    MotionProfileNavigationConfiguration driving_config = config_;
    driving_config.run_away_terminal_speed_fraction = kDriveThroughFraction;
    auto driving = make_nav(driving_config);
    const double distance = 0.3;

    tick_target_straight_ahead(*driving, distance, BehaviorMode::RUN_AWAY, 1.0);
    EXPECT_GT(tick_target_straight_ahead(*driving, config_.stop_distance_m * 0.5,
                                         BehaviorMode::RUN_AWAY, 1.05)
                  .forward,
              0.0);
}

TEST_F(MotionProfileModeTest, AttackTerminalSpeedFractionIsConfigurable) {
    MotionProfileNavigationConfiguration slow = config_;
    constexpr double kSlowRamFraction = 0.1;
    slow.attack_terminal_speed_fraction = kSlowRamFraction;
    MotionProfileNavigationConfiguration fast = config_;
    constexpr double kFastRamFraction = 0.3;
    fast.attack_terminal_speed_fraction = kFastRamFraction;

    auto slow_nav = make_nav(slow);
    auto fast_nav = make_nav(fast);
    const double kUnsaturatedDistance = 0.1;

    EXPECT_GT(tick_target_straight_ahead(*fast_nav, kUnsaturatedDistance, BehaviorMode::ATTACK, 1.0)
                  .forward,
              tick_target_straight_ahead(*slow_nav, kUnsaturatedDistance, BehaviorMode::ATTACK, 1.0)
                  .forward);
}

TEST_F(MotionProfileModeTest, ModeFlipResetsTheTrajectory) {
    auto held = make_nav(config_);
    auto flipped = make_nav(config_);

    constexpr double kJustShortOfBehind = M_PI - 0.05;
    const double behind_left = kJustShortOfBehind;
    const double behind_right = -kJustShortOfBehind;

    for (auto *nav : {held.get(), flipped.get()}) {
        clock_->set(1.0);
        nav->update(
            make_our_robot(0.0, 0.0, 0.0), make_field(),
            make_target(std::cos(behind_left), std::sin(behind_left), BehaviorMode::ATTACK));
    }

    clock_->set(1.05);
    const StickCommand held_cmd = update_sticks(
        *held, make_our_robot(0.0, 0.0, 0.0), make_field(),
        make_target(std::cos(behind_right), std::sin(behind_right), BehaviorMode::ATTACK));
    clock_->set(1.05);
    const StickCommand flipped_cmd = update_sticks(
        *flipped, make_our_robot(0.0, 0.0, 0.0), make_field(),
        make_target(std::cos(behind_right), std::sin(behind_right), BehaviorMode::RUN_AWAY));

    EXPECT_GT(held_cmd.yaw, 0.0);
    EXPECT_LT(flipped_cmd.yaw, 0.0);
}

class MotionProfileGateTest : public MotionProfileModeTest {
   protected:
    static constexpr double kCoastingTowardHazardMps = 3.0;
    StickCommand step(MotionProfileNavigation &nav, double time_s, double target_x, double target_y,
                      BehaviorMode mode, double vx = 0.0, bool our_robot_missing = false,
                      const FieldDescription &field = make_field()) {
        clock_->set(time_s);
        return update_sticks(
            nav, our_robot_missing ? RobotDescriptionsStamped{} : make_our_robot(0.0, 0.0, 0.0, vx),
            field, make_target(target_x, target_y, mode));
    }
};

TEST_F(MotionProfileGateTest, ModeFlipDropsTheCarriedReferenceSpeed) {
    auto nav = make_nav(config_);
    for (const double t : {1.0, 1.05, 1.1}) step(*nav, t, 3.0, 0.0, BehaviorMode::ATTACK);

    const StickCommand cmd = step(*nav, 1.15, 0.5, 0.0, BehaviorMode::RUN_AWAY);

    EXPECT_GT(cmd.forward, 0.0);
    EXPECT_LT(cmd.forward, 0.5);
}

TEST_F(MotionProfileGateTest, MissingSpeedHoldsTheLastMeasurement) {
    config_.hazard_tangent_enable = false;
    FieldDescription field = make_field();
    field.hazards.push_back(make_hazard(0.6, 0.0, 0.1, 0.2));
    auto remembers = make_nav(config_);
    auto forgets = make_nav(config_);
    step(*remembers, 1.0, 3.0, 0.0, BehaviorMode::ATTACK, 2.0, false, field);
    step(*forgets, 1.0, 3.0, 0.0, BehaviorMode::ATTACK, 0.0, false, field);

    const StickCommand held =
        step(*remembers, 1.05, 3.0, 0.0, BehaviorMode::ATTACK, 0.0, true, field);
    const StickCommand zero =
        step(*forgets, 1.05, 3.0, 0.0, BehaviorMode::ATTACK, 0.0, true, field);

    EXPECT_LT(held.forward, zero.forward);
}

TEST_F(MotionProfileGateTest, HeadingGateBleedsLinearState) {
    config_.angular_kp = 0.0;
    config_.angular_kd = 0.0;
    config_.speed_ki = 2.0;
    auto wound = make_nav(config_);
    auto clean = make_nav(config_);
    for (const double t : {1.0, 1.05, 1.1}) step(*wound, t, 3.0, 0.0, BehaviorMode::ATTACK);
    step(*wound, 1.15, -1.0, 3.0, BehaviorMode::ATTACK);
    step(*clean, 1.15, -1.0, 3.0, BehaviorMode::ATTACK);

    EXPECT_DOUBLE_EQ(step(*wound, 1.2, 3.0, 0.0, BehaviorMode::ATTACK).forward,
                     step(*clean, 1.2, 3.0, 0.0, BehaviorMode::ATTACK).forward);
}

TEST_F(MotionProfileGateTest, BarrierClipsWallReverseTowardHole) {
    config_.hazard_tangent_enable = false;
    config_.wall_reverse_distance_m = 0.5;
    FieldDescription field = make_field();
    field.hazards.push_back(make_hazard(2.35, 0.0, 0.1, 0.15));
    auto nav = make_nav(config_);

    clock_->set(1.0);
    const StickCommand cmd = update_sticks(*nav, make_our_robot(2.7, 0.0, 0.0), field,
                                           make_target(3.0, 0.0, BehaviorMode::ATTACK));

    EXPECT_LT(cmd.forward, 0.0);
    EXPECT_GT(cmd.forward, -0.5);
}

TEST_F(MotionProfileGateTest, BarrierBrakesEvenWhenTheAngleGateZeroesTheSpeedChannel) {
    config_.hazard_tangent_enable = false;
    FieldDescription field = make_field();
    field.hazards.push_back(make_hazard(0.6, 0.0, 0.1, 0.15));
    auto nav = make_nav(config_);
    const StickCommand cmd =
        step(*nav, 1.0, -3.0, 0.0, BehaviorMode::ATTACK, kCoastingTowardHazardMps, false, field);
    EXPECT_LT(cmd.forward, 0.0) << "the brake was zeroed by the angle gate, so the robot coasts "
                                   "into the hazard";
}

TEST_F(MotionProfileGateTest, MissingTrackGivesNoMeasuredSpeed) {
    MotionProfileNavigationConfiguration open_loop = config_;
    open_loop.speed_kp = 0.0;
    open_loop.speed_ki = 0.0;
    auto stale_nav = make_nav(config_);
    auto open_nav = make_nav(open_loop);
    step(*stale_nav, 1.0, 0.3, 0.0, BehaviorMode::RUN_AWAY);
    step(*open_nav, 1.0, 0.3, 0.0, BehaviorMode::RUN_AWAY);

    EXPECT_DOUBLE_EQ(step(*stale_nav, 1.0, 0.3, 0.0, BehaviorMode::RUN_AWAY, 3.0, true).forward,
                     step(*open_nav, 1.0, 0.3, 0.0, BehaviorMode::RUN_AWAY, 3.0, true).forward);
}

TEST_F(MotionProfileGateTest, FallingReferenceIsNotRateLimited) {
    auto nav = make_nav(config_);
    for (const double t : {1.0, 1.05, 1.1}) step(*nav, t, 3.0, 0.0, BehaviorMode::RUN_AWAY);

    EXPECT_LT(step(*nav, 1.15, 0.3, 0.0, BehaviorMode::RUN_AWAY).forward, 0.0);
}

TEST_F(MotionProfileGateTest, MissingSpeedFreezesTheIntegrator) {
    config_.speed_ki = 2.0;
    MotionProfileNavigationConfiguration open_loop = config_;
    open_loop.speed_kp = 0.0;
    open_loop.speed_ki = 0.0;
    auto stale_nav = make_nav(config_);
    auto open_nav = make_nav(open_loop);
    step(*stale_nav, 1.0, 0.3, 0.0, BehaviorMode::RUN_AWAY);
    step(*open_nav, 1.0, 0.3, 0.0, BehaviorMode::RUN_AWAY);

    double stale_cmd = 0.0;
    double open_cmd = 0.0;
    for (const double t : {1.0, 1.05, 1.1, 1.15}) {
        stale_cmd = step(*stale_nav, t, 0.3, 0.0, BehaviorMode::RUN_AWAY, 0.0, true).forward;
        open_cmd = step(*open_nav, t, 0.3, 0.0, BehaviorMode::RUN_AWAY, 0.0, true).forward;
    }

    EXPECT_DOUBLE_EQ(stale_cmd, open_cmd);
}

TEST_F(MotionProfileGateTest, ReverseCommandRescaledByReverseGain) {
    MotionProfileNavigationConfiguration weak = config_;
    weak.plant.k_rev = config_.plant.k_fwd * 0.8;
    MotionProfileNavigationConfiguration equal = config_;
    equal.plant.k_rev = config_.plant.k_fwd;
    auto weak_nav = make_nav(weak);
    auto equal_nav = make_nav(equal);
    double weak_cmd = 0.0;
    double equal_cmd = 0.0;
    double t = 1.0;
    for (const double distance : {3.0, 3.0, 3.0, 0.4}) {
        t += 0.05;
        weak_cmd = step(*weak_nav, t, distance, 0.0, BehaviorMode::RUN_AWAY).forward;
        equal_cmd = step(*equal_nav, t, distance, 0.0, BehaviorMode::RUN_AWAY).forward;
    }

    ASSERT_LT(equal_cmd, 0.0);
    ASSERT_GT(equal_cmd, -0.8);
    EXPECT_NEAR(weak_cmd / equal_cmd, 1.25, 1e-9);
}

TEST_F(MotionProfileGateTest, CouplingTermsReadUncompensatedCommands) {
    config_.compensate_angular_droop = true;
    config_.angle_threshold = 3.0;
    config_.angular_kp = 20.0;
    config_.max_yaw_rate = 0.0;
    config_.max_angular_command = std::nullopt;
    config_.max_linear_command = std::nullopt;
    config_.accel_limit = 0.0;
    config_.speed_kp = 0.0;
    auto nav = make_nav(config_);
    StickCommand cmd;
    for (int i = 0; i < 12; ++i) {
        cmd = step(*nav, 1.0 + 0.05 * i, 0.245, 0.08, BehaviorMode::RUN_AWAY);
    }

    EXPECT_LT(cmd.forward, 0.8);
    EXPECT_LT(cmd.yaw, 0.25);
}

TEST_F(MotionProfileGateTest, GoalInsideHazardIsPushedOut) {
    config_.hazard_tangent_enable = false;
    FieldDescription field = make_field();
    field.hazards.push_back(make_hazard(0.6, 0.0, 0.2, 0.3));
    auto blocked = make_nav(config_);
    auto open = make_nav(config_);

    const StickCommand pushed =
        step(*blocked, 1.0, 0.6, 0.1, BehaviorMode::ATTACK, 0.0, false, field);
    const StickCommand direct = step(*open, 1.0, 0.6, 0.1, BehaviorMode::ATTACK);

    ASSERT_GT(direct.yaw, 0.0);
    EXPECT_GT(pushed.yaw, 1.5 * direct.yaw);
}

}  // namespace
}  // namespace auto_battlebot
