#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include "enums/behavior_mode.hpp"
#include "navigation/motion_profile_navigation.hpp"
#include "plant/mrs_buff_mk3_params.hpp"
#include "time/manual_clock.hpp"
#include "transform_utils.hpp"

namespace auto_battlebot {
namespace {

// Per-mode terminal velocity: ATTACK drives through the opponent, RUN_AWAY arrives stopped at the
// safe point. The mode rides on TargetSelection, stamped by ControlLoop::resolve_target.

RobotDescriptionsStamped make_our_robot(double x, double y, double yaw) {
    RobotDescription robot;
    robot.label = Label::MRS_BUFF_MK3;
    robot.frame_id = FrameId::OUR_ROBOT_1;
    robot.group = Group::OURS;
    robot.pose.position.x = x;
    robot.pose.position.y = y;
    robot.pose.rotation = euler_to_quaternion(0.0, 0.0, yaw);
    robot.velocity = Velocity2D{0.0, 0.0, 0.0};

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

TargetSelection make_target(double x, double y, BehaviorMode mode) {
    TargetSelection target;
    target.pose.x = x;
    target.pose.y = y;
    target.label = Label::OPPONENT;
    target.mode = mode;
    return target;
}

class MotionProfileModeTest : public ::testing::Test {
   protected:
    std::unique_ptr<MotionProfileNavigation> make_nav(
        const MotionProfileNavigationConfiguration &config) {
        auto nav = std::make_unique<MotionProfileNavigation>(config, clock_);
        nav->initialize();
        return nav;
    }

    // One tick with our robot at the origin facing +x and the target straight ahead.
    VelocityCommand tick(MotionProfileNavigation &nav, double distance, BehaviorMode mode,
                         double time_s) {
        clock_->set(time_s);
        return nav.update(make_our_robot(0.0, 0.0, 0.0), make_field(),
                          make_target(distance, 0.0, mode));
    }

    // Nothing in C++ defaults to a fit any more: the plant arrives from the [plant] config
    // table, so a test config has to ask for one or it drives a zero drivetrain.
    MotionProfileNavigationConfiguration make_config() {
        MotionProfileNavigationConfiguration config;
        config.plant = mrs_buff_mk3_plant();
        return config;
    }

    MotionProfileNavigationConfiguration config_ = make_config();
    std::shared_ptr<ManualClock> clock_ = std::make_shared<ManualClock>();
};

// --- what each mode does on arrival ---

TEST_F(MotionProfileModeTest, RunAwayCutsTheCommandInsideStopDistance) {
    // Zero terminal velocity is a precise stop: at the safe point the robot holds, it does not
    // creep through and give the ground back.
    auto nav = make_nav(config_);
    const VelocityCommand cmd =
        tick(*nav, config_.stop_distance * 0.5, BehaviorMode::RUN_AWAY, 1.0);

    EXPECT_DOUBLE_EQ(cmd.linear_x, 0.0);
    EXPECT_DOUBLE_EQ(cmd.angular_z, 0.0);
}

TEST_F(MotionProfileModeTest, AttackKeepsDrivingInsideStopDistance) {
    // Same geometry, opposite mission. Stopping short of the opponent is the failure mode the
    // drive-through terminal velocity exists to prevent.
    auto nav = make_nav(config_);
    const VelocityCommand cmd = tick(*nav, config_.stop_distance * 0.5, BehaviorMode::ATTACK, 1.0);

    EXPECT_GT(cmd.linear_x, 0.0);
}

TEST_F(MotionProfileModeTest, AttackDoesNotBrakeWhereRunAwayDoes) {
    // 0.3 m is inside the brake schedule for a zero terminal velocity (0.3 / 0.176 s = 1.7 m/s)
    // but not for a full-speed one, which stays clamped at max_linear_speed_fwd.
    auto attack = make_nav(config_);
    auto run_away = make_nav(config_);
    const double distance = 0.3;

    const double attack_cmd = tick(*attack, distance, BehaviorMode::ATTACK, 1.0).linear_x;
    const double run_away_cmd = tick(*run_away, distance, BehaviorMode::RUN_AWAY, 1.0).linear_x;

    EXPECT_GT(attack_cmd, run_away_cmd);
    EXPECT_GT(run_away_cmd, 0.0);  // still approaching, just slower
}

// --- how the configured numbers are read ---

TEST_F(MotionProfileModeTest, TerminalSpeedFractionScalesWithTheTopSpeed) {
    // Half throttle at the goal has to mean half of whatever the fit says the drivetrain does, so
    // a refit rescales the mission instead of leaving a hand-copied m/s stale. Checked by halving
    // the top speed and asking for twice the fraction: the same arrival speed either way.
    MotionProfileNavigationConfiguration fast = config_;
    fast.attack_terminal_speed_fraction = 0.4;
    MotionProfileNavigationConfiguration slow = config_;
    slow.plant.k_fwd = config_.plant.k_fwd * 0.5;
    slow.attack_terminal_speed_fraction = 0.8;

    ASSERT_DOUBLE_EQ(fast.attack_terminal_speed_fraction * fast.plant.k_fwd,
                     slow.attack_terminal_speed_fraction * slow.plant.k_fwd);

    // Read through the command, which the halved top speed rescales by the same factor: the
    // feedforward divides the reference speed by the plant's k_fwd.
    auto fast_nav = make_nav(fast);
    auto slow_nav = make_nav(slow);
    const double fast_cmd = tick(*fast_nav, 0.1, BehaviorMode::ATTACK, 1.0).linear_x;
    const double slow_cmd = tick(*slow_nav, 0.1, BehaviorMode::ATTACK, 1.0).linear_x;
    EXPECT_GT(fast_cmd, 0.0);
    EXPECT_GT(slow_cmd, fast_cmd);  // same m/s asked of half the drivetrain
}

TEST_F(MotionProfileModeTest, TerminalSpeedFractionIsClampedToTheUnitInterval) {
    // Out-of-range fractions are nonsense rather than a request: above 1 the drivetrain has
    // nothing left to give, and below 0 there is no reverse arrival to ask for, since the
    // controller only ever drives toward the goal.
    MotionProfileNavigationConfiguration over = config_;
    over.attack_terminal_speed_fraction = 3.0;
    MotionProfileNavigationConfiguration full = config_;
    full.attack_terminal_speed_fraction = 1.0;

    auto over_nav = make_nav(over);
    auto full_nav = make_nav(full);
    EXPECT_DOUBLE_EQ(tick(*over_nav, 0.5, BehaviorMode::ATTACK, 1.0).linear_x,
                     tick(*full_nav, 0.5, BehaviorMode::ATTACK, 1.0).linear_x);

    MotionProfileNavigationConfiguration negative = config_;
    negative.run_away_terminal_speed_fraction = -2.0;
    auto negative_nav = make_nav(negative);
    const VelocityCommand cmd =
        tick(*negative_nav, config_.stop_distance * 0.5, BehaviorMode::RUN_AWAY, 1.0);
    EXPECT_DOUBLE_EQ(cmd.linear_x, 0.0);
    EXPECT_DOUBLE_EQ(cmd.angular_z, 0.0);
}

TEST_F(MotionProfileModeTest, RunAwayTerminalSpeedFractionIsConfigurable) {
    // Nothing pins RUN_AWAY to zero in the code; it is a config value like any other. A positive
    // one turns the safe point into a waypoint the robot drives through.
    MotionProfileNavigationConfiguration driving_config = config_;
    driving_config.run_away_terminal_speed_fraction = 0.4;  // ~2.0 m/s

    auto stopping = make_nav(config_);
    auto driving = make_nav(driving_config);
    const double distance = 0.3;

    const double stopping_cmd = tick(*stopping, distance, BehaviorMode::RUN_AWAY, 1.0).linear_x;
    const double driving_cmd = tick(*driving, distance, BehaviorMode::RUN_AWAY, 1.0).linear_x;

    EXPECT_GT(driving_cmd, stopping_cmd);
    // And the stop cut is gone, since a drive-through mission never arrives.
    EXPECT_GT(tick(*driving, config_.stop_distance * 0.5, BehaviorMode::RUN_AWAY, 1.05).linear_x,
              0.0);
}

TEST_F(MotionProfileModeTest, AttackTerminalSpeedFractionIsConfigurable) {
    // A finite attack speed is the ram-tuning knob playground/control_stage0 sweeps.
    MotionProfileNavigationConfiguration slow = config_;
    slow.attack_terminal_speed_fraction = 0.1;  // ~0.5 m/s
    MotionProfileNavigationConfiguration fast = config_;
    fast.attack_terminal_speed_fraction = 0.3;  // ~1.5 m/s

    auto slow_nav = make_nav(slow);
    auto fast_nav = make_nav(fast);
    // Close enough in that neither reference speed saturates the command; past ~0.23 m both ask
    // for more than the drivetrain has and the clamp hides the difference.
    const double distance = 0.1;

    EXPECT_GT(tick(*fast_nav, distance, BehaviorMode::ATTACK, 1.0).linear_x,
              tick(*slow_nav, distance, BehaviorMode::ATTACK, 1.0).linear_x);
}

// --- the flip itself ---

TEST_F(MotionProfileModeTest, ModeFlipResetsTheTrajectory) {
    // A switch flip is a new mission: the goal jumps from the opponent to the safe point and the
    // terminal speed changes with it. Observed through the turn latch, which holds a committed
    // turn direction across the 180 deg sign flip and must not survive into the new mission.
    //
    // Both navs see the same two ticks. The robot faces +x with the target behind it, so the
    // heading error is near 180 deg and the latch commits on tick 1. Tick 2 puts the target a
    // hair to the other side, flipping the raw error's sign.
    auto held = make_nav(config_);
    auto flipped = make_nav(config_);

    const double behind_left = M_PI - 0.05;      // +171 deg
    const double behind_right = -(M_PI - 0.05);  // -171 deg

    for (auto *nav : {held.get(), flipped.get()}) {
        clock_->set(1.0);
        nav->update(
            make_our_robot(0.0, 0.0, 0.0), make_field(),
            make_target(std::cos(behind_left), std::sin(behind_left), BehaviorMode::ATTACK));
    }

    clock_->set(1.05);
    const VelocityCommand held_cmd = held->update(
        make_our_robot(0.0, 0.0, 0.0), make_field(),
        make_target(std::cos(behind_right), std::sin(behind_right), BehaviorMode::ATTACK));
    clock_->set(1.05);
    const VelocityCommand flipped_cmd = flipped->update(
        make_our_robot(0.0, 0.0, 0.0), make_field(),
        make_target(std::cos(behind_right), std::sin(behind_right), BehaviorMode::RUN_AWAY));

    // Still in the same mission, so the latch holds the left turn it committed to.
    EXPECT_GT(held_cmd.angular_z, 0.0);
    // New mission, so the latch is clear and the turn follows the shortest way round again.
    EXPECT_LT(flipped_cmd.angular_z, 0.0);
}

}  // namespace
}  // namespace auto_battlebot
