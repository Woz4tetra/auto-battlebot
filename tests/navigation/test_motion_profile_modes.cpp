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

RobotDescriptionsStamped make_our_robot(double x, double y, double yaw, double vx = 0.0,
                                        bool is_stale = false) {
    RobotDescription robot;
    robot.label = Label::MRS_BUFF_MK3;
    robot.frame_id = FrameId::OUR_ROBOT_1;
    robot.group = Group::OURS;
    robot.pose.position.x = x;
    robot.pose.position.y = y;
    robot.pose.rotation = euler_to_quaternion(0.0, 0.0, yaw);
    robot.velocity = Velocity2D{vx, 0.0, 0.0};
    robot.is_stale = is_stale;

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
                         double time_s, double vx = 0.0) {
        clock_->set(time_s);
        return nav.update(make_our_robot(0.0, 0.0, 0.0, vx), make_field(),
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

TEST_F(MotionProfileModeTest, RunAwayCutsTheCommandInsideStopDistance) {
    // Zero terminal velocity is a precise stop: at the safe point the robot holds, it does not
    // creep through and give the ground back.
    auto nav = make_nav(config_);
    const VelocityCommand cmd =
        tick(*nav, config_.stop_distance_m * 0.5, BehaviorMode::RUN_AWAY, 1.0);

    EXPECT_DOUBLE_EQ(cmd.linear_x, 0.0);
    EXPECT_DOUBLE_EQ(cmd.angular_z, 0.0);
}

TEST_F(MotionProfileModeTest, AttackKeepsDrivingInsideStopDistance) {
    // Same geometry, opposite mission. Stopping short of the opponent is the failure mode the
    // drive-through terminal velocity exists to prevent.
    auto nav = make_nav(config_);
    const VelocityCommand cmd =
        tick(*nav, config_.stop_distance_m * 0.5, BehaviorMode::ATTACK, 1.0);

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

TEST_F(MotionProfileModeTest, BrakeScheduleDoesNotSubtractALatencyLead) {
    // The motion estimator already renders our robot at now + plant.delay_s, so the distance
    // reaching the brake schedule is measured from where the robot will be when this command
    // reaches the wheels. Subtracting v*delay_s again here double-counted that travel and braked
    // early. With the speed feedback disabled the command is a pure function of the reference
    // speed, so a robot barrelling in at 2 m/s must be commanded exactly like a stationary one at
    // the same distance.
    config_.speed_kp = 0.0;
    config_.speed_ki = 0.0;

    auto stationary = make_nav(config_);
    auto moving = make_nav(config_);
    const double distance = 0.5;

    const double stationary_cmd =
        tick(*stationary, distance, BehaviorMode::RUN_AWAY, 1.0, 0.0).linear_x;
    const double moving_cmd = tick(*moving, distance, BehaviorMode::RUN_AWAY, 1.0, 2.0).linear_x;

    EXPECT_GT(stationary_cmd, 0.0);
    EXPECT_DOUBLE_EQ(moving_cmd, stationary_cmd);
}

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
        tick(*negative_nav, config_.stop_distance_m * 0.5, BehaviorMode::RUN_AWAY, 1.0);
    EXPECT_DOUBLE_EQ(cmd.linear_x, 0.0);
    EXPECT_DOUBLE_EQ(cmd.angular_z, 0.0);
}

TEST_F(MotionProfileModeTest, RunAwayTerminalSpeedFractionIsConfigurable) {
    MotionProfileNavigationConfiguration driving_config = config_;
    driving_config.run_away_terminal_speed_fraction = 0.4;  // ~2.0 m/s

    auto stopping = make_nav(config_);
    auto driving = make_nav(driving_config);
    const double distance = 0.3;

    const double stopping_cmd = tick(*stopping, distance, BehaviorMode::RUN_AWAY, 1.0).linear_x;
    const double driving_cmd = tick(*driving, distance, BehaviorMode::RUN_AWAY, 1.0).linear_x;

    EXPECT_GT(driving_cmd, stopping_cmd);
    // And the stop cut is gone, since a drive-through mission never arrives.
    EXPECT_GT(tick(*driving, config_.stop_distance_m * 0.5, BehaviorMode::RUN_AWAY, 1.05).linear_x,
              0.0);
}

TEST_F(MotionProfileModeTest, AttackTerminalSpeedFractionIsConfigurable) {
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

class MotionProfileGateTest : public MotionProfileModeTest {
   protected:
    VelocityCommand step(MotionProfileNavigation &nav, double time_s, double target_x,
                         double target_y, BehaviorMode mode, double vx = 0.0, bool stale = false,
                         const FieldDescription &field = make_field()) {
        clock_->set(time_s);
        return nav.update(make_our_robot(0.0, 0.0, 0.0, vx, stale), field,
                          make_target(target_x, target_y, mode));
    }
};

TEST_F(MotionProfileGateTest, ModeFlipDropsTheCarriedReferenceSpeed) {
    auto nav = make_nav(config_);
    for (const double t : {1.0, 1.05, 1.1}) step(*nav, t, 3.0, 0.0, BehaviorMode::ATTACK);

    const VelocityCommand cmd = step(*nav, 1.15, 0.5, 0.0, BehaviorMode::RUN_AWAY);

    EXPECT_GT(cmd.linear_x, 0.0);
    EXPECT_LT(cmd.linear_x, 0.5);
}

TEST_F(MotionProfileGateTest, MissingSpeedHoldsTheLastMeasurement) {
    config_.hazard_tangent_enable = false;
    FieldDescription field = make_field();
    field.hazards.push_back(make_hazard(0.6, 0.0, 0.1, 0.2));
    auto remembers = make_nav(config_);
    auto forgets = make_nav(config_);
    step(*remembers, 1.0, 3.0, 0.0, BehaviorMode::ATTACK, 2.0, false, field);
    step(*forgets, 1.0, 3.0, 0.0, BehaviorMode::ATTACK, 0.0, false, field);

    const VelocityCommand held =
        step(*remembers, 1.05, 3.0, 0.0, BehaviorMode::ATTACK, 0.0, true, field);
    const VelocityCommand zero =
        step(*forgets, 1.05, 3.0, 0.0, BehaviorMode::ATTACK, 0.0, true, field);

    EXPECT_LT(held.linear_x, zero.linear_x);
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

    EXPECT_DOUBLE_EQ(step(*wound, 1.2, 3.0, 0.0, BehaviorMode::ATTACK).linear_x,
                     step(*clean, 1.2, 3.0, 0.0, BehaviorMode::ATTACK).linear_x);
}

TEST_F(MotionProfileGateTest, BarrierClipsWallReverseTowardHole) {
    config_.hazard_tangent_enable = false;
    config_.wall_reverse_distance_m = 0.5;
    FieldDescription field = make_field();
    field.hazards.push_back(make_hazard(2.35, 0.0, 0.1, 0.15));
    auto nav = make_nav(config_);

    clock_->set(1.0);
    const VelocityCommand cmd = nav->update(make_our_robot(2.7, 0.0, 0.0), field,
                                            make_target(3.0, 0.0, BehaviorMode::ATTACK));

    EXPECT_LT(cmd.linear_x, 0.0);
    EXPECT_GT(cmd.linear_x, -0.5);
}

TEST_F(MotionProfileGateTest, StaleTrackGivesNoMeasuredSpeed) {
    MotionProfileNavigationConfiguration open_loop = config_;
    open_loop.speed_kp = 0.0;
    open_loop.speed_ki = 0.0;
    auto stale_nav = make_nav(config_);
    auto open_nav = make_nav(open_loop);

    EXPECT_DOUBLE_EQ(step(*stale_nav, 1.0, 0.3, 0.0, BehaviorMode::RUN_AWAY, 3.0, true).linear_x,
                     step(*open_nav, 1.0, 0.3, 0.0, BehaviorMode::RUN_AWAY, 3.0, true).linear_x);
}

TEST_F(MotionProfileGateTest, FallingReferenceIsNotRateLimited) {
    auto nav = make_nav(config_);
    for (const double t : {1.0, 1.05, 1.1}) step(*nav, t, 3.0, 0.0, BehaviorMode::RUN_AWAY);

    EXPECT_LT(step(*nav, 1.15, 0.3, 0.0, BehaviorMode::RUN_AWAY).linear_x, 0.0);
}

TEST_F(MotionProfileGateTest, StaleSpeedFreezesTheIntegrator) {
    config_.speed_ki = 2.0;
    MotionProfileNavigationConfiguration open_loop = config_;
    open_loop.speed_kp = 0.0;
    open_loop.speed_ki = 0.0;
    auto stale_nav = make_nav(config_);
    auto open_nav = make_nav(open_loop);

    double stale_cmd = 0.0;
    double open_cmd = 0.0;
    for (const double t : {1.0, 1.05, 1.1, 1.15}) {
        stale_cmd = step(*stale_nav, t, 0.3, 0.0, BehaviorMode::RUN_AWAY, 0.0, true).linear_x;
        open_cmd = step(*open_nav, t, 0.3, 0.0, BehaviorMode::RUN_AWAY, 0.0, true).linear_x;
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
        weak_cmd = step(*weak_nav, t, distance, 0.0, BehaviorMode::RUN_AWAY).linear_x;
        equal_cmd = step(*equal_nav, t, distance, 0.0, BehaviorMode::RUN_AWAY).linear_x;
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
    VelocityCommand cmd;
    for (int i = 0; i < 12; ++i) {
        cmd = step(*nav, 1.0 + 0.05 * i, 0.245, 0.08, BehaviorMode::RUN_AWAY);
    }

    EXPECT_LT(cmd.linear_x, 0.8);
    EXPECT_LT(cmd.angular_z, 0.25);
}

TEST_F(MotionProfileGateTest, GoalInsideHazardIsPushedOut) {
    config_.hazard_tangent_enable = false;
    FieldDescription field = make_field();
    field.hazards.push_back(make_hazard(0.6, 0.0, 0.2, 0.3));
    auto blocked = make_nav(config_);
    auto open = make_nav(config_);

    const VelocityCommand pushed =
        step(*blocked, 1.0, 0.6, 0.1, BehaviorMode::ATTACK, 0.0, false, field);
    const VelocityCommand direct = step(*open, 1.0, 0.6, 0.1, BehaviorMode::ATTACK);

    ASSERT_GT(direct.angular_z, 0.0);
    EXPECT_GT(pushed.angular_z, 1.5 * direct.angular_z);
}

}  // namespace
}  // namespace auto_battlebot
