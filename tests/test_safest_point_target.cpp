
#include <gtest/gtest.h>

#include <cmath>
#include <map>
#include <optional>
#include <string>

#include "diagnostics_logger/diagnostics_logger.hpp"
#include "target_selector/nearest_target.hpp"
#include "target_selector/safest_point_target.hpp"
#include "transform_utils.hpp"

namespace auto_battlebot {
namespace {

FieldDescription make_field(double x, double y) {
    FieldDescription field;
    field.size.size.x = x;
    field.size.size.y = y;
    return field;
}

RobotDescription make_robot(FrameId frame_id, Group group, double x, double y) {
    RobotDescription robot;
    robot.frame_id = frame_id;
    robot.group = group;
    robot.pose = pose2d_to_pose(Pose2D{x, y, 0.0});
    return robot;
}

RobotDescriptionsStamped our_robot_at(double x, double y) {
    RobotDescriptionsStamped robots;
    robots.descriptions.push_back(make_robot(FrameId::OUR_ROBOT_1, Group::OURS, x, y));
    return robots;
}

void add_opponent(RobotDescriptionsStamped &robots, double x, double y) {
    robots.descriptions.push_back(make_robot(FrameId::THEIR_ROBOT_1, Group::THEIRS, x, y));
}

SafestPointTarget make_selector() { return SafestPointTarget(SafestPointTargetConfiguration{}); }

SafestPointTarget make_selector_with_hysteresis(double retarget_improvement_m) {
    SafestPointTargetConfiguration config;
    config.retarget_improvement_m = retarget_improvement_m;
    return SafestPointTarget(config);
}

TEST(SafestPointTargetTest, RunAwayNoOpponentsTargetsFieldCenter) {
    auto selector = make_selector();
    const auto target =
        selector.get_target(our_robot_at(0.5, 0.5), make_field(2.44, 2.44), BehaviorMode::RUN_AWAY);
    ASSERT_TRUE(target.has_value());
    EXPECT_NEAR(target->pose.x, 0.0, 1e-9);
    EXPECT_NEAR(target->pose.y, 0.0, 1e-9);
}

TEST(SafestPointTargetTest, RunAwayRectangularFieldStaysOnOurSideOfThePlateau) {
    auto selector = make_selector();
    const auto target =
        selector.get_target(our_robot_at(1.4, 0.0), make_field(3.0, 2.0), BehaviorMode::RUN_AWAY);
    ASSERT_TRUE(target.has_value());
    constexpr double kNearestTiedOptimumX = 0.5;
    EXPECT_NEAR(target->pose.x, kNearestTiedOptimumX, 1e-9);
    EXPECT_NEAR(target->pose.y, 0.0, 1e-9);
}

TEST(SafestPointTargetTest, RunAwayCornerTiePrefersCornerNearestUs) {
    auto selector = make_selector();
    auto robots = our_robot_at(0.9, 0.9);
    add_opponent(robots, 0.0, 0.0);
    const auto target = selector.get_target(robots, make_field(2.44, 2.44), BehaviorMode::RUN_AWAY);
    ASSERT_TRUE(target.has_value());
    EXPECT_GT(target->pose.x, 0.0);
    EXPECT_GT(target->pose.y, 0.0);
    const double corner_optimum_radius = 1.22 * (2.0 - std::sqrt(2.0));
    EXPECT_NEAR(std::hypot(target->pose.x, target->pose.y),
                std::sqrt(2.0) * (1.22 - corner_optimum_radius), 1e-9);
}

TEST(SafestPointTargetTest, AttackMatchesNearestTarget) {
    auto selector = make_selector();
    auto robots = our_robot_at(0.3, -0.2);
    add_opponent(robots, 0.8, 0.4);
    robots.descriptions.push_back(make_robot(FrameId::THEIR_ROBOT_2, Group::THEIRS, -0.9, 0.1));
    const FieldDescription field = make_field(2.44, 2.44);

    const auto from_selector = selector.get_target(robots, field, BehaviorMode::ATTACK);
    NearestTarget nearest;
    const auto reference = nearest.get_target(robots, field, BehaviorMode::ATTACK);
    ASSERT_TRUE(from_selector.has_value());
    ASSERT_TRUE(reference.has_value());
    EXPECT_EQ(from_selector->pose.x, reference->pose.x);
    EXPECT_EQ(from_selector->pose.y, reference->pose.y);
}

TEST(SafestPointTargetTest, HeldTargetSurvivesSmallOpponentMove) {
    constexpr double kHysteresisM = 0.15;
    constexpr double kSmallShiftM = 0.05;
    auto selector = make_selector_with_hysteresis(kHysteresisM);
    const FieldDescription field = make_field(2.44, 2.44);
    auto robots = our_robot_at(0.5, 0.5);
    add_opponent(robots, 0.6, 0.0);
    const auto first = selector.get_target(robots, field, BehaviorMode::RUN_AWAY);
    ASSERT_TRUE(first.has_value());

    auto moved = our_robot_at(0.5, 0.5);
    add_opponent(moved, 0.6 - kSmallShiftM, 0.0);
    const auto second = selector.get_target(moved, field, BehaviorMode::RUN_AWAY);
    ASSERT_TRUE(second.has_value());
    EXPECT_EQ(second->pose.x, first->pose.x);
    EXPECT_EQ(second->pose.y, first->pose.y);
}

TEST(SafestPointTargetTest, DefaultThresholdRetargetsOnAnyImprovement) {
    auto selector = make_selector();
    const FieldDescription field = make_field(2.44, 2.44);
    auto robots = our_robot_at(0.5, 0.5);
    add_opponent(robots, 0.6, 0.0);
    const auto first = selector.get_target(robots, field, BehaviorMode::RUN_AWAY);
    ASSERT_TRUE(first.has_value());

    auto moved = our_robot_at(0.5, 0.5);
    add_opponent(moved, 0.55, 0.0);
    const auto second = selector.get_target(moved, field, BehaviorMode::RUN_AWAY);
    ASSERT_TRUE(second.has_value());
    EXPECT_NE(second->pose.x, first->pose.x);
}

TEST(SafestPointTargetTest, RetargetsWhenCandidateBeatsHeldByThreshold) {
    auto selector = make_selector_with_hysteresis(0.15);
    const FieldDescription field = make_field(2.44, 2.44);
    auto robots = our_robot_at(0.5, 0.5);
    add_opponent(robots, 0.6, 0.0);
    const auto first = selector.get_target(robots, field, BehaviorMode::RUN_AWAY);
    ASSERT_TRUE(first.has_value());
    EXPECT_LT(first->pose.x, 0.0) << "safe spot on far side of opponent";

    constexpr double kCrossedOpponentX = -0.9;
    auto crossed = our_robot_at(0.5, 0.5);
    add_opponent(crossed, kCrossedOpponentX, 0.0);
    const auto second = selector.get_target(crossed, field, BehaviorMode::RUN_AWAY);
    ASSERT_TRUE(second.has_value());
    EXPECT_GT(second->pose.x, 0.0);
}

TEST(SafestPointTargetTest, HeldTargetCoveredByHazardLoses) {
    auto selector = make_selector_with_hysteresis(0.15);
    FieldDescription field = make_field(2.44, 2.44);
    const auto robots = our_robot_at(0.9, 0.9);
    const auto first = selector.get_target(robots, field, BehaviorMode::RUN_AWAY);
    ASSERT_TRUE(first.has_value());
    EXPECT_NEAR(first->pose.x, 0.0, 1e-9);
    EXPECT_NEAR(first->pose.y, 0.0, 1e-9);

    FieldHazard hazard;
    hazard.center.x = 0.0;
    hazard.center.y = 0.0;
    hazard.inflated_radius_m = 0.4;
    field.hazards.push_back(hazard);
    const auto second = selector.get_target(robots, field, BehaviorMode::RUN_AWAY);
    ASSERT_TRUE(second.has_value());
    EXPECT_GT(std::hypot(second->pose.x, second->pose.y), hazard.inflated_radius_m)
        << "held target stayed inside the hazard";
}

TEST(SafestPointTargetTest, DegenerateFieldReturnsNoTarget) {
    auto selector = make_selector();
    const auto target =
        selector.get_target(our_robot_at(0.0, 0.0), make_field(0.0, 0.0), BehaviorMode::RUN_AWAY);
    EXPECT_FALSE(target.has_value());
}

TEST(SafestPointTargetTest, RunAwayDeclinesWhenHazardsCoverTheField) {
    auto selector = make_selector();
    FieldDescription field = make_field(2.44, 2.44);
    FieldHazard hazard;
    hazard.center.x = 0.0;
    hazard.center.y = 0.0;
    hazard.inflated_radius_m = 5.0;
    field.hazards.push_back(hazard);
    const auto target = selector.get_target(our_robot_at(0.5, 0.5), field, BehaviorMode::RUN_AWAY);
    EXPECT_FALSE(target.has_value());
}

TEST(SafestPointTargetTest, LeavingRunAwayClearsHeldTarget) {
    auto selector = make_selector();
    const FieldDescription field = make_field(2.44, 2.44);
    auto robots = our_robot_at(0.9, 0.9);
    add_opponent(robots, 0.0, 0.0);
    const auto first = selector.get_target(robots, field, BehaviorMode::RUN_AWAY);
    ASSERT_TRUE(first.has_value());
    EXPECT_GT(first->pose.x, 0.0);

    selector.get_target(robots, field, BehaviorMode::ATTACK);
    auto far_side = our_robot_at(-0.9, -0.9);
    add_opponent(far_side, 0.0, 0.0);
    const auto second = selector.get_target(far_side, field, BehaviorMode::RUN_AWAY);
    ASSERT_TRUE(second.has_value());
    EXPECT_LT(second->pose.x, 0.0);
    EXPECT_LT(second->pose.y, 0.0);
}

TEST(SafestPointTargetTest, MissingOurPoseStillAnswersRunAwayButNotAttack) {
    auto selector = make_selector();
    const FieldDescription field = make_field(2.44, 2.44);
    RobotDescriptionsStamped robots;
    add_opponent(robots, 0.6, 0.0);

    const auto run_away = selector.get_target(robots, field, BehaviorMode::RUN_AWAY);
    EXPECT_TRUE(run_away.has_value());
    const auto attack = selector.get_target(robots, field, BehaviorMode::ATTACK);
    EXPECT_FALSE(attack.has_value());
}

TEST(SafestPointTargetTest, LogsSolverSourceForTheChosenPoint) {
    auto logger = DiagnosticsLogger::get_logger("safest_point_target");
    logger->clear();

    auto selector = make_selector();
    auto robots = our_robot_at(0.9, 0.9);
    add_opponent(robots, 0.0, 0.0);
    ASSERT_TRUE(
        selector.get_target(robots, make_field(2.44, 2.44), BehaviorMode::RUN_AWAY).has_value());

    std::map<std::string, std::string> solver_values;
    for (const auto &snapshot : logger->get_snapshots()) {
        if (snapshot.subsection != "solver") continue;
        for (const auto &[key, value] : snapshot.values) {
            solver_values[key] = diagnostic_scalar_to_string(value);
        }
    }

    ASSERT_FALSE(solver_values.empty()) << "no solver subsection in the selector diagnostics";
    EXPECT_EQ(solver_values["source"], "perpendicular_walls_point");
    EXPECT_EQ(solver_values["n_opponents"], "1");
    EXPECT_EQ(solver_values["tie_break"], "1");
    EXPECT_NE(solver_values.find("radius"), solver_values.end());
    EXPECT_NE(solver_values.find("evaluations"), solver_values.end());

    logger->clear();
}

}  // namespace
}  // namespace auto_battlebot
