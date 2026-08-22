// Behavior tests for SafestPointTarget: attack delegation, run-away targeting through the
// exact solver, held-target hysteresis, and mode transitions. Geometric ground truth for
// the solver itself lives in test_empty_circle_solver.cpp.

#include <gtest/gtest.h>

#include <cmath>
#include <optional>

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

SafestPointTarget make_selector() {
    return SafestPointTarget(SafestPointTargetConfiguration{});  // retarget_improvement_m 0.15
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
    // Medial segment endpoints and center tie at radius 1.0; nearest to us wins.
    EXPECT_NEAR(target->pose.x, 0.5, 1e-9);
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
    // Corner optimum radius, solved exactly on the diagonal.
    const double expected_radius = 1.22 * (2.0 - std::sqrt(2.0));
    EXPECT_NEAR(std::hypot(target->pose.x, target->pose.y),
                std::sqrt(2.0) * (1.22 - expected_radius), 1e-9);
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
    auto selector = make_selector();
    const FieldDescription field = make_field(2.44, 2.44);
    auto robots = our_robot_at(0.5, 0.5);
    add_opponent(robots, 0.6, 0.0);
    const auto first = selector.get_target(robots, field, BehaviorMode::RUN_AWAY);
    ASSERT_TRUE(first.has_value());

    // A 5 cm opponent shift improves the fresh optimum by well under
    // retarget_improvement_m, so the held target must not move.
    auto moved = our_robot_at(0.5, 0.5);
    add_opponent(moved, 0.55, 0.0);
    const auto second = selector.get_target(moved, field, BehaviorMode::RUN_AWAY);
    ASSERT_TRUE(second.has_value());
    EXPECT_EQ(second->pose.x, first->pose.x);
    EXPECT_EQ(second->pose.y, first->pose.y);
}

TEST(SafestPointTargetTest, RetargetsWhenCandidateBeatsHeldByThreshold) {
    auto selector = make_selector();
    const FieldDescription field = make_field(2.44, 2.44);
    auto robots = our_robot_at(0.5, 0.5);
    add_opponent(robots, 0.6, 0.0);
    const auto first = selector.get_target(robots, field, BehaviorMode::RUN_AWAY);
    ASSERT_TRUE(first.has_value());
    EXPECT_LT(first->pose.x, 0.0);  // safe spot on the far side of the opponent

    // Opponent crosses the field: the held far-side spot goes bad and the fresh optimum
    // beats it by more than retarget_improvement_m, so the target must swap sides.
    auto crossed = our_robot_at(0.5, 0.5);
    add_opponent(crossed, -0.9, 0.0);
    const auto second = selector.get_target(crossed, field, BehaviorMode::RUN_AWAY);
    ASSERT_TRUE(second.has_value());
    EXPECT_GT(second->pose.x, 0.0);
}

TEST(SafestPointTargetTest, LeavingRunAwayClearsHeldTarget) {
    auto selector = make_selector();
    const FieldDescription field = make_field(2.44, 2.44);
    auto robots = our_robot_at(0.9, 0.9);
    add_opponent(robots, 0.0, 0.0);
    const auto first = selector.get_target(robots, field, BehaviorMode::RUN_AWAY);
    ASSERT_TRUE(first.has_value());
    EXPECT_GT(first->pose.x, 0.0);

    // Bounce through ATTACK, then re-enter run-away from the opposite corner. The corners
    // tie on radius, so a stale held target would win the hysteresis and stick; getting the
    // near corner proves re-entry started fresh.
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

    // Run-away is exactly the moment to keep producing answers; only the tie-break
    // degrades without our pose.
    const auto run_away = selector.get_target(robots, field, BehaviorMode::RUN_AWAY);
    EXPECT_TRUE(run_away.has_value());
    // Attack delegates to NearestTarget, which cannot rank opponents without us.
    const auto attack = selector.get_target(robots, field, BehaviorMode::ATTACK);
    EXPECT_FALSE(attack.has_value());
}

}  // namespace
}  // namespace auto_battlebot
