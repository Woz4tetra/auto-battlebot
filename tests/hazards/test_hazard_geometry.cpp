// Hand-checkable geometry for the hazard layers, plus the degenerate cases the plan calls out:
// both sides blocked, a goal wedged between overlapping discs, and a hazard that has to be
// predicted forward rather than tested where it currently sits.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "hazards/hazard_geometry.hpp"
#include "navigation/hazard_avoidance.hpp"

namespace auto_battlebot {
namespace {

constexpr double kTol = 1e-9;

Pose2D pose(double x, double y, double yaw = 0.0) {
    Pose2D p;
    p.x = x;
    p.y = y;
    p.yaw = yaw;
    return p;
}

FieldHazard hazard(double x, double y, double radius, HazardSource source = HazardSource::STATIC,
                   double vx = 0.0, double vy = 0.0) {
    FieldHazard h;
    h.center.x = x;
    h.center.y = y;
    h.inflated_radius = radius;
    // The barrier tests below reason about one radius, so the hard radius matches the keep-out
    // here; the assembler tests cover the two radii differing.
    h.hard_radius = radius;
    h.source = source;
    h.velocity.vx = vx;
    h.velocity.vy = vy;
    return h;
}

FieldDescription field_with(std::vector<FieldHazard> hazards, double size = 2.4) {
    FieldDescription field;
    field.size.size = Size{size, size, 0.0};
    field.hazards = std::move(hazards);
    return field;
}

// --- push_out_of_hazards ---------------------------------------------------------------------

TEST(HazardGeometryTest, PushOutLeavesALegalPointAlone) {
    const std::vector<FieldHazard> hazards = {hazard(0.0, 0.0, 0.3)};
    const Pose2D result = push_out_of_hazards(pose(0.8, 0.0), hazards, 1.2, 1.2);
    EXPECT_NEAR(result.x, 0.8, kTol);
    EXPECT_NEAR(result.y, 0.0, kTol);
}

TEST(HazardGeometryTest, PushOutMovesToTheNearestBoundaryPoint) {
    const std::vector<FieldHazard> hazards = {hazard(0.0, 0.0, 0.3)};
    const Pose2D result = push_out_of_hazards(pose(0.1, 0.0), hazards, 1.2, 1.2);
    EXPECT_NEAR(result.x, 0.3, kTol) << "should leave along the radius it came in on";
    EXPECT_NEAR(result.y, 0.0, kTol);
}

TEST(HazardGeometryTest, PushOutFromDeadCentreIsDeterministic) {
    // No radial direction exists here. Any answer is as good as any other, but it has to be the
    // same answer every tick or the goal jitters.
    const std::vector<FieldHazard> hazards = {hazard(0.0, 0.0, 0.3)};
    const Pose2D first = push_out_of_hazards(pose(0.0, 0.0), hazards, 1.2, 1.2);
    const Pose2D second = push_out_of_hazards(pose(0.0, 0.0), hazards, 1.2, 1.2);
    EXPECT_NEAR(first.x, second.x, kTol);
    EXPECT_NEAR(first.y, second.y, kTol);
    EXPECT_NEAR(std::hypot(first.x, first.y), 0.3, kTol);
}

TEST(HazardGeometryTest, PushOutClampsBackInsideTheField) {
    // A hazard hugging a wall would otherwise push the goal out of the arena entirely.
    const std::vector<FieldHazard> hazards = {hazard(1.15, 0.0, 0.4)};
    const Pose2D result = push_out_of_hazards(pose(1.2, 0.0), hazards, 1.1, 1.1);
    EXPECT_LE(result.x, 1.1 + kTol);
    EXPECT_GE(result.x, -1.1 - kTol);
}

TEST(HazardGeometryTest, PushOutSettlesBetweenOverlappingDiscs) {
    // Two overlapping discs can bounce a point back and forth. The routine gives up rather than
    // looping; what matters is that it terminates and stays inside the field.
    const std::vector<FieldHazard> hazards = {hazard(-0.2, 0.0, 0.35), hazard(0.2, 0.0, 0.35)};
    const Pose2D result = push_out_of_hazards(pose(0.0, 0.0), hazards, 1.1, 1.1);
    EXPECT_TRUE(std::isfinite(result.x));
    EXPECT_TRUE(std::isfinite(result.y));
    EXPECT_LE(std::abs(result.x), 1.1 + kTol);
    EXPECT_LE(std::abs(result.y), 1.1 + kTol);
}

// --- first_blocking_hazard -------------------------------------------------------------------

TEST(HazardGeometryTest, ClearRunFindsNoBlocker) {
    const std::vector<FieldHazard> hazards = {hazard(0.0, 1.0, 0.2)};
    EXPECT_FALSE(first_blocking_hazard(pose(-1.0, 0.0), pose(1.0, 0.0), hazards, 0.0).has_value());
}

TEST(HazardGeometryTest, BlockedRunFindsTheNearestBlockerFirst) {
    const std::vector<FieldHazard> hazards = {hazard(0.6, 0.0, 0.2), hazard(-0.4, 0.0, 0.2)};
    const auto blocking = first_blocking_hazard(pose(-1.0, 0.0), pose(1.0, 0.0), hazards, 0.0);
    ASSERT_TRUE(blocking.has_value());
    EXPECT_EQ(*blocking, 1u) << "the one at -0.4 is reached first";
}

TEST(HazardGeometryTest, TrackedHazardIsTestedWhereItWillBe) {
    // Currently clear, but moving into the path. Reacting to where it is now aims at where it was:
    // this is the house-bot-crossing case the prediction horizon exists for.
    const std::vector<FieldHazard> tracked = {
        hazard(0.0, 0.6, 0.2, HazardSource::TRACKED, 0.0, -2.0)};
    EXPECT_FALSE(first_blocking_hazard(pose(-1.0, 0.0), pose(1.0, 0.0), tracked, 0.0).has_value());
    EXPECT_TRUE(first_blocking_hazard(pose(-1.0, 0.0), pose(1.0, 0.0), tracked, 0.3).has_value());
}

TEST(HazardGeometryTest, StaticHazardIgnoresItsVelocityField) {
    const std::vector<FieldHazard> statics = {
        hazard(0.0, 0.6, 0.2, HazardSource::STATIC, 0.0, -2.0)};
    EXPECT_FALSE(first_blocking_hazard(pose(-1.0, 0.0), pose(1.0, 0.0), statics, 0.3).has_value());
}

// --- tangent_waypoint ------------------------------------------------------------------------

TEST(HazardGeometryTest, TangentPointSitsOnTheCircle) {
    // Start at the origin, circle of radius 1 centred at (2, 0): half-angle asin(1/2) = 30 deg,
    // tangent length sqrt(3), so the left tangency point is (1.5, sqrt(3)/2).
    const TangentWaypoint waypoint = tangent_waypoint(pose(0.0, 0.0), pose(2.0, 0.0), 1.0, 0.0, 1);
    EXPECT_FALSE(waypoint.inside);
    EXPECT_EQ(waypoint.side, 1);
    EXPECT_NEAR(waypoint.point.x, 1.5, 1e-9);
    EXPECT_NEAR(waypoint.point.y, std::sqrt(3.0) / 2.0, 1e-9);
    EXPECT_NEAR(std::hypot(waypoint.point.x - 2.0, waypoint.point.y), 1.0, 1e-9);
}

TEST(HazardGeometryTest, TangentSideFollowsTheCheaperHeadingChange) {
    const Pose2D center = pose(2.0, 0.0);
    // Pointing 20 deg right of the hazard: the right tangent is 10 deg away, the left 50.
    const TangentWaypoint right =
        tangent_waypoint(pose(0.0, 0.0), center, 1.0, -20.0 * M_PI / 180.0, 0);
    EXPECT_EQ(right.side, -1);
    const TangentWaypoint left =
        tangent_waypoint(pose(0.0, 0.0), center, 1.0, 20.0 * M_PI / 180.0, 0);
    EXPECT_EQ(left.side, 1);
}

TEST(HazardGeometryTest, CommittedSideOverridesTheCheaperChoice) {
    // The latch is what stops the robot cutting back across a hazard it was already rounding.
    const TangentWaypoint latched =
        tangent_waypoint(pose(0.0, 0.0), pose(2.0, 0.0), 1.0, -20.0 * M_PI / 180.0, 1);
    EXPECT_EQ(latched.side, 1);
    EXPECT_GT(latched.point.y, 0.0);
}

TEST(HazardGeometryTest, InsideTheCircleTheWaypointPointsStraightOut) {
    const TangentWaypoint waypoint = tangent_waypoint(pose(0.3, 0.0), pose(0.0, 0.0), 1.0, 0.0, 0);
    EXPECT_TRUE(waypoint.inside);
    // Outward from where the robot already is: it sits at +0.3, so the exit is at +1.0, not the
    // far side of the disc.
    EXPECT_NEAR(waypoint.point.x, 1.0, kTol);
    EXPECT_NEAR(waypoint.point.y, 0.0, kTol);
}

// --- HazardAvoidance layers ------------------------------------------------------------------

HazardAvoidanceSettings default_settings() {
    HazardAvoidanceSettings settings;
    settings.prediction_horizon_s = 0.0;
    return settings;
}

TEST(HazardAvoidanceTest, ClearRunKeepsTheTrueGoal) {
    HazardAvoidance avoidance(default_settings());
    const FieldDescription field = field_with({hazard(0.0, 1.0, 0.2)});
    const Pose2D goal = pose(1.0, 0.0);
    const Pose2D steered = avoidance.steer_around(pose(-1.0, 0.0, 0.0), goal, field);
    EXPECT_NEAR(steered.x, goal.x, kTol);
    EXPECT_NEAR(steered.y, goal.y, kTol);
    EXPECT_FALSE(avoidance.last_substituted());
}

TEST(HazardAvoidanceTest, BlockedRunSubstitutesAClearWaypoint) {
    HazardAvoidance avoidance(default_settings());
    const FieldDescription field = field_with({hazard(0.0, 0.0, 0.3)});
    const Pose2D steered = avoidance.steer_around(pose(-1.0, 0.0, 0.0), pose(1.0, 0.0), field);
    EXPECT_TRUE(avoidance.last_substituted());
    EXPECT_GT(std::hypot(steered.x, steered.y), 0.3) << "waypoint must be outside the keep-out";
    EXPECT_NE(avoidance.committed_side(), 0);
}

TEST(HazardAvoidanceTest, SideStaysLatchedWhileRoundingTheSameHazard) {
    HazardAvoidance avoidance(default_settings());
    const FieldDescription field = field_with({hazard(0.0, 0.0, 0.3)});
    avoidance.steer_around(pose(-1.0, 0.0, 0.0), pose(1.0, 0.0), field);
    const int first = avoidance.committed_side();
    ASSERT_NE(first, 0);
    // Approach again from a heading that would now prefer the other side. The latch has to win,
    // or the robot cuts back across the hazard it was already rounding.
    for (int i = 0; i < 5; ++i) {
        avoidance.steer_around(pose(-0.6, 0.05, -0.6), pose(1.0, 0.0), field);
    }
    EXPECT_EQ(avoidance.committed_side(), first);
}

TEST(HazardAvoidanceTest, BothSidesBlockedStillTerminatesWithAFiniteWaypoint) {
    // A gap too narrow to thread: whichever side is chosen, the other hazard blocks it. The
    // iteration cap has to end the search rather than loop, and the answer has to be usable.
    HazardAvoidance avoidance(default_settings());
    const FieldDescription field = field_with({hazard(0.0, 0.35, 0.4), hazard(0.0, -0.35, 0.4)});
    const Pose2D steered = avoidance.steer_around(pose(-1.0, 0.0, 0.0), pose(1.0, 0.0), field);
    EXPECT_TRUE(std::isfinite(steered.x));
    EXPECT_TRUE(std::isfinite(steered.y));
    EXPECT_TRUE(avoidance.last_substituted());
}

TEST(HazardAvoidanceTest, ReverseFiresOnlyWhenCloseAndPointedAtIt) {
    HazardAvoidance avoidance(default_settings());
    const FieldDescription field = field_with({hazard(0.35, 0.0, 0.3)});

    VelocityCommand pointed{0.5, 0.0, 0.0};
    EXPECT_TRUE(avoidance.apply_reverse(pose(0.0, 0.0, 0.0), field, pointed));
    EXPECT_LT(pointed.linear_x, 0.0);

    VelocityCommand turned_away{0.5, 0.0, 0.0};
    EXPECT_FALSE(avoidance.apply_reverse(pose(0.0, 0.0, M_PI), field, turned_away));
    EXPECT_NEAR(turned_away.linear_x, 0.5, kTol);

    VelocityCommand far{0.5, 0.0, 0.0};
    EXPECT_FALSE(avoidance.apply_reverse(pose(-1.0, 0.0, 0.0), field, far));
    EXPECT_NEAR(far.linear_x, 0.5, kTol);
}

TEST(HazardAvoidanceTest, ResetClearsTheLatchedSide) {
    HazardAvoidance avoidance(default_settings());
    const FieldDescription field = field_with({hazard(0.0, 0.0, 0.3)});
    avoidance.steer_around(pose(-1.0, 0.0, 0.0), pose(1.0, 0.0), field);
    ASSERT_NE(avoidance.committed_side(), 0);
    avoidance.reset();
    EXPECT_EQ(avoidance.committed_side(), 0);
}

TEST(HazardAvoidanceTest, NoHazardsIsAPassThrough) {
    // The whole feature has to be invisible on a field with nothing to avoid.
    HazardAvoidance avoidance(default_settings());
    const FieldDescription field = field_with({});
    const Pose2D goal = pose(0.7, -0.3);
    const Pose2D steered = avoidance.steer_around(pose(-1.0, 0.0, 0.4), goal, field);
    EXPECT_NEAR(steered.x, goal.x, kTol);
    EXPECT_NEAR(steered.y, goal.y, kTol);
    VelocityCommand command{0.5, 0.0, 0.0};
    EXPECT_FALSE(avoidance.limit_command(pose(0.0, 0.0, 0.0), field, 2.0, command).engaged);
    EXPECT_NEAR(command.linear_x, 0.5, kTol);
    EXPECT_FALSE(avoidance.apply_reverse(pose(0.0, 0.0, 0.0), field, command));
}

// --- the velocity barrier --------------------------------------------------------------------
//
// Round-number plant so every bound is hand-checkable: k_fwd = 5, k_rev = 4, horizon = 0.2 s,
// delay = 0.05 s. The envelope is bound = (gap - max(v_approach, 0) * delay) / horizon, and the
// command constraint is u * k * toward <= bound per hazard.

HazardAvoidanceSettings barrier_settings() {
    HazardAvoidanceSettings settings;
    settings.prediction_horizon_s = 0.0;
    settings.barrier_enable = true;
    settings.max_linear_speed_fwd = 5.0;
    settings.max_linear_speed_rev = 4.0;
    settings.barrier_horizon_s = 0.2;
    settings.barrier_delay_s = 0.05;
    return settings;
}

TEST(HazardBarrierTest, FarHazardLeavesTheCommandAlone) {
    // Gap 1.6 m: bound 8 m/s, above what full command reaches, so nothing binds.
    HazardAvoidance avoidance(barrier_settings());
    const FieldDescription field = field_with({hazard(2.0, 0.0, 0.4)});
    VelocityCommand cmd{1.0, 0.0, 0.0};
    const auto result = avoidance.limit_command(pose(0.0, 0.0, 0.0), field, 0.0, cmd);
    EXPECT_FALSE(result.engaged);
    EXPECT_NEAR(cmd.linear_x, 1.0, kTol);
}

TEST(HazardBarrierTest, NearHazardTapersThrustWithTheGap) {
    // Gap 0.4 m, stationary: bound 2 m/s, so full thrust clips to 2/5 = 0.4. This is what keeps
    // wheels from spinning up during a blocked push: the command near a hazard can never ask for
    // a speed the remaining gap cannot absorb.
    HazardAvoidance avoidance(barrier_settings());
    const FieldDescription field = field_with({hazard(0.8, 0.0, 0.4)});
    VelocityCommand cmd{1.0, 0.0, 0.0};
    const auto result = avoidance.limit_command(pose(0.0, 0.0, 0.0), field, 0.0, cmd);
    EXPECT_TRUE(result.engaged);
    EXPECT_FALSE(result.braking);
    EXPECT_NEAR(cmd.linear_x, 0.4, kTol);
    EXPECT_NEAR(result.bound_mps, 2.0, kTol);
}

TEST(HazardBarrierTest, MeasuredExcessCommandsReverseThroughAZeroCommand) {
    // The 2026-08-28 coast: angle gate holds linear at zero while the chassis carries 3 m/s at a
    // hazard. Bound (0.4 - 3*0.05)/0.2 = 1.25, excess 1.75, so the barrier commands full reverse
    // through the zeroed channel. A v_ref-side brake can never do this.
    HazardAvoidance avoidance(barrier_settings());
    const FieldDescription field = field_with({hazard(0.8, 0.0, 0.4)});
    VelocityCommand cmd{0.0, 0.0, 0.0};
    const auto result = avoidance.limit_command(pose(0.0, 0.0, 0.0), field, 3.0, cmd);
    EXPECT_TRUE(result.engaged);
    EXPECT_TRUE(result.braking);
    EXPECT_NEAR(cmd.linear_x, -1.0, kTol);
}

TEST(HazardBarrierTest, InsideTheHardRadiusReversesProportionallyToDepth) {
    // Distance 0.3 to a 0.4 hard radius: gap -0.1, bound -0.5. The excess term (1.5 * 0.5 = 0.75)
    // is the binding one, so the exit command scales with how deep the robot sits. This replaces
    // the old fixed -0.35 backstop.
    HazardAvoidance avoidance(barrier_settings());
    const FieldDescription field = field_with({hazard(0.3, 0.0, 0.4)});
    VelocityCommand cmd{0.5, 0.0, 0.0};
    const auto result = avoidance.limit_command(pose(0.0, 0.0, 0.0), field, 0.0, cmd);
    EXPECT_TRUE(result.engaged);
    EXPECT_TRUE(result.braking);
    EXPECT_NEAR(cmd.linear_x, -0.75, kTol);
    EXPECT_LT(result.bound_mps, 0.0);
}

TEST(HazardBarrierTest, ReverseTowardAHazardBehindIsLimited) {
    // The rear-clearance check the old blind reverses lacked. Hazard 0.3 m behind: bound 1.5,
    // and with toward = -1 the same rule becomes a floor: u >= 1.5 / (4 * -1) = -0.375.
    HazardAvoidance avoidance(barrier_settings());
    const FieldDescription field = field_with({hazard(-0.7, 0.0, 0.4)});
    VelocityCommand cmd{-1.0, 0.0, 0.0};
    const auto result = avoidance.limit_command(pose(0.0, 0.0, 0.0), field, 0.0, cmd);
    EXPECT_TRUE(result.engaged);
    EXPECT_NEAR(cmd.linear_x, -0.375, kTol);
}

TEST(HazardBarrierTest, SlidingRearFirstAtAHazardCommandsForwardNotReverse) {
    // The excess recovery opposes the APPROACH, not the robot's forward axis. Rear toward the
    // hazard (toward = -1) and sliding into it at 3 m/s: v_approach = 3, bound
    // (0.3 - 3*0.05)/0.2 = 0.75, excess 2.25, and the recovery lands on u_min as full FORWARD
    // drive, away. A literal reverse here would back the robot into the hole.
    HazardAvoidance avoidance(barrier_settings());
    const FieldDescription field = field_with({hazard(-0.7, 0.0, 0.4)});
    VelocityCommand cmd{0.0, 0.0, 0.0};
    const auto result = avoidance.limit_command(pose(0.0, 0.0, 0.0), field, -3.0, cmd);
    EXPECT_TRUE(result.engaged);
    EXPECT_TRUE(result.braking);
    EXPECT_NEAR(cmd.linear_x, 1.0, kTol);
}

TEST(HazardBarrierTest, SqueezedBetweenOverlappingHazardsStops) {
    // Inside both a hazard ahead and one behind: the forward limit demands reverse and the rear
    // limit demands forward. No command satisfies both, so the barrier splits the violation,
    // which here is a stop -- not a dive into either.
    HazardAvoidance avoidance(barrier_settings());
    const FieldDescription field = field_with({hazard(0.2, 0.0, 0.4), hazard(-0.2, 0.0, 0.4)});
    VelocityCommand cmd{1.0, 0.0, 0.0};
    const auto result = avoidance.limit_command(pose(0.0, 0.0, 0.0), field, 0.0, cmd);
    EXPECT_TRUE(result.engaged);
    EXPECT_NEAR(cmd.linear_x, 0.0, kTol);
}

TEST(HazardBarrierTest, PassingAbeamCostsNothing) {
    // Tangent motion has zero approach component, so rounding a hazard at speed is free. The old
    // brake's on/off across-gate sat exactly on this boundary and chattered; the projection makes
    // it continuous.
    HazardAvoidance avoidance(barrier_settings());
    const FieldDescription field = field_with({hazard(0.0, 0.45, 0.4)});
    VelocityCommand cmd{1.0, 0.0, 0.0};
    const auto result = avoidance.limit_command(pose(0.0, 0.0, 0.0), field, 3.0, cmd);
    EXPECT_FALSE(result.engaged);
    EXPECT_NEAR(cmd.linear_x, 1.0, kTol);
}

TEST(HazardBarrierTest, NoPlantFitMeansNoBarrier) {
    // Pursuit constructs the shared settings without a fit: speeds stay zero and the barrier
    // must be a pass-through, not a division by zero.
    HazardAvoidanceSettings settings;
    settings.prediction_horizon_s = 0.0;
    HazardAvoidance avoidance(settings);
    const FieldDescription field = field_with({hazard(0.5, 0.0, 0.4)});
    VelocityCommand cmd{1.0, 0.0, 0.0};
    EXPECT_FALSE(avoidance.limit_command(pose(0.0, 0.0, 0.0), field, 3.0, cmd).engaged);
    EXPECT_NEAR(cmd.linear_x, 1.0, kTol);
}

TEST(HazardBarrierTest, DisabledIsAPassThrough) {
    // The baseline sweep measures the failure mode with the barrier off; the switch has to
    // actually switch.
    HazardAvoidanceSettings settings = barrier_settings();
    settings.barrier_enable = false;
    HazardAvoidance avoidance(settings);
    const FieldDescription field = field_with({hazard(0.5, 0.0, 0.4)});
    VelocityCommand cmd{1.0, 0.0, 0.0};
    EXPECT_FALSE(avoidance.limit_command(pose(0.0, 0.0, 0.0), field, 3.0, cmd).engaged);
    EXPECT_NEAR(cmd.linear_x, 1.0, kTol);
}

// --- the display contract -------------------------------------------------------------------

TEST(HazardDisplayTest, ATrackedRingIsBothRobotCirclesPlusTheMargin) {
    // What the UI draws has to add up to what the assembler inflates by, or the picture teaches
    // the wrong thing. The overlay draws half_diagonal(size) per robot; a tracked ring is the
    // hazard's circle plus ours plus the margin.
    const Size house_bot{0.254, 0.3302, 0.1};
    const Size ours{0.2225, 0.2535, 0.1};
    const double margin = 0.05;

    const double drawn_house_bot = half_diagonal(house_bot);
    const double drawn_ours = half_diagonal(ours);
    const double ring = drawn_house_bot + drawn_ours + margin;

    EXPECT_NEAR(drawn_house_bot, 0.2083, 1e-4);
    EXPECT_NEAR(drawn_ours, 0.1686, 1e-4);
    EXPECT_NEAR(ring, 0.4269, 1e-4);
    // The ring reads as roughly double the hazard's own circle, which is the thing that looked
    // wrong on screen and is not.
    EXPECT_NEAR(ring / drawn_house_bot, 2.05, 0.05);
}

TEST(HazardDisplayTest, AKnownSizeShrinksWhatALooseBoxWouldContribute) {
    // The house bot's real footprint against a badly loose detection. Every consumer reads `size`,
    // so overriding it is what keeps the keep-out and the drawn circle honest at once.
    const Size loose{0.60, 0.60, 0.20};
    const Size known{0.254, 0.3302, 0.2309};
    EXPECT_NEAR(half_diagonal(known), 0.2083, 1e-4);
    EXPECT_GT(half_diagonal(loose), 2.0 * half_diagonal(known));
}

}  // namespace
}  // namespace auto_battlebot
