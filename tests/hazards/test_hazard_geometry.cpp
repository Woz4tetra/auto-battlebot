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
    h.radius = radius;
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

// --- hazard_speed_cap ------------------------------------------------------------------------

TEST(HazardSpeedCapTest, MatchesTheClosedFormAtHandComputedRanges) {
    // cap = max_yaw_rate * L^2 / (2 R), with the hazard dead ahead. These are the three ranges
    // the plan quotes for w_max = 7.93 rad/s and R = 0.4 m.
    const double yaw_rate = 7.93;
    const double radius = 0.4;
    const double max_speed = 100.0;  // high enough that the cap, not the ceiling, is measured
    for (const auto &[range, expected] :
         std::vector<std::pair<double, double>>{{1.0, 9.9125}, {0.5, 2.478125}, {0.3, 0.892125}}) {
        const std::vector<FieldHazard> hazards = {hazard(range, 0.0, radius)};
        const double cap =
            hazard_speed_cap(pose(0.0, 0.0, 0.0), hazards, yaw_rate, max_speed, 0.0, 0.0);
        EXPECT_NEAR(cap, expected, 1e-6) << "range " << range;
    }
}

TEST(HazardSpeedCapTest, HazardOffToOneSideDoesNotThrottle) {
    // A hazard whose disc the swept path misses entirely must not slow a clean run past it.
    const std::vector<FieldHazard> hazards = {hazard(0.5, 0.5, 0.2)};
    EXPECT_NEAR(hazard_speed_cap(pose(0.0, 0.0, 0.0), hazards, 7.93, 4.88, 0.35, 0.0), 4.88, kTol);
}

TEST(HazardSpeedCapTest, HazardBehindUsDoesNotThrottle) {
    const std::vector<FieldHazard> hazards = {hazard(-0.3, 0.0, 0.4)};
    EXPECT_NEAR(hazard_speed_cap(pose(0.0, 0.0, 0.0), hazards, 7.93, 4.88, 0.35, 0.0), 4.88, kTol);
}

TEST(HazardSpeedCapTest, GrazingHazardCapsLessThanOneDeadAhead) {
    // Scaling the required clearance by the cross-track offset is what keeps the cap out of the
    // way until a hazard is genuinely in front of us.
    const std::vector<FieldHazard> ahead = {hazard(0.4, 0.0, 0.3)};
    const std::vector<FieldHazard> grazing = {hazard(0.4, 0.25, 0.3)};
    const double cap_ahead = hazard_speed_cap(pose(0.0, 0.0, 0.0), ahead, 7.93, 100.0, 0.0, 0.0);
    const double cap_grazing =
        hazard_speed_cap(pose(0.0, 0.0, 0.0), grazing, 7.93, 100.0, 0.0, 0.0);
    EXPECT_LT(cap_ahead, cap_grazing);
}

TEST(HazardSpeedCapTest, NeverReturnsBelowTheFloor) {
    // A cap that reaches zero recreates the stop-in-time behaviour this plant cannot deliver and
    // strands the robot with the hazard in front of it.
    const std::vector<FieldHazard> hazards = {hazard(0.02, 0.0, 0.4)};
    EXPECT_NEAR(hazard_speed_cap(pose(0.0, 0.0, 0.0), hazards, 7.93, 4.88, 0.35, 0.0), 0.35, kTol);
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
    EXPECT_NEAR(avoidance.cap_speed(pose(0.0, 0.0, 0.0), field, 4.88), 4.88, kTol);
    VelocityCommand command{0.5, 0.0, 0.0};
    EXPECT_FALSE(avoidance.apply_reverse(pose(0.0, 0.0, 0.0), field, command));
}

}  // namespace
}  // namespace auto_battlebot
