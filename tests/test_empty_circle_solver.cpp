#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include "target_selector/empty_circle_solver.hpp"

namespace auto_battlebot {
namespace {

Size make_field(double x, double y) {
    Size field;
    field.x = x;
    field.y = y;
    return field;
}

Pose2D make_pose(double x, double y) {
    Pose2D pose;
    pose.x = x;
    pose.y = y;
    return pose;
}

constexpr double kTol = 1e-9;

FieldHazard make_hazard(double x, double y, double radius) {
    FieldHazard hazard;
    hazard.center.x = x;
    hazard.center.y = y;
    hazard.inflated_radius_m = radius;
    return hazard;
}

double brute_force_radius_with_hazards(const Size &field, const std::vector<Pose2D> &opponents,
                                       const std::vector<FieldHazard> &hazards,
                                       double resolution_m) {
    const int nx = std::max(2, static_cast<int>(std::ceil(field.x / resolution_m)) + 1);
    const int ny = std::max(2, static_cast<int>(std::ceil(field.y / resolution_m)) + 1);
    const double step_x = field.x / (nx - 1);
    const double step_y = field.y / (ny - 1);
    double best = -std::numeric_limits<double>::infinity();
    for (int iy = 0; iy < ny; ++iy) {
        const double y = -field.y / 2.0 + iy * step_y;
        for (int ix = 0; ix < nx; ++ix) {
            const double x = -field.x / 2.0 + ix * step_x;
            best = std::max(best, empty_circle_radius(field, x, y, opponents, hazards));
        }
    }
    return best;
}

double brute_force_radius(const Size &field, const std::vector<Pose2D> &opponents,
                          double resolution_m) {
    const int nx = std::max(2, static_cast<int>(std::ceil(field.x / resolution_m)) + 1);
    const int ny = std::max(2, static_cast<int>(std::ceil(field.y / resolution_m)) + 1);
    const double step_x = field.x / (nx - 1);
    const double step_y = field.y / (ny - 1);
    double best = -std::numeric_limits<double>::infinity();
    for (int iy = 0; iy < ny; ++iy) {
        const double y = -field.y / 2.0 + iy * step_y;
        for (int ix = 0; ix < nx; ++ix) {
            best = std::max(best,
                            empty_circle_radius(field, -field.x / 2.0 + ix * step_x, y, opponents));
        }
    }
    return best;
}

TEST(EmptyCircleSolverTest, NoOpponentsSquareFieldCentersAtOrigin) {
    const EmptyCircle result = solve_exact(make_field(2.44, 2.44), {});
    EXPECT_NEAR(result.radius, 1.22, kTol);
    EXPECT_NEAR(result.center.x, 0.0, kTol);
    EXPECT_NEAR(result.center.y, 0.0, kTol);
}

TEST(EmptyCircleSolverTest, NoOpponentsRectangularFieldLimitedByShortAxis) {
    constexpr double kLong = 3.0;
    constexpr double kShort = 2.0;
    const EmptyCircle result = solve_exact(make_field(kLong, kShort), {});
    EXPECT_NEAR(result.radius, kShort / 2.0, kTol);
    EXPECT_NEAR(result.center.y, 0.0, kTol);
    EXPECT_LE(std::abs(result.center.x), (kLong - kShort) / 2.0 + kTol);
}

TEST(EmptyCircleSolverTest, RectangularPlateauTieBreaksTowardPreference) {
    constexpr double kPlateauHalfLength = (3.0 - 2.0) / 2.0;
    const Size field = make_field(3.0, 2.0);
    const EmptyCircle toward_pos = solve_exact(field, {}, make_pose(1.4, 0.8));
    EXPECT_NEAR(toward_pos.radius, 1.0, kTol);
    EXPECT_NEAR(toward_pos.center.x, kPlateauHalfLength, kTol);
    const EmptyCircle toward_neg = solve_exact(field, {}, make_pose(-1.4, -0.8));
    EXPECT_NEAR(toward_neg.radius, 1.0, kTol);
    EXPECT_NEAR(toward_neg.center.x, -kPlateauHalfLength, kTol);
}

TEST(EmptyCircleSolverTest, OpponentAtCenterOfSquareFieldPushesToCorners) {
    const std::vector<Pose2D> opponents = {make_pose(0.0, 0.0)};
    const double corner_optimum_radius = 1.22 * (2.0 - std::sqrt(2.0));
    const EmptyCircle result = solve_exact(make_field(2.44, 2.44), opponents);
    EXPECT_NEAR(result.radius, corner_optimum_radius, kTol);
    EXPECT_NEAR(std::abs(result.center.x), std::abs(result.center.y), kTol);
}

TEST(EmptyCircleSolverTest, CenterOpponentCornerTieBreaksTowardPreference) {
    const std::vector<Pose2D> opponents = {make_pose(0.0, 0.0)};
    for (const double sx : {1.0, -1.0}) {
        for (const double sy : {1.0, -1.0}) {
            const EmptyCircle result =
                solve_exact(make_field(2.44, 2.44), opponents, make_pose(sx, sy));
            EXPECT_GT(result.center.x * sx, 0.0) << sx << "," << sy;
            EXPECT_GT(result.center.y * sy, 0.0) << sx << "," << sy;
        }
    }
}

TEST(EmptyCircleSolverTest, OffCenterOpponentBeatsEquidistantCase) {
    const double equidistant = 1.22 * (2.0 - std::sqrt(2.0));
    const std::vector<Pose2D> opponents = {make_pose(0.7, 0.0)};
    const EmptyCircle result = solve_exact(make_field(2.44, 2.44), opponents);
    EXPECT_GT(result.radius, equidistant + 0.05);
    EXPECT_LT(result.center.x, 0.0) << "far side from the opponent";
}

TEST(EmptyCircleSolverTest, OpponentAgainstWallDoesNotStraddleIt) {
    const std::vector<Pose2D> opponents = {make_pose(1.2, 0.0)};
    const EmptyCircle result = solve_exact(make_field(2.44, 2.44), opponents);
    const double dx = result.center.x - opponents[0].x;
    const double dy = result.center.y - opponents[0].y;
    EXPECT_LE(result.radius, std::hypot(dx, dy) + kTol);
}

TEST(EmptyCircleSolverTest, CoincidentTracksMatchSingleOpponentAnswer) {
    const Size field = make_field(2.44, 2.44);
    const EmptyCircle one = solve_exact(field, {make_pose(0.4, -0.3)});
    const EmptyCircle two = solve_exact(field, {make_pose(0.4, -0.3), make_pose(0.4, -0.3)});
    EXPECT_NEAR(one.radius, two.radius, kTol);
}

TEST(EmptyCircleSolverTest, OpponentOutsideFieldIsHarmless) {
    constexpr double kOutsideFieldX = 5.0;
    const EmptyCircle result =
        solve_exact(make_field(2.44, 2.44), {make_pose(kOutsideFieldX, 0.0)});
    EXPECT_NEAR(result.radius, 2.44 / 2.0, kTol);
}

TEST(EmptyCircleSolverTest, ExactMatchesBruteForceOnRandomConfigurations) {
    const double brute_resolution = 0.005;
    const double brute_bound = brute_resolution / std::sqrt(2.0);
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> coord(-1.4, 1.4);
    std::uniform_int_distribution<int> opponent_count(0, 3);
    const Size field = make_field(2.44, 2.44);
    for (int trial = 0; trial < 50; ++trial) {
        std::vector<Pose2D> opponents;
        const int count = opponent_count(rng);
        for (int i = 0; i < count; ++i) {
            opponents.push_back(make_pose(coord(rng), coord(rng)));
        }
        const EmptyCircle exact = solve_exact(field, opponents);
        const double brute = brute_force_radius(field, opponents, brute_resolution);
        EXPECT_GE(exact.radius, brute - kTol) << "trial " << trial;
        EXPECT_LE(exact.radius, brute + brute_bound + kTol) << "trial " << trial;
    }
}

TEST(EmptyCircleSolverTest, ReportsWinningConstraintFamily) {
    const EmptyCircle square = solve_exact(make_field(4.0, 4.0), {});
    EXPECT_TRUE(square.constraint == EmptyCircleConstraint::THREE_WALL ||
                square.constraint == EmptyCircleConstraint::FIELD_CENTER);

    const EmptyCircle pushed = solve_exact(make_field(4.0, 4.0), {make_pose(0.0, 0.0)});
    EXPECT_EQ(pushed.constraint, EmptyCircleConstraint::PERPENDICULAR_WALLS_POINT);

    const EmptyCircle degenerate = solve_exact(make_field(0.0, 0.0), {});
    EXPECT_NE(degenerate.constraint, EmptyCircleConstraint::NONE);
}

double weighted_corner_radius(double half_field, double hazard_radius) {
    return (std::sqrt(2.0) * half_field - hazard_radius) / (1.0 + std::sqrt(2.0));
}

TEST(EmptyCircleSolverTest, HazardAtCentreSolvesTheWeightedCornerCase) {
    const double h = 1.22;
    const double hazard_r = 0.3;
    const Size field = make_field(2.0 * h, 2.0 * h);
    const EmptyCircle result = solve_exact(field, {}, {make_hazard(0.0, 0.0, hazard_r)});
    const double expected = weighted_corner_radius(h, hazard_r);
    EXPECT_NEAR(result.radius, expected, 1e-9);
    EXPECT_NEAR(std::abs(result.center.x), std::abs(result.center.y), 1e-9);
    EXPECT_NEAR(std::abs(result.center.x), h - expected, 1e-9);
}

TEST(EmptyCircleSolverTest, GoalIsPushedOffAHazardSittingOnTheBestPoint) {
    const Size field = make_field(2.4, 2.4);
    const std::vector<FieldHazard> hazards = {make_hazard(0.0, 0.0, 0.4)};
    const EmptyCircle result = solve_exact(field, {}, hazards);
    const double distance_to_hazard = std::hypot(result.center.x, result.center.y);
    EXPECT_GT(distance_to_hazard, 0.4) << "goal sits inside the hazard";
    EXPECT_NEAR(result.radius, distance_to_hazard - 0.4, 1e-6);
}

TEST(EmptyCircleSolverTest, HazardCoveringTheFieldReportsNegativeRadius) {
    const Size field = make_field(2.4, 2.4);
    const std::vector<FieldHazard> hazards = {make_hazard(0.0, 0.0, 5.0)};
    const EmptyCircle result = solve_exact(field, {}, hazards);
    EXPECT_LT(result.radius, 0.0);
}

TEST(EmptyCircleSolverTest, MixedOpponentAndHazardMatchesBruteForce) {
    std::mt19937 rng(20260824);
    std::uniform_real_distribution<double> coord(-1.15, 1.15);
    std::uniform_real_distribution<double> hazard_radius(0.05, 0.45);
    std::uniform_int_distribution<int> opponent_count(0, 3);
    std::uniform_int_distribution<int> hazard_count(1, 3);
    const Size field = make_field(2.44, 2.44);
    const double brute_resolution = 0.005;
    const double brute_bound = brute_resolution / std::sqrt(2.0);

    for (int trial = 0; trial < 60; ++trial) {
        std::vector<Pose2D> opponents;
        for (int i = 0, n = opponent_count(rng); i < n; ++i) {
            opponents.push_back(make_pose(coord(rng), coord(rng)));
        }
        std::vector<FieldHazard> hazards;
        for (int i = 0, n = hazard_count(rng); i < n; ++i) {
            hazards.push_back(make_hazard(coord(rng), coord(rng), hazard_radius(rng)));
        }
        const EmptyCircle exact = solve_exact(field, opponents, hazards);
        const double brute =
            brute_force_radius_with_hazards(field, opponents, hazards, brute_resolution);
        EXPECT_GE(exact.radius, brute - kTol) << "trial " << trial;
        EXPECT_LE(exact.radius, brute + brute_bound + kTol) << "trial " << trial;
    }
}

TEST(EmptyCircleRadiusTest, NegativeOutsideAndSubtractsHazardRadius) {
    const Size field = make_field(4.0, 2.0);
    EXPECT_DOUBLE_EQ(empty_circle_radius(field, 0.0, 0.0, {}), 1.0);
    EXPECT_DOUBLE_EQ(empty_circle_radius(field, 1.5, 0.0, {}), 0.5);
    EXPECT_LT(empty_circle_radius(field, 2.5, 0.0, {}), 0.0);
    EXPECT_LT(empty_circle_radius(field, 0.0, -1.5, {}), 0.0);
    EXPECT_DOUBLE_EQ(empty_circle_radius(field, 0.0, 0.0, {}, {make_hazard(0.8, 0.0, 0.3)}), 0.5);
}

TEST(EmptyCircleSolverTest, HazardFamiliesAreNamedApart) {
    const Size field = make_field(2.44, 2.44);
    const EmptyCircle no_hazard = solve_exact(field, {make_pose(0.0, 0.0)});
    EXPECT_EQ(no_hazard.constraint, EmptyCircleConstraint::PERPENDICULAR_WALLS_POINT);

    const EmptyCircle with_hazard = solve_exact(field, {}, {make_hazard(0.0, 0.0, 0.3)});
    EXPECT_EQ(with_hazard.constraint, EmptyCircleConstraint::PERPENDICULAR_WALLS_HAZARD);
}

}  // namespace
}  // namespace auto_battlebot
