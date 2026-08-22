// Geometric cases with hand-checkable answers for the largest-empty-circle solvers, plus an
// exact-vs-brute-force agreement sweep. Assertions are on radius, not just center: a
// center-only check passes for the wrong reason when the clearance terms are wrong in
// compensating ways. These mirror the cases the solver experiment validates on recorded
// fights, so they regression-test code that has been measured.

#include <gtest/gtest.h>

#include <cmath>
#include <random>
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

constexpr double kGridResolution = 0.15;
// Best grid node radius is within resolution / sqrt(2) of the optimum (1-Lipschitz).
const double kGridBound = kGridResolution / std::sqrt(2.0);
constexpr double kBnbTolerance = 0.01;
constexpr double kExactTol = 1e-9;

struct Solver {
    const char *name;
    double radius_bound;  // how far below the true optimum the reported radius may fall
    EmptyCircle (*solve)(const Size &, const std::vector<Pose2D> &);
};

EmptyCircle run_grid(const Size &field, const std::vector<Pose2D> &opponents) {
    return solve_coarse_grid(field, opponents, kGridResolution);
}
EmptyCircle run_exact(const Size &field, const std::vector<Pose2D> &opponents) {
    return solve_exact(field, opponents);
}
EmptyCircle run_bnb(const Size &field, const std::vector<Pose2D> &opponents) {
    return solve_branch_and_bound(field, opponents, kBnbTolerance);
}

const Solver kSolvers[] = {
    {"grid", kGridBound, run_grid},
    {"exact", kExactTol, run_exact},
    {"bnb", kBnbTolerance, run_bnb},
};

TEST(EmptyCircleSolverTest, NoOpponentsSquareFieldCentersAtOrigin) {
    const Size field = make_field(2.44, 2.44);
    for (const auto &solver : kSolvers) {
        const EmptyCircle result = solver.solve(field, {});
        EXPECT_NEAR(result.radius, 1.22, solver.radius_bound) << solver.name;
        EXPECT_NEAR(result.center.x, 0.0, solver.radius_bound) << solver.name;
        EXPECT_NEAR(result.center.y, 0.0, solver.radius_bound) << solver.name;
    }
}

TEST(EmptyCircleSolverTest, NoOpponentsRectangularFieldLimitedByShortAxis) {
    // Catches an implementation that assumes a square field: radius is the short half
    // extent and the center sits on the long-axis medial segment, not pinned to origin.
    const Size field = make_field(3.0, 2.0);
    for (const auto &solver : kSolvers) {
        const EmptyCircle result = solver.solve(field, {});
        EXPECT_NEAR(result.radius, 1.0, solver.radius_bound) << solver.name;
        EXPECT_NEAR(result.center.y, 0.0, solver.radius_bound) << solver.name;
        EXPECT_LE(std::abs(result.center.x), 0.5 + solver.radius_bound) << solver.name;
    }
}

TEST(EmptyCircleSolverTest, OpponentAtCenterOfSquareFieldPushesToCorners) {
    // Four corners tie. On the diagonal at distance t from center: wall clearance
    // half - t / sqrt(2)... solved exactly, r = half * (2 - sqrt(2)).
    const Size field = make_field(2.44, 2.44);
    const std::vector<Pose2D> opponents = {make_pose(0.0, 0.0)};
    const double expected = 1.22 * (2.0 - std::sqrt(2.0));
    for (const auto &solver : kSolvers) {
        const EmptyCircle result = solver.solve(field, opponents);
        EXPECT_NEAR(result.radius, expected, solver.radius_bound) << solver.name;
        // Winner is corner-symmetric; check the center is on a diagonal.
        EXPECT_NEAR(std::abs(result.center.x), std::abs(result.center.y), 2.0 * solver.radius_bound)
            << solver.name;
    }
}

TEST(EmptyCircleSolverTest, OffCenterOpponentBeatsEquidistantCase) {
    const Size field = make_field(2.44, 2.44);
    const double equidistant = 1.22 * (2.0 - std::sqrt(2.0));
    const std::vector<Pose2D> opponents = {make_pose(0.7, 0.0)};
    for (const auto &solver : kSolvers) {
        const EmptyCircle result = solver.solve(field, opponents);
        EXPECT_GT(result.radius, equidistant + 0.05) << solver.name;
        EXPECT_LT(result.center.x, 0.0) << solver.name;  // far side from the opponent
    }
}

TEST(EmptyCircleSolverTest, OpponentAgainstWallDoesNotStraddleIt) {
    const Size field = make_field(2.44, 2.44);
    const std::vector<Pose2D> opponents = {make_pose(1.2, 0.0)};
    for (const auto &solver : kSolvers) {
        const EmptyCircle result = solver.solve(field, opponents);
        const double dx = result.center.x - opponents[0].x;
        const double dy = result.center.y - opponents[0].y;
        EXPECT_LE(result.radius, std::hypot(dx, dy) + kExactTol) << solver.name;
    }
}

TEST(EmptyCircleSolverTest, CoincidentTracksMatchSingleOpponentAnswer) {
    // Two filter tracks landing on one robot must not perturb the exact solver.
    const Size field = make_field(2.44, 2.44);
    const std::vector<Pose2D> one = {make_pose(0.4, -0.3)};
    const std::vector<Pose2D> two = {make_pose(0.4, -0.3), make_pose(0.4, -0.3)};
    for (const auto &solver : kSolvers) {
        const EmptyCircle a = solver.solve(field, one);
        const EmptyCircle b = solver.solve(field, two);
        EXPECT_NEAR(a.radius, b.radius, kExactTol) << solver.name;
    }
}

TEST(EmptyCircleSolverTest, OpponentOutsideFieldIsHarmless) {
    const Size field = make_field(2.44, 2.44);
    const std::vector<Pose2D> opponents = {make_pose(5.0, 0.0)};
    for (const auto &solver : kSolvers) {
        const EmptyCircle result = solver.solve(field, opponents);
        // Too far away to constrain anything: same answer as the empty field.
        EXPECT_NEAR(result.radius, 1.22, solver.radius_bound) << solver.name;
    }
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
        const EmptyCircle brute = solve_brute_force(field, opponents, brute_resolution);
        // Exact must never fall below the sampled optimum, and the sample can only trail
        // the true optimum by its grid bound.
        EXPECT_GE(exact.radius, brute.radius - kExactTol) << "trial " << trial;
        EXPECT_LE(exact.radius, brute.radius + brute_bound + kExactTol) << "trial " << trial;
    }
}

}  // namespace
}  // namespace auto_battlebot
