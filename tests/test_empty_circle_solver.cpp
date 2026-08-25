// Geometric cases with hand-checkable answers for the exact largest-empty-circle solver,
// plus a brute-force agreement sweep. Assertions are on radius, not just center: a
// center-only check passes for the wrong reason when the clearance terms are wrong in
// compensating ways. These mirror the cases the solver experiment validated on recorded
// fights, so they regression-test code that has been measured.

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
    hazard.inflated_radius = radius;
    return hazard;
}

/** Brute-force sweep with hazards, same 1-Lipschitz argument as the opponent-only version:
 *  distance-to-centre minus a constant is still 1-Lipschitz. */
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

/** Sampled ground truth for the agreement sweep: best radius over a uniform grid. The
 *  radius function is 1-Lipschitz, so the best node trails the true optimum by at most
 *  resolution / sqrt(2). Lives here rather than in the library because only this test
 *  needs it. */
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
    // Catches an implementation that assumes a square field: radius is the short half
    // extent and the center sits on the long-axis medial segment.
    const EmptyCircle result = solve_exact(make_field(3.0, 2.0), {});
    EXPECT_NEAR(result.radius, 1.0, kTol);
    EXPECT_NEAR(result.center.y, 0.0, kTol);
    EXPECT_LE(std::abs(result.center.x), 0.5 + kTol);
}

TEST(EmptyCircleSolverTest, RectangularPlateauTieBreaksTowardPreference) {
    // The no-opponent solution set on 3.0 x 2.0 is the segment y = 0, |x| <= 0.5; the
    // solver generates its endpoints and the center. The preference point picks among
    // those ties without changing the radius.
    const Size field = make_field(3.0, 2.0);
    const EmptyCircle toward_pos = solve_exact(field, {}, make_pose(1.4, 0.8));
    EXPECT_NEAR(toward_pos.radius, 1.0, kTol);
    EXPECT_NEAR(toward_pos.center.x, 0.5, kTol);
    const EmptyCircle toward_neg = solve_exact(field, {}, make_pose(-1.4, -0.8));
    EXPECT_NEAR(toward_neg.radius, 1.0, kTol);
    EXPECT_NEAR(toward_neg.center.x, -0.5, kTol);
}

TEST(EmptyCircleSolverTest, OpponentAtCenterOfSquareFieldPushesToCorners) {
    // Four corners tie; solved exactly on the diagonal, r = half * (2 - sqrt(2)).
    const std::vector<Pose2D> opponents = {make_pose(0.0, 0.0)};
    const double expected = 1.22 * (2.0 - std::sqrt(2.0));
    const EmptyCircle result = solve_exact(make_field(2.44, 2.44), opponents);
    EXPECT_NEAR(result.radius, expected, kTol);
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
    EXPECT_LT(result.center.x, 0.0);  // far side from the opponent
}

TEST(EmptyCircleSolverTest, OpponentAgainstWallDoesNotStraddleIt) {
    const std::vector<Pose2D> opponents = {make_pose(1.2, 0.0)};
    const EmptyCircle result = solve_exact(make_field(2.44, 2.44), opponents);
    const double dx = result.center.x - opponents[0].x;
    const double dy = result.center.y - opponents[0].y;
    EXPECT_LE(result.radius, std::hypot(dx, dy) + kTol);
}

TEST(EmptyCircleSolverTest, CoincidentTracksMatchSingleOpponentAnswer) {
    // Two filter tracks landing on one robot must not perturb the solver.
    const Size field = make_field(2.44, 2.44);
    const EmptyCircle one = solve_exact(field, {make_pose(0.4, -0.3)});
    const EmptyCircle two = solve_exact(field, {make_pose(0.4, -0.3), make_pose(0.4, -0.3)});
    EXPECT_NEAR(one.radius, two.radius, kTol);
}

TEST(EmptyCircleSolverTest, OpponentOutsideFieldIsHarmless) {
    // The filter can produce sites outside the rectangle; too far away to constrain
    // anything, the answer matches the empty field.
    const EmptyCircle result = solve_exact(make_field(2.44, 2.44), {make_pose(5.0, 0.0)});
    EXPECT_NEAR(result.radius, 1.22, kTol);
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
        // Exact must never fall below the sampled optimum, and the sample can only trail
        // the true optimum by its grid bound.
        EXPECT_GE(exact.radius, brute - kTol) << "trial " << trial;
        EXPECT_LE(exact.radius, brute + brute_bound + kTol) << "trial " << trial;
    }
}

TEST(EmptyCircleSolverTest, ReportsWinningConstraintFamily) {
    // No opponents: only the wall constraints can bind, and on a square field the three-wall
    // triples and the field center tie at the middle.
    const EmptyCircle square = solve_exact(make_field(4.0, 4.0), {});
    EXPECT_TRUE(square.source == "three_wall" || square.source == "field_center") << square.source;

    // One opponent dead center pushes the answer to a corner, which two perpendicular walls
    // and the opponent pin.
    const EmptyCircle pushed = solve_exact(make_field(4.0, 4.0), {make_pose(0.0, 0.0)});
    EXPECT_EQ(pushed.source, "perpendicular_walls_point");

    // A degenerate field still leaves the field marked, never an empty string.
    const EmptyCircle degenerate = solve_exact(make_field(0.0, 0.0), {});
    EXPECT_FALSE(degenerate.source.empty());
}

TEST(EmptyCircleSolverTest, HazardAtCentreSolvesTheWeightedCornerCase) {
    // A hazard is a weighted site: the clearance it contributes is distance minus its radius.
    // As with a bare opponent at the origin the answer sits on a diagonal, but the balance
    // point moves: with the centre at (-(h - r), -(h - r)) the two walls give r and the hazard
    // gives sqrt(2) (h - r) - R, so r = (sqrt(2) h - R) / (1 + sqrt(2)). Not the point answer
    // minus R, which is the easy thing to assume and is wrong.
    const double h = 1.22;
    const double hazard_r = 0.3;
    const Size field = make_field(2.0 * h, 2.0 * h);
    const EmptyCircle result = solve_exact(field, {}, {make_hazard(0.0, 0.0, hazard_r)});
    const double expected = (std::sqrt(2.0) * h - hazard_r) / (1.0 + std::sqrt(2.0));
    EXPECT_NEAR(result.radius, expected, 1e-9);
    EXPECT_NEAR(std::abs(result.center.x), std::abs(result.center.y), 1e-9);
    EXPECT_NEAR(std::abs(result.center.x), h - expected, 1e-9);
}

TEST(EmptyCircleSolverTest, GoalIsPushedOffAHazardSittingOnTheBestPoint) {
    // Without the hazard the safest point is the field center. Dropping a hazard on it has to
    // move the goal out of the disc entirely, not merely reduce the reported radius.
    const Size field = make_field(2.4, 2.4);
    const std::vector<FieldHazard> hazards = {make_hazard(0.0, 0.0, 0.4)};
    const EmptyCircle result = solve_exact(field, {}, hazards);
    const double distance_to_hazard = std::hypot(result.center.x, result.center.y);
    EXPECT_GT(distance_to_hazard, 0.4) << "goal sits inside the hazard";
    EXPECT_NEAR(result.radius, distance_to_hazard - 0.4, 1e-6);
}

TEST(EmptyCircleSolverTest, HazardCoveringTheFieldReportsNegativeRadius) {
    // No legal goal exists. The solver must say so with a negative radius rather than inventing
    // a point, because that is the signal SafestPointTarget uses to decline.
    const Size field = make_field(2.4, 2.4);
    const std::vector<FieldHazard> hazards = {make_hazard(0.0, 0.0, 5.0)};
    const EmptyCircle result = solve_exact(field, {}, hazards);
    EXPECT_LT(result.radius, 0.0);
}

TEST(EmptyCircleSolverTest, MixedOpponentAndHazardMatchesBruteForce) {
    // The families that mix a point site and a weighted site are the ones the closed forms had
    // to grow a radius term for. Brute force is the only check that covers all of them.
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

TEST(EmptyCircleSolverTest, HazardFamiliesAreNamedApart) {
    // A recorded fight without hazards has to read exactly as it did before, and one with them
    // has to say so, or the diagnostics cannot tell which geometry pinned a bad goal.
    const Size field = make_field(2.44, 2.44);
    const EmptyCircle no_hazard = solve_exact(field, {make_pose(0.0, 0.0)});
    EXPECT_EQ(no_hazard.source, "perpendicular_walls_point");

    const EmptyCircle with_hazard = solve_exact(field, {}, {make_hazard(0.0, 0.0, 0.3)});
    EXPECT_EQ(with_hazard.source, "perpendicular_walls_hazard");
}

}  // namespace
}  // namespace auto_battlebot
