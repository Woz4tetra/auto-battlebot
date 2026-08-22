#pragma once

#include <vector>

#include "data_structures/field.hpp"
#include "data_structures/pose.hpp"

namespace auto_battlebot {

/** Result of a largest-empty-circle solve: the circle center, its radius, and how many
 *  candidate evaluations the solver spent. `evaluations` is the machine-independent cost
 *  metric the solver experiment reports alongside wall time. */
struct EmptyCircle {
    Pose2D center{};
    double radius = 0.0;
    int evaluations = 0;
};

/** Radius of the largest circle centered at (x, y) that stays inside the field rectangle
 *  and contains no opponent. The field frame origin is the field center, so the wall term
 *  is min(half_x - |x|, half_y - |y|). Negative outside the rectangle. */
double empty_circle_radius(const Size &field, double x, double y,
                           const std::vector<Pose2D> &opponents);

/** Method A: single pass over a uniform grid with spacing <= resolution_m, no refinement.
 *  The radius function is 1-Lipschitz, so the best node is within resolution_m / sqrt(2)
 *  of the true maximum radius. Ties keep the first node in scan order. */
EmptyCircle solve_coarse_grid(const Size &field, const std::vector<Pose2D> &opponents,
                              double resolution_m);

/** Method B: exact constraint-triple enumeration. The optimum is a vertex of the
 *  generalized Voronoi diagram of the four wall lines plus the opponent sites, so at least
 *  three constraints are active there. Enumerates every triple of the 4 + N constraints,
 *  solves each in closed form, and keeps the feasible candidate with the largest true
 *  radius. Degenerate triples (collinear or coincident sites) produce no candidate and are
 *  skipped; the surviving triples still cover the optimum. */
EmptyCircle solve_exact(const Size &field, const std::vector<Pose2D> &opponents);

/** Method C: Lipschitz branch and bound. Starts from a coarse grid, prunes any cell whose
 *  center value plus its half-diagonal cannot beat the incumbent, subdivides survivors,
 *  and stops once the cell half-diagonal is under tolerance_m. */
EmptyCircle solve_branch_and_bound(const Size &field, const std::vector<Pose2D> &opponents,
                                   double tolerance_m);

/** Ground-truth reference: the coarse grid at a fine resolution (1 mm for the experiment).
 *  Separate entry point so batch runs name the reference explicitly. */
EmptyCircle solve_brute_force(const Size &field, const std::vector<Pose2D> &opponents,
                              double resolution_m);

}  // namespace auto_battlebot
