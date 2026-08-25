#pragma once

#include <optional>
#include <string>
#include <vector>

#include "data_structures/field.hpp"
#include "data_structures/pose.hpp"

namespace auto_battlebot {

/** Result of a largest-empty-circle solve: the circle center, its radius, how many
 *  candidate evaluations the solver spent, and which constraint family won. */
struct EmptyCircle {
    Pose2D center{};
    double radius = 0.0;
    int evaluations = 0;
    /** Constraint family that produced the winning center: "three_wall",
     *  "opposite_walls_point", "perpendicular_walls_point", "wall_point_pair", "point_triple",
     *  or "field_center". The hazard variants swap the site word: "opposite_walls_hazard",
     *  "perpendicular_walls_hazard", "wall_site_pair", "site_triple". "none" when nothing was
     *  considered. Says which geometry pinned the answer, so a bad target can be traced to the
     *  generator that produced it. */
    std::string source = "none";
};

/** Radius of the largest circle centered at (x, y) that stays inside the field rectangle,
 *  contains no opponent, and touches no hazard. The field frame origin is the field center, so
 *  the wall term is min(half_x - |x|, half_y - |y|). Negative outside the rectangle.
 *
 *  A hazard is a site with a radius, so its contribution is `distance_to_center - hazard.radius`
 *  rather than `distance_to_center`. Hazard radii arrive already inflated (see FieldHazard), so
 *  a non-negative result here means the whole robot fits, not just its centre. */
double empty_circle_radius(const Size &field, double x, double y,
                           const std::vector<Pose2D> &opponents,
                           const std::vector<FieldHazard> &hazards);

/** Overload for callers with no hazards to model. */
double empty_circle_radius(const Size &field, double x, double y,
                           const std::vector<Pose2D> &opponents);

/** Exact largest-empty-circle solver by constraint-triple enumeration. The optimum is a
 *  vertex of the generalized Voronoi diagram of the four wall lines plus the opponent
 *  sites, so at least three constraints are active there. Enumerates every triple of the
 *  4 + N constraints, solves each in closed form, and keeps the feasible candidate with
 *  the largest true radius. Degenerate triples (collinear or coincident sites) produce no
 *  candidate and are skipped; the surviving triples still cover the optimum.
 *
 *  Candidates tied on radius (the solution plateau when no opponent constrains the
 *  circle) resolve toward `tie_break_near` when given, so a caller can prefer the spot
 *  closest to its own position instead of crossing the field for nothing. Without it,
 *  ties keep the first candidate in generation order.
 *
 *  Hazards join the site set as weighted points: the constraint families are the same, but the
 *  closed-form solves carry each site's radius, so a triple mixing an opponent and a hazard is
 *  handled by the same routine. Cost grows as the cube of (opponents + hazards), which is fine
 *  because a field holds a handful of hazards, not a hundred.
 *
 *  Chosen over a coarse grid and Lipschitz branch and bound by the solver experiment:
 *  matched a 1 mm brute-force reference on every tick of six recorded fights while being
 *  the cheapest of the three (~15 evaluations per tick). See
 *  docs/experiments/control_improvement/run_away_solver_report.md. */
EmptyCircle solve_exact(const Size &field, const std::vector<Pose2D> &opponents,
                        const std::vector<FieldHazard> &hazards,
                        const std::optional<Pose2D> &tie_break_near = std::nullopt);

/** Overload for callers with no hazards to model. */
EmptyCircle solve_exact(const Size &field, const std::vector<Pose2D> &opponents,
                        const std::optional<Pose2D> &tie_break_near = std::nullopt);

}  // namespace auto_battlebot
