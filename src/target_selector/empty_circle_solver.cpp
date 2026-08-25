#include "target_selector/empty_circle_solver.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace auto_battlebot {
namespace {

constexpr double kDegenerateEps = 1e-9;
// Radii closer than this count as tied and fall through to the tie-break preference.
constexpr double kTieEps = 1e-9;

/** A constraint site: an opponent is a point (radius 0), a hazard is a disc. Clearance from a
 *  site is `distance_to_center - radius`, which makes the two uniform everywhere below. */
struct Site {
    double x = 0.0;
    double y = 0.0;
    double radius = 0.0;
    bool is_hazard = false;
};

/** Wall constraint: clearance(c) = half - sign * c[axis]. */
struct Wall {
    int axis;     // 0 = x, 1 = y
    double sign;  // +1 for the +half wall, -1 for the -half wall
    double half;
};

/** One linear equation in the unknowns (cx, cy, r): a_x cx + a_y cy + a_r r = b.
 *
 *  Every constraint family reduces to two of these plus one site's quadratic. A wall being
 *  active is already linear; two sites being equally clear is linear too, because subtracting
 *  their squared forms cancels cx^2 + cy^2 and r^2. That is what lets one routine solve the
 *  wall/point/hazard mixtures the plan's families spell out separately. */
struct Linear {
    double ax = 0.0;
    double ay = 0.0;
    double ar = 0.0;
    double b = 0.0;
};

double dist2(double ax, double ay, const Pose2D &b) {
    const double dx = ax - b.x;
    const double dy = ay - b.y;
    return dx * dx + dy * dy;
}

/** Real roots of a t^2 + b t + c = 0, degrading to linear when a ~ 0. Returns the number of
 *  roots written. */
int solve_quadratic(double a, double b, double c, std::array<double, 2> &roots) {
    if (std::abs(a) < kDegenerateEps) {
        if (std::abs(b) < kDegenerateEps) return 0;
        roots[0] = -c / b;
        return 1;
    }
    const double disc = b * b - 4.0 * a * c;
    if (disc < 0.0) return 0;
    const double sq = std::sqrt(disc);
    roots[0] = (-b + sq) / (2.0 * a);
    roots[1] = (-b - sq) / (2.0 * a);
    return 2;
}

Linear wall_constraint(const Wall &wall) {
    // r = half - sign * c[axis]  ->  sign * c[axis] + r = half
    Linear linear;
    (wall.axis == 0 ? linear.ax : linear.ay) = wall.sign;
    linear.ar = 1.0;
    linear.b = wall.half;
    return linear;
}

/** Two sites equally clear. Squaring both |c - s_i| = r + R_i and subtracting cancels the
 *  quadratic terms, leaving one linear equation. */
Linear equal_clearance_constraint(const Site &a, const Site &b) {
    Linear linear;
    linear.ax = 2.0 * (b.x - a.x);
    linear.ay = 2.0 * (b.y - a.y);
    linear.ar = 2.0 * (b.radius - a.radius);
    linear.b = (b.x * b.x + b.y * b.y - b.radius * b.radius) -
               (a.x * a.x + a.y * a.y - a.radius * a.radius);
    return linear;
}

/** Tracks the best candidate seen. Every candidate is scored on the true objective, which
 *  doubles as the feasibility check: a candidate outside the rectangle or crowding an
 *  unrelated opponent scores low and loses. Radii within kTieEps count as tied: with a
 *  preference point set, the tied candidate nearest it wins; otherwise the first candidate in
 *  generation order is kept. */
class Best {
   public:
    Best(const Size &field, const std::vector<Pose2D> &opponents,
         const std::vector<FieldHazard> &hazards, const Pose2D *prefer)
        : field_(field), opponents_(opponents), hazards_(hazards), prefer_(prefer) {
        // A field fully covered by constraints still has a least-bad point; starting at -inf
        // reports it rather than a zero-initialized origin.
        result_.radius = -std::numeric_limits<double>::infinity();
    }

    /** `source` names the constraint family that generated the point. Held as a literal
     *  pointer through the enumeration and turned into a string once, in finish(). */
    void consider(double x, double y, const char *source) {
        ++evaluations_;
        const double r = empty_circle_radius(field_, x, y, opponents_, hazards_);
        bool take = r > result_.radius + kTieEps;
        if (!take && prefer_ != nullptr && r > result_.radius - kTieEps) {
            take = dist2(x, y, *prefer_) < dist2(result_.center.x, result_.center.y, *prefer_);
        }
        if (take) {
            result_.center.x = x;
            result_.center.y = y;
            result_.radius = r;
            source_ = source;
        }
    }

    EmptyCircle finish() {
        result_.evaluations = evaluations_;
        result_.source = source_;
        return result_;
    }

   private:
    EmptyCircle result_;
    int evaluations_ = 0;
    const char *source_ = "none";
    const Size &field_;
    const std::vector<Pose2D> &opponents_;
    const std::vector<FieldHazard> &hazards_;
    const Pose2D *prefer_;
};

/** Solve two linear constraints plus one site's quadratic and hand the resulting centers to
 *  `best`.
 *
 *  In (cx, cy, r) space the two linear constraints meet on a line, found as the cross product
 *  of their normals with the usual two-plane particular point. Substituting that line into
 *  |c - s| = r + R leaves a quadratic in the line parameter. Parallel constraints (which is
 *  what a degenerate triple looks like here: coincident sites, collinear sites, the same wall
 *  twice) give a zero direction and are skipped. */
void add_candidates(Best &best, const Linear &first, const Linear &second, const Site &site,
                    const char *source) {
    const double dx = first.ay * second.ar - first.ar * second.ay;
    const double dy = first.ar * second.ax - first.ax * second.ar;
    const double dr = first.ax * second.ay - first.ay * second.ax;
    const double d2 = dx * dx + dy * dy + dr * dr;
    if (d2 < kDegenerateEps) return;

    // P = (b1 (n2 x D) + b2 (D x n1)) / |D|^2 satisfies both planes.
    const double n2d_x = second.ay * dr - second.ar * dy;
    const double n2d_y = second.ar * dx - second.ax * dr;
    const double n2d_r = second.ax * dy - second.ay * dx;
    const double dn1_x = dy * first.ar - dr * first.ay;
    const double dn1_y = dr * first.ax - dx * first.ar;
    const double dn1_r = dx * first.ay - dy * first.ax;

    const double px = (first.b * n2d_x + second.b * dn1_x) / d2;
    const double py = (first.b * n2d_y + second.b * dn1_y) / d2;
    const double pr = (first.b * n2d_r + second.b * dn1_r) / d2;

    const double ux = px - site.x;
    const double uy = py - site.y;
    const double ur = pr + site.radius;

    std::array<double, 2> roots{};
    const int n = solve_quadratic(dx * dx + dy * dy - dr * dr, 2.0 * (ux * dx + uy * dy - ur * dr),
                                  ux * ux + uy * uy - ur * ur, roots);
    for (int i = 0; i < n; ++i) {
        best.consider(px + roots[i] * dx, py + roots[i] * dy, source);
    }
}

/** Three walls: a triple of the four walls contains exactly one opposite pair (axis a) plus one
 *  wall on the other axis (axis b, sign s). Equidistance from the pair pins c[a] = 0 and
 *  r = half_a; matching the third wall puts c[b] at s * (half_b - half_a). */
void add_three_wall_candidates(Best &best, double half_x, double half_y) {
    for (int b_axis = 0; b_axis < 2; ++b_axis) {
        const double half_a = b_axis == 0 ? half_y : half_x;
        const double half_b = b_axis == 0 ? half_x : half_y;
        for (const double s : {1.0, -1.0}) {
            const double cb = s * (half_b - half_a);
            if (b_axis == 0) {
                best.consider(cb, 0.0, "three_wall");
            } else {
                best.consider(0.0, cb, "three_wall");
            }
        }
    }
}

/** Family names are kept in the vocabulary the diagnostics already use, so a recorded fight
 *  without hazards reads exactly as it did before. Hazard involvement swaps the site word. */
const char *two_wall_source(const Wall &a, const Wall &b, const Site &site) {
    if (a.axis == b.axis) {
        return site.is_hazard ? "opposite_walls_hazard" : "opposite_walls_point";
    }
    return site.is_hazard ? "perpendicular_walls_hazard" : "perpendicular_walls_point";
}

}  // namespace

double empty_circle_radius(const Size &field, double x, double y,
                           const std::vector<Pose2D> &opponents,
                           const std::vector<FieldHazard> &hazards) {
    const double half_x = field.x / 2.0;
    const double half_y = field.y / 2.0;
    double radius = std::min(half_x - std::abs(x), half_y - std::abs(y));
    for (const auto &opponent : opponents) {
        const double dx = x - opponent.x;
        const double dy = y - opponent.y;
        radius = std::min(radius, std::sqrt(dx * dx + dy * dy));
    }
    for (const auto &hazard : hazards) {
        const double dx = x - hazard.center.x;
        const double dy = y - hazard.center.y;
        radius = std::min(radius, std::sqrt(dx * dx + dy * dy) - hazard.inflated_radius);
    }
    return radius;
}

double empty_circle_radius(const Size &field, double x, double y,
                           const std::vector<Pose2D> &opponents) {
    static const std::vector<FieldHazard> kNoHazards;
    return empty_circle_radius(field, x, y, opponents, kNoHazards);
}

EmptyCircle solve_exact(const Size &field, const std::vector<Pose2D> &opponents,
                        const std::vector<FieldHazard> &hazards,
                        const std::optional<Pose2D> &tie_break_near) {
    const double half_x = field.x / 2.0;
    const double half_y = field.y / 2.0;

    Best best(field, opponents, hazards,
              tie_break_near.has_value() ? &tie_break_near.value() : nullptr);
    // The field center covers the fully degenerate case (every triple skipped) and joins the
    // no-opponent plateau tie on a non-square field.
    best.consider(0.0, 0.0, "field_center");

    std::vector<Site> sites;
    sites.reserve(opponents.size() + hazards.size());
    for (const auto &opponent : opponents) {
        sites.push_back(Site{opponent.x, opponent.y, 0.0, false});
    }
    for (const auto &hazard : hazards) {
        sites.push_back(Site{hazard.center.x, hazard.center.y, hazard.inflated_radius, true});
    }

    const Wall walls[4] = {
        {0, 1.0, half_x}, {0, -1.0, half_x}, {1, 1.0, half_y}, {1, -1.0, half_y}};
    std::array<Linear, 4> wall_linears{};
    for (int i = 0; i < 4; ++i) {
        wall_linears[i] = wall_constraint(walls[i]);
    }

    add_three_wall_candidates(best, half_x, half_y);

    for (int i = 0; i < 4; ++i) {
        for (int j = i + 1; j < 4; ++j) {
            for (const auto &site : sites) {
                add_candidates(best, wall_linears[i], wall_linears[j], site,
                               two_wall_source(walls[i], walls[j], site));
            }
        }
    }

    for (int w = 0; w < 4; ++w) {
        for (size_t i = 0; i < sites.size(); ++i) {
            for (size_t j = i + 1; j < sites.size(); ++j) {
                const char *source = (sites[i].is_hazard || sites[j].is_hazard) ? "wall_site_pair"
                                                                                : "wall_point_pair";
                add_candidates(best, wall_linears[w],
                               equal_clearance_constraint(sites[i], sites[j]), sites[i], source);
            }
        }
    }

    for (size_t i = 0; i < sites.size(); ++i) {
        for (size_t j = i + 1; j < sites.size(); ++j) {
            for (size_t k = j + 1; k < sites.size(); ++k) {
                const char *source =
                    (sites[i].is_hazard || sites[j].is_hazard || sites[k].is_hazard)
                        ? "site_triple"
                        : "point_triple";
                add_candidates(best, equal_clearance_constraint(sites[i], sites[j]),
                               equal_clearance_constraint(sites[i], sites[k]), sites[i], source);
            }
        }
    }

    return best.finish();
}

EmptyCircle solve_exact(const Size &field, const std::vector<Pose2D> &opponents,
                        const std::optional<Pose2D> &tie_break_near) {
    static const std::vector<FieldHazard> kNoHazards;
    return solve_exact(field, opponents, kNoHazards, tie_break_near);
}

}  // namespace auto_battlebot
