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

/** Wall constraint: clearance(c) = half - sign * c[axis]. */
struct Wall {
    int axis;     // 0 = x, 1 = y
    double sign;  // +1 for the +half wall, -1 for the -half wall
    double half;
};

double coord(const Pose2D &p, int axis) { return axis == 0 ? p.x : p.y; }

double dist2(double ax, double ay, const Pose2D &b) {
    const double dx = ax - b.x;
    const double dy = ay - b.y;
    return dx * dx + dy * dy;
}

/** Real roots of a t^2 + b t + c = 0, degrading to linear when a ~ 0. Returns the number
 *  of roots written. */
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

/** Tracks the best candidate seen. Every candidate is scored with the true objective,
 *  which doubles as the feasibility check: a candidate outside the rectangle or crowding
 *  an unrelated opponent scores low and loses. Radii within kTieEps count as tied: with a
 *  preference point set, the tied candidate nearest it wins; otherwise the first candidate
 *  in generation order is kept. */
class Best {
   public:
    Best(const Size &field, const std::vector<Pose2D> &opponents, const Pose2D *prefer)
        : field_(field), opponents_(opponents), prefer_(prefer) {
        // Start below any real radius so a fully constrained field still returns the least
        // bad candidate instead of the zero-initialized origin.
        result_.radius = -std::numeric_limits<double>::infinity();
    }

    /** `source` names the constraint family that generated the point. Held as a literal
     *  pointer through the enumeration and turned into a string once, in finish(). */
    void consider(double x, double y, const char *source) {
        ++evaluations_;
        const double r = empty_circle_radius(field_, x, y, opponents_);
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
    const Pose2D *prefer_;
};

/** Three walls: each triple of the four walls contains exactly one opposite pair (axis a)
 *  plus one wall on the other axis (axis b, sign s). Equidistance from the pair pins
 *  c[a] = 0 with r = half_a; matching the third wall puts c[b] at s * (half_b - half_a). */
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

/** Two opposite walls + one point: c[a] = 0, r = half_a, and |c - o| = half_a fixes the
 *  other coordinate up to a sign. */
void add_opposite_walls_point_candidates(Best &best, double half_x, double half_y,
                                         const std::vector<Pose2D> &opponents) {
    for (int a_axis = 0; a_axis < 2; ++a_axis) {
        const double half_a = a_axis == 0 ? half_x : half_y;
        for (const auto &o : opponents) {
            const double oa = coord(o, a_axis);
            const double ob = coord(o, 1 - a_axis);
            const double disc = half_a * half_a - oa * oa;
            if (disc < 0.0) continue;
            const double offset = std::sqrt(disc);
            for (const double s : {1.0, -1.0}) {
                const double cb = ob + s * offset;
                if (a_axis == 0) {
                    best.consider(0.0, cb, "opposite_walls_point");
                } else {
                    best.consider(cb, 0.0, "opposite_walls_point");
                }
            }
        }
    }
}

/** Two perpendicular walls + one point: equidistance from the walls gives
 *  x = sx * (half_x - r), y = sy * (half_y - r); equating the point distance yields
 *  r^2 - 2 (p + q) r + p^2 + q^2 = 0 with p = half_x - sx * ox, q = half_y - sy * oy,
 *  so r = (p + q) +/- sqrt(2 p q). */
void add_perpendicular_walls_point_candidates(Best &best, double half_x, double half_y,
                                              const std::vector<Pose2D> &opponents) {
    for (const double sx : {1.0, -1.0}) {
        for (const double sy : {1.0, -1.0}) {
            for (const auto &o : opponents) {
                const double p = half_x - sx * o.x;
                const double q = half_y - sy * o.y;
                const double disc = 2.0 * p * q;
                if (disc < 0.0) continue;
                const double root = std::sqrt(disc);
                for (const double s : {1.0, -1.0}) {
                    const double r = p + q + s * root;
                    if (r <= 0.0) continue;
                    best.consider(sx * (half_x - r), sy * (half_y - r),
                                  "perpendicular_walls_point");
                }
            }
        }
    }
}

/** One wall + one pair of points: the center sits on the pair's perpendicular bisector,
 *  c(t) = m + t * u with u perpendicular to (q - p). Along it the wall distance is linear,
 *  w0 + w1 t, and the point distance squared is e^2 + t^2 (e = half the pair separation),
 *  so equating them is a quadratic in t. */
void add_wall_point_pair_candidates(Best &best, const Wall &wall, const Pose2D &p,
                                    const Pose2D &q) {
    const double dx = q.x - p.x;
    const double dy = q.y - p.y;
    const double sep = std::sqrt(dx * dx + dy * dy);
    if (sep < kDegenerateEps) return;  // coincident tracks on one robot
    const double mx = (p.x + q.x) / 2.0;
    const double my = (p.y + q.y) / 2.0;
    const double ux = -dy / sep;
    const double uy = dx / sep;
    const double e = sep / 2.0;
    const double m_a = wall.axis == 0 ? mx : my;
    const double u_a = wall.axis == 0 ? ux : uy;
    const double w0 = wall.half - wall.sign * m_a;
    const double w1 = -wall.sign * u_a;
    // (w0 + w1 t)^2 = e^2 + t^2
    std::array<double, 2> roots{};
    const int n = solve_quadratic(w1 * w1 - 1.0, 2.0 * w0 * w1, w0 * w0 - e * e, roots);
    for (int i = 0; i < n; ++i) {
        best.consider(mx + roots[i] * ux, my + roots[i] * uy, "wall_point_pair");
    }
}

/** Three points: the circumcenter, skipped when the sites are collinear (or coincident,
 *  which makes them collinear too). */
void add_point_triple_candidates(Best &best, const std::vector<Pose2D> &opponents) {
    for (size_t i = 0; i < opponents.size(); ++i) {
        for (size_t j = i + 1; j < opponents.size(); ++j) {
            for (size_t k = j + 1; k < opponents.size(); ++k) {
                const Pose2D &pa = opponents[i];
                const Pose2D &pb = opponents[j];
                const Pose2D &pc = opponents[k];
                const double d =
                    2.0 * ((pb.x - pa.x) * (pc.y - pa.y) - (pb.y - pa.y) * (pc.x - pa.x));
                if (std::abs(d) < kDegenerateEps) continue;
                const double a2 = pa.x * pa.x + pa.y * pa.y;
                const double b2 = pb.x * pb.x + pb.y * pb.y;
                const double c2 = pc.x * pc.x + pc.y * pc.y;
                const double cx =
                    (a2 * (pb.y - pc.y) + b2 * (pc.y - pa.y) + c2 * (pa.y - pb.y)) / d;
                const double cy =
                    (a2 * (pc.x - pb.x) + b2 * (pa.x - pc.x) + c2 * (pb.x - pa.x)) / d;
                best.consider(cx, cy, "point_triple");
            }
        }
    }
}

}  // namespace

double empty_circle_radius(const Size &field, double x, double y,
                           const std::vector<Pose2D> &opponents) {
    const double half_x = field.x / 2.0;
    const double half_y = field.y / 2.0;
    double radius = std::min(half_x - std::abs(x), half_y - std::abs(y));
    for (const auto &opponent : opponents) {
        const double dx = x - opponent.x;
        const double dy = y - opponent.y;
        radius = std::min(radius, std::sqrt(dx * dx + dy * dy));
    }
    return radius;
}

EmptyCircle solve_exact(const Size &field, const std::vector<Pose2D> &opponents,
                        const std::optional<Pose2D> &tie_break_near) {
    const double half_x = field.x / 2.0;
    const double half_y = field.y / 2.0;

    Best best(field, opponents, tie_break_near.has_value() ? &tie_break_near.value() : nullptr);
    // Field center covers the fully degenerate case (every triple skipped) and joins the
    // no-opponent plateau tie on a non-square field.
    best.consider(0.0, 0.0, "field_center");

    add_three_wall_candidates(best, half_x, half_y);
    add_opposite_walls_point_candidates(best, half_x, half_y, opponents);
    add_perpendicular_walls_point_candidates(best, half_x, half_y, opponents);
    const Wall walls[4] = {
        {0, 1.0, half_x}, {0, -1.0, half_x}, {1, 1.0, half_y}, {1, -1.0, half_y}};
    for (const auto &wall : walls) {
        for (size_t i = 0; i < opponents.size(); ++i) {
            for (size_t j = i + 1; j < opponents.size(); ++j) {
                add_wall_point_pair_candidates(best, wall, opponents[i], opponents[j]);
            }
        }
    }
    add_point_triple_candidates(best, opponents);

    return best.finish();
}

}  // namespace auto_battlebot
