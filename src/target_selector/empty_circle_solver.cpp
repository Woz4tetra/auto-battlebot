#include "target_selector/empty_circle_solver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace auto_battlebot {
namespace {

constexpr double kDegenerateEps = 1e-9;

/** Wall constraint: clearance(c) = half - sign * c[axis]. */
struct Wall {
    int axis;     // 0 = x, 1 = y
    double sign;  // +1 for the +half wall, -1 for the -half wall
    double half;
};

double coord(const Pose2D &p, int axis) { return axis == 0 ? p.x : p.y; }

/** Evaluate a candidate against the true objective and keep it if it beats the incumbent.
 *  Strictly-greater keeps the first best in generation order, so results are deterministic. */
struct Best {
    EmptyCircle result;
    int evaluations = 0;

    double consider(const Size &field, const std::vector<Pose2D> &opponents, double x, double y) {
        ++evaluations;
        const double r = empty_circle_radius(field, x, y, opponents);
        if (r > result.radius) {
            result.center.x = x;
            result.center.y = y;
            result.radius = r;
        }
        return r;
    }
};

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

EmptyCircle solve_coarse_grid(const Size &field, const std::vector<Pose2D> &opponents,
                              double resolution_m) {
    const double step = std::max(resolution_m, 1e-6);
    const int nx = std::max(2, static_cast<int>(std::ceil(field.x / step)) + 1);
    const int ny = std::max(2, static_cast<int>(std::ceil(field.y / step)) + 1);
    const double step_x = field.x / (nx - 1);
    const double step_y = field.y / (ny - 1);
    const double half_x = field.x / 2.0;
    const double half_y = field.y / 2.0;

    Best best;
    // Start below any real radius so a field with an opponent on every node still returns the
    // least bad node instead of the zero-initialized origin.
    best.result.radius = -std::numeric_limits<double>::infinity();
    for (int iy = 0; iy < ny; ++iy) {
        const double y = -half_y + iy * step_y;
        for (int ix = 0; ix < nx; ++ix) {
            best.consider(field, opponents, -half_x + ix * step_x, y);
        }
    }
    best.result.evaluations = best.evaluations;
    return best.result;
}

EmptyCircle solve_brute_force(const Size &field, const std::vector<Pose2D> &opponents,
                              double resolution_m) {
    return solve_coarse_grid(field, opponents, resolution_m);
}

EmptyCircle solve_exact(const Size &field, const std::vector<Pose2D> &opponents) {
    const double half_x = field.x / 2.0;
    const double half_y = field.y / 2.0;
    const Wall walls[4] = {
        {0, 1.0, half_x}, {0, -1.0, half_x}, {1, 1.0, half_y}, {1, -1.0, half_y}};

    Best best;
    best.result.radius = -std::numeric_limits<double>::infinity();
    // Field center covers the fully degenerate case (every triple skipped) and ties the
    // no-opponent square field to its exact answer.
    best.consider(field, opponents, 0.0, 0.0);

    // Three walls: each triple of the four walls contains exactly one opposite pair (axis a)
    // plus one wall on the other axis (axis b, sign s). Equidistance from the pair pins
    // c[a] = 0 with r = half_a; matching the third wall puts c[b] at s * (half_b - half_a).
    for (int b_axis = 0; b_axis < 2; ++b_axis) {
        const double half_a = b_axis == 0 ? half_y : half_x;
        const double half_b = b_axis == 0 ? half_x : half_y;
        for (const double s : {1.0, -1.0}) {
            const double cb = s * (half_b - half_a);
            if (b_axis == 0) {
                best.consider(field, opponents, cb, 0.0);
            } else {
                best.consider(field, opponents, 0.0, cb);
            }
        }
    }

    // Two opposite walls + one point: c[a] = 0, r = half_a, and |c - o| = half_a fixes the
    // other coordinate up to a sign.
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
                    best.consider(field, opponents, 0.0, cb);
                } else {
                    best.consider(field, opponents, cb, 0.0);
                }
            }
        }
    }

    // Two perpendicular walls + one point: equidistance from the walls gives
    // x = sx * (half_x - r), y = sy * (half_y - r); equating the point distance yields
    // r^2 - 2 (p + q) r + p^2 + q^2 = 0 with p = half_x - sx * ox, q = half_y - sy * oy,
    // so r = (p + q) +/- sqrt(2 p q).
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
                    best.consider(field, opponents, sx * (half_x - r), sy * (half_y - r));
                }
            }
        }
    }

    // One wall + two points: the center sits on the perpendicular bisector of the pair,
    // c(t) = m + t * d with d perpendicular to (q - p), where the wall distance is linear in t
    // and the point distance squared is e^2 + t^2 (e = half the pair separation). Equating
    // them is a quadratic in t.
    for (const auto &wall : walls) {
        for (size_t i = 0; i < opponents.size(); ++i) {
            for (size_t j = i + 1; j < opponents.size(); ++j) {
                const Pose2D &p = opponents[i];
                const Pose2D &q = opponents[j];
                const double dx = q.x - p.x;
                const double dy = q.y - p.y;
                const double sep = std::sqrt(dx * dx + dy * dy);
                if (sep < kDegenerateEps) continue;  // coincident tracks on one robot
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
                const double a = w1 * w1 - 1.0;
                const double b = 2.0 * w0 * w1;
                const double c = w0 * w0 - e * e;
                std::vector<double> roots;
                if (std::abs(a) < kDegenerateEps) {
                    if (std::abs(b) > kDegenerateEps) roots.push_back(-c / b);
                } else {
                    const double disc = b * b - 4.0 * a * c;
                    if (disc >= 0.0) {
                        const double sq = std::sqrt(disc);
                        roots.push_back((-b + sq) / (2.0 * a));
                        roots.push_back((-b - sq) / (2.0 * a));
                    }
                }
                for (const double t : roots) {
                    best.consider(field, opponents, mx + t * ux, my + t * uy);
                }
            }
        }
    }

    // Three points: circumcenter, skipped when the sites are collinear (or coincident, which
    // makes them collinear too).
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
                best.consider(field, opponents, cx, cy);
            }
        }
    }

    best.result.evaluations = best.evaluations;
    return best.result;
}

EmptyCircle solve_branch_and_bound(const Size &field, const std::vector<Pose2D> &opponents,
                                   double tolerance_m) {
    constexpr double kInitialCellSize = 0.20;
    const double tolerance = std::max(tolerance_m, 1e-4);
    const double half_x = field.x / 2.0;
    const double half_y = field.y / 2.0;

    // Cells are uniform per level; only centers are stored.
    struct Cell {
        double x;
        double y;
    };
    const int ncx = std::max(1, static_cast<int>(std::ceil(field.x / kInitialCellSize)));
    const int ncy = std::max(1, static_cast<int>(std::ceil(field.y / kInitialCellSize)));
    double cell_x = field.x / ncx;
    double cell_y = field.y / ncy;

    std::vector<Cell> cells;
    cells.reserve(static_cast<size_t>(ncx) * ncy);
    for (int iy = 0; iy < ncy; ++iy) {
        for (int ix = 0; ix < ncx; ++ix) {
            cells.push_back({-half_x + (ix + 0.5) * cell_x, -half_y + (iy + 0.5) * cell_y});
        }
    }

    Best best;
    best.result.radius = -std::numeric_limits<double>::infinity();
    while (true) {
        const double half_diag = std::sqrt(cell_x * cell_x + cell_y * cell_y) / 2.0;
        std::vector<double> values(cells.size());
        for (size_t i = 0; i < cells.size(); ++i) {
            values[i] = best.consider(field, opponents, cells[i].x, cells[i].y);
        }
        if (half_diag < tolerance || cells.empty()) break;

        // The radius is 1-Lipschitz, so no point of a cell can beat its center value by more
        // than the half-diagonal. Cells that cannot reach the incumbent are discarded.
        std::vector<Cell> survivors;
        for (size_t i = 0; i < cells.size(); ++i) {
            if (values[i] + half_diag > best.result.radius) survivors.push_back(cells[i]);
        }

        cell_x /= 2.0;
        cell_y /= 2.0;
        std::vector<Cell> children;
        children.reserve(survivors.size() * 4);
        for (const auto &cell : survivors) {
            for (const double sx : {-0.5, 0.5}) {
                for (const double sy : {-0.5, 0.5}) {
                    children.push_back({cell.x + sx * cell_x, cell.y + sy * cell_y});
                }
            }
        }
        cells.swap(children);
    }

    best.result.evaluations = best.evaluations;
    return best.result;
}

}  // namespace auto_battlebot
