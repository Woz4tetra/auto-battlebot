#include "field_filter/field_outline.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

namespace auto_battlebot {
namespace {
/** Total-least-squares line through points, as a homogeneous line (a, b, c) with a^2 + b^2 = 1
 *  and a*x + b*y + c = 0. */
cv::Vec3d fit_line(const std::vector<cv::Point2d> &points) {
    cv::Point2d centroid(0.0, 0.0);
    for (const auto &point : points) {
        centroid += point;
    }
    centroid *= 1.0 / static_cast<double>(points.size());

    cv::Mat centered(static_cast<int>(points.size()), 2, CV_64F);
    for (size_t i = 0; i < points.size(); ++i) {
        centered.at<double>(static_cast<int>(i), 0) = points[i].x - centroid.x;
        centered.at<double>(static_cast<int>(i), 1) = points[i].y - centroid.y;
    }
    cv::Mat w;
    cv::Mat u;
    cv::Mat vt;
    cv::SVD::compute(centered, w, u, vt, cv::SVD::MODIFY_A);
    // Largest singular vector is the direction; the normal is perpendicular to it.
    const double dx = vt.at<double>(0, 0);
    const double dy = vt.at<double>(0, 1);
    const cv::Vec2d normal(-dy, dx);
    return cv::Vec3d(normal[0], normal[1], -(normal[0] * centroid.x + normal[1] * centroid.y));
}

double point_line_distance(const cv::Vec3d &line, const cv::Point2d &point) {
    return std::abs(line[0] * point.x + line[1] * point.y + line[2]);
}

/** Intersection of two homogeneous lines, or nullopt when they are parallel. */
bool intersect_lines(const cv::Vec3d &first, const cv::Vec3d &second, cv::Point2d &out) {
    const cv::Vec3d point = first.cross(second);
    if (std::abs(point[2]) < 1e-9) {
        return false;
    }
    out = cv::Point2d(point[0] / point[2], point[1] / point[2]);
    return true;
}

cv::Vec3d line_through(const cv::Point2d &first, const cv::Point2d &second) {
    const cv::Vec3d a(first.x, first.y, 1.0);
    const cv::Vec3d b(second.x, second.y, 1.0);
    cv::Vec3d line = a.cross(b);
    const double scale = std::hypot(line[0], line[1]);
    if (scale > 1e-12) {
        line /= scale;
    }
    return line;
}

/** Counter-clockwise from the corner with the smallest x + y.
 *
 * A consistent winding is what makes the recovered rotation reproducible; without it the yaw
 * jumps by multiples of 90 degrees frame to frame. */
std::array<cv::Point2d, 4> order_corners(std::array<cv::Point2d, 4> corners) {
    cv::Point2d centre(0.0, 0.0);
    for (const auto &corner : corners) {
        centre += corner;
    }
    centre *= 0.25;

    std::sort(corners.begin(), corners.end(), [&centre](const auto &a, const auto &b) {
        return std::atan2(a.y - centre.y, a.x - centre.x) <
               std::atan2(b.y - centre.y, b.x - centre.x);
    });

    size_t start = 0;
    double best = std::numeric_limits<double>::max();
    for (size_t i = 0; i < corners.size(); ++i) {
        const double sum = corners[i].x + corners[i].y;
        if (sum < best) {
            best = sum;
            start = i;
        }
    }
    std::array<cv::Point2d, 4> ordered{};
    for (size_t i = 0; i < corners.size(); ++i) {
        ordered[i] = corners[(start + i) % corners.size()];
    }
    return ordered;
}

/** Dense boundary of the largest contour, with points near the frame edge removed. */
std::vector<cv::Point2d> boundary_points(const cv::Mat &binary, int border_margin_px) {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    std::vector<cv::Point2d> points;
    if (contours.empty()) {
        return points;
    }
    const auto &largest = *std::max_element(
        contours.begin(), contours.end(),
        [](const auto &a, const auto &b) { return cv::contourArea(a) < cv::contourArea(b); });

    const double margin = static_cast<double>(border_margin_px);
    const double max_x = binary.cols - 1.0 - margin;
    const double max_y = binary.rows - 1.0 - margin;
    points.reserve(largest.size());
    for (const auto &point : largest) {
        if (point.x <= margin || point.y <= margin || point.x >= max_x || point.y >= max_y) {
            continue;
        }
        points.emplace_back(point.x, point.y);
    }
    return points;
}

/** Four seed corners from the convex hull, loosening approxPolyDP until exactly four survive.
 *
 * Under perspective a rectangle projects to a general quadrilateral, not a rotated rectangle, so
 * minAreaRect is wrong here: it returns the bounding box of the quad and the error grows with
 * camera tilt. */
bool seed_quad(const cv::Mat &binary, std::array<cv::Point2d, 4> &out) {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) {
        return false;
    }
    const auto &largest = *std::max_element(
        contours.begin(), contours.end(),
        [](const auto &a, const auto &b) { return cv::contourArea(a) < cv::contourArea(b); });

    std::vector<cv::Point> hull;
    cv::convexHull(largest, hull);
    if (hull.size() < 4) {
        return false;
    }
    const double perimeter = cv::arcLength(hull, true);
    constexpr int kSteps = 60;
    for (int step = 0; step < kSteps; ++step) {
        const double fraction = 0.005 + (0.12 - 0.005) * step / (kSteps - 1.0);
        std::vector<cv::Point> approx;
        cv::approxPolyDP(hull, approx, fraction * perimeter, true);
        if (approx.size() == 4) {
            for (size_t i = 0; i < 4; ++i) {
                out[i] = cv::Point2d(approx[i].x, approx[i].y);
            }
            return true;
        }
    }
    return false;
}
}  // namespace

cv::Mat find_largest_contour_mask(const cv::Mat &mask) {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    cv::Mat result = cv::Mat::zeros(mask.size(), mask.type());
    if (contours.empty()) {
        return result;
    }

    double max_area = 0.0;
    size_t max_area_index = 0;
    for (size_t i = 0; i < contours.size(); ++i) {
        const double area = cv::contourArea(contours[i]);
        if (area > max_area) {
            max_area = area;
            max_area_index = i;
        }
    }
    cv::drawContours(result, contours, static_cast<int>(max_area_index), cv::Scalar(255),
                     cv::FILLED);
    return result;
}

FieldOutline extract_field_outline(const cv::Mat &contour_mask, const FieldOutlineParams &params) {
    FieldOutline outline;
    if (contour_mask.empty()) {
        outline.failure = "empty field mask";
        return outline;
    }
    cv::Mat binary;
    cv::compare(contour_mask, 0, binary, cv::CMP_GT);

    std::array<cv::Point2d, 4> seed{};
    if (!seed_quad(binary, seed)) {
        outline.failure = "no four-sided approximation of the field outline";
        return outline;
    }

    const std::vector<cv::Point2d> points = boundary_points(binary, params.border_margin_px);
    if (points.size() < 64) {
        outline.failure = "field outline has too few points away from the frame edge";
        return outline;
    }

    std::array<cv::Point2d, 4> quad = order_corners(seed);
    std::array<cv::Vec3d, 4> lines{};
    std::array<bool, 4> supported{};
    for (size_t i = 0; i < 4; ++i) {
        lines[i] = line_through(quad[i], quad[(i + 1) % 4]);
    }

    for (int iteration = 0; iteration < params.refine_iterations; ++iteration) {
        std::array<cv::Point2d, 4> starts{};
        std::array<cv::Point2d, 4> units{};
        std::array<double, 4> lengths{};
        for (size_t i = 0; i < 4; ++i) {
            starts[i] = quad[i];
            const cv::Point2d edge = quad[(i + 1) % 4] - quad[i];
            lengths[i] = std::hypot(edge.x, edge.y);
            if (lengths[i] < 1e-6) {
                outline.failure = "degenerate field outline edge";
                return outline;
            }
            units[i] = edge * (1.0 / lengths[i]);
        }

        // Assign each boundary point to whichever side it is nearest, then keep the straight
        // middle stretch. approxPolyDP can only return vertices lying on the outline, so where
        // the mat has rounded corners its quad chords across them and comes out short; fitting
        // the middle of each side and intersecting extrapolates back to the real corners.
        std::array<std::vector<cv::Point2d>, 4> owned;
        for (const auto &point : points) {
            size_t owner = 0;
            double best = std::numeric_limits<double>::max();
            double best_along = 0.0;
            for (size_t i = 0; i < 4; ++i) {
                const cv::Point2d relative = point - starts[i];
                const double perpendicular =
                    std::abs(relative.x * -units[i].y + relative.y * units[i].x);
                if (perpendicular < best) {
                    best = perpendicular;
                    owner = i;
                    best_along = (relative.x * units[i].x + relative.y * units[i].y) / lengths[i];
                }
            }
            if (best_along <= params.corner_skip_fraction ||
                best_along >= 1.0 - params.corner_skip_fraction) {
                continue;
            }
            owned[owner].push_back(point);
        }

        for (size_t i = 0; i < 4; ++i) {
            if (owned[i].size() < static_cast<size_t>(params.min_side_points)) {
                // No support. Keep the seed line so the quad stays closed, and let the pose
                // solver work from the sides that do have support.
                supported[i] = false;
                continue;
            }
            cv::Vec3d line = fit_line(owned[i]);
            // One robust pass, so a robot resting on the field edge cannot pull the line in.
            std::vector<double> residuals;
            residuals.reserve(owned[i].size());
            for (const auto &point : owned[i]) {
                residuals.push_back(point_line_distance(line, point));
            }
            std::vector<double> sorted = residuals;
            std::nth_element(sorted.begin(), sorted.begin() + sorted.size() / 2, sorted.end());
            const double limit = std::max(2.0, 2.5 * sorted[sorted.size() / 2]);
            std::vector<cv::Point2d> kept;
            kept.reserve(owned[i].size());
            for (size_t p = 0; p < owned[i].size(); ++p) {
                if (residuals[p] <= limit) {
                    kept.push_back(owned[i][p]);
                }
            }
            if (kept.size() >= static_cast<size_t>(params.min_side_points)) {
                line = fit_line(kept);
            }
            lines[i] = line;
            supported[i] = true;
        }

        std::array<cv::Point2d, 4> next{};
        for (size_t i = 0; i < 4; ++i) {
            // Intersections land in edge order, so corner i sits between sides i-1 and i.
            if (!intersect_lines(lines[(i + 3) % 4], lines[i], next[i])) {
                outline.failure = "field outline sides do not intersect";
                return outline;
            }
        }
        quad = next;
    }

    outline.corners = quad;
    outline.lines = lines;
    outline.side_supported = supported;
    outline.supported_sides =
        static_cast<int>(std::count(supported.begin(), supported.end(), true));

    // Against the quad clipped to the image, not the whole quad. A clipped field puts corners
    // off the sensor, so the raw quad is larger than anything the mask could ever fill and the
    // ratio would read as a bad outline on exactly the case this filter is built to handle.
    std::vector<cv::Point2f> quad_f;
    quad_f.reserve(4);
    for (const auto &corner : quad) {
        quad_f.emplace_back(static_cast<float>(corner.x), static_cast<float>(corner.y));
    }
    const std::vector<cv::Point2f> image_rectangle = {
        {0.0F, 0.0F},
        {static_cast<float>(binary.cols), 0.0F},
        {static_cast<float>(binary.cols), static_cast<float>(binary.rows)},
        {0.0F, static_cast<float>(binary.rows)}};
    std::vector<cv::Point2f> visible_quad;
    const double quad_area =
        std::abs(cv::intersectConvexConvex(quad_f, image_rectangle, visible_quad, true));
    outline.mask_over_quad_area = cv::countNonZero(binary) / std::max(1.0, quad_area);

    if (outline.supported_sides < 3) {
        outline.failure = "only " + std::to_string(outline.supported_sides) +
                          " field side(s) have support away from the frame edge";
        return outline;
    }
    outline.ok = true;
    return outline;
}
}  // namespace auto_battlebot
