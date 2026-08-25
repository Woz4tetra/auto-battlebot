#include "hazards/hazard_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace auto_battlebot {
namespace {
constexpr double kEps = 1e-9;
/** Enough passes to settle two or three overlapping discs; past that the goal is wedged and
 *  another pass will not free it. */
constexpr int kPushOutPasses = 6;

double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

/** Squared distance from `point` to the segment [a, b], plus where along it the foot lands
 *  (0 at a, 1 at b). */
double segment_distance(const Pose2D &a, const Pose2D &b, const Pose2D &point, double &t_out) {
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double len2 = dx * dx + dy * dy;
    double t = 0.0;
    if (len2 > kEps) {
        t = ((point.x - a.x) * dx + (point.y - a.y) * dy) / len2;
        t = std::clamp(t, 0.0, 1.0);
    }
    t_out = t;
    return std::hypot(a.x + t * dx - point.x, a.y + t * dy - point.y);
}
}  // namespace

Pose2D predicted_center(const FieldHazard &hazard, double seconds) {
    if (hazard.source == HazardSource::STATIC || seconds <= 0.0) return hazard.center;
    Pose2D center = hazard.center;
    center.x += hazard.velocity.vx * seconds;
    center.y += hazard.velocity.vy * seconds;
    return center;
}

Pose2D push_out_of_hazards(const Pose2D &pose, const std::vector<FieldHazard> &hazards,
                           double half_x, double half_y) {
    Pose2D result = pose;
    for (int pass = 0; pass < kPushOutPasses; ++pass) {
        bool moved = false;
        for (const auto &hazard : hazards) {
            double dx = result.x - hazard.center.x;
            double dy = result.y - hazard.center.y;
            double distance = std::hypot(dx, dy);
            if (distance >= hazard.inflated_radius) continue;
            if (distance < kEps) {
                // Dead centre: no radial direction to follow, so leave along +x. Any direction
                // is equally good and picking one keeps the result deterministic.
                dx = 1.0;
                dy = 0.0;
                distance = 1.0;
            }
            result.x = hazard.center.x + dx / distance * hazard.inflated_radius;
            result.y = hazard.center.y + dy / distance * hazard.inflated_radius;
            moved = true;
        }
        if (half_x > 0.0) result.x = std::clamp(result.x, -half_x, half_x);
        if (half_y > 0.0) result.y = std::clamp(result.y, -half_y, half_y);
        if (!moved) break;
    }
    return result;
}

std::optional<size_t> first_blocking_hazard(const Pose2D &start, const Pose2D &goal,
                                            const std::vector<FieldHazard> &hazards,
                                            double predict_s, const std::vector<size_t> &skip) {
    std::optional<size_t> nearest;
    double nearest_t = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < hazards.size(); ++i) {
        if (std::find(skip.begin(), skip.end(), i) != skip.end()) continue;
        Pose2D center = predicted_center(hazards[i], predict_s);
        double t = 0.0;
        const double distance = segment_distance(start, goal, center, t);
        if (distance >= hazards[i].inflated_radius) continue;
        if (t < nearest_t) {
            nearest_t = t;
            nearest = i;
        }
    }
    return nearest;
}

TangentWaypoint tangent_waypoint(const Pose2D &start, const Pose2D &center, double radius,
                                 double heading, int committed_side) {
    const double dx = center.x - start.x;
    const double dy = center.y - start.y;
    const double distance = std::hypot(dx, dy);

    TangentWaypoint waypoint;
    if (distance <= kEps) {
        // Dead centre: no radial direction either. Hold the current heading and let the reverse
        // backstop work; any choice here is arbitrary, so make it deterministic.
        waypoint.inside = true;
        waypoint.point.x = start.x + radius * std::cos(heading);
        waypoint.point.y = start.y + radius * std::sin(heading);
        return waypoint;
    }

    if (distance <= radius) {
        // Inside the circle. Straight out along the radius is the shortest way to stop being.
        waypoint.inside = true;
        waypoint.point.x = center.x - dx / distance * radius;
        waypoint.point.y = center.y - dy / distance * radius;
        return waypoint;
    }

    const double to_center = std::atan2(dy, dx);
    const double half_angle = std::asin(std::clamp(radius / distance, -1.0, 1.0));
    const double tangent_len = std::sqrt(std::max(0.0, distance * distance - radius * radius));

    int side = committed_side;
    if (side == 0) {
        // Cheaper side = the tangent that costs less heading change from where we point now.
        const double left = std::abs(normalize_angle(to_center + half_angle - heading));
        const double right = std::abs(normalize_angle(to_center - half_angle - heading));
        side = (left <= right) ? 1 : -1;
    }

    const double bearing = to_center + side * half_angle;
    waypoint.point.x = start.x + tangent_len * std::cos(bearing);
    waypoint.point.y = start.y + tangent_len * std::sin(bearing);
    waypoint.side = side;
    return waypoint;
}

double hazard_speed_cap(const Pose2D &our_pose, const std::vector<FieldHazard> &hazards,
                        double max_yaw_rate, double max_speed, double floor_speed,
                        double predict_s) {
    if (max_yaw_rate <= 0.0) return max_speed;
    double cap = max_speed;
    const double cos_yaw = std::cos(our_pose.yaw);
    const double sin_yaw = std::sin(our_pose.yaw);

    for (const auto &hazard : hazards) {
        const Pose2D center = predicted_center(hazard, predict_s);
        const double dx = center.x - our_pose.x;
        const double dy = center.y - our_pose.y;
        // Along-track and cross-track offsets in the robot's own frame.
        const double along = dx * cos_yaw + dy * sin_yaw;
        const double across = std::abs(-dx * sin_yaw + dy * cos_yaw);
        if (along <= kEps) continue;                     // behind us
        if (across >= hazard.inflated_radius) continue;  // the swept path misses it

        // Scale the clearance the turn has to produce by how far off the ray the centre sits: a
        // hazard grazed on one edge needs much less deviation than one dead ahead.
        const double needed = hazard.inflated_radius - across;
        if (needed <= kEps) continue;
        cap = std::min(cap, max_yaw_rate * along * along / (2.0 * needed));
    }
    return std::max(cap, floor_speed);
}

double hazard_brake_speed(const Pose2D &our_pose, const std::vector<FieldHazard> &hazards,
                          double brake_distance, double max_speed, double predict_s) {
    if (brake_distance <= 0.0) return max_speed;
    double limit = max_speed;
    const double cos_yaw = std::cos(our_pose.yaw);
    const double sin_yaw = std::sin(our_pose.yaw);

    for (const auto &hazard : hazards) {
        const Pose2D center = predicted_center(hazard, predict_s);
        const double dx = center.x - our_pose.x;
        const double dy = center.y - our_pose.y;
        // Same frame and the same directional gate the steering cap uses: only brake for a hazard
        // the swept path actually intrudes on, so passing one abeam costs nothing.
        const double along = dx * cos_yaw + dy * sin_yaw;
        const double across = std::abs(-dx * sin_yaw + dy * cos_yaw);
        if (along <= kEps) continue;                     // behind us
        if (across >= hazard.inflated_radius) continue;  // the swept path misses it

        // Along-track clearance to the keep-out rim, which is where the robot has to be stopped.
        // The radius is already inflated by our own half-diagonal and the static margin, so the
        // real hole sits well beyond this.
        const double clearance =
            along - std::sqrt(hazard.inflated_radius * hazard.inflated_radius - across * across);
        if (clearance >= brake_distance) continue;
        limit = std::min(limit, max_speed * std::max(0.0, clearance) / brake_distance);
    }
    return limit;
}

}  // namespace auto_battlebot
