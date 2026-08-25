#include "navigation/hazard_avoidance.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "hazards/hazard_geometry.hpp"

namespace auto_battlebot {
namespace {
double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}
}  // namespace

HazardAvoidance::HazardAvoidance(HazardAvoidanceSettings settings) : settings_(settings) {}

void HazardAvoidance::reset() {
    committed_side_ = 0;
    committed_hazard_ = 0;
    last_substituted_ = false;
}

Pose2D HazardAvoidance::steer_around(const Pose2D &our_pose, const Pose2D &goal,
                                     const FieldDescription &field) {
    last_substituted_ = false;
    if (!settings_.tangent_enable || field.hazards.empty()) {
        committed_side_ = 0;
        return goal;
    }

    Pose2D waypoint = goal;
    std::vector<size_t> handled;
    for (int iteration = 0; iteration < std::max(1, settings_.tangent_max_iterations);
         ++iteration) {
        const auto blocking = first_blocking_hazard(our_pose, waypoint, field.hazards,
                                                    settings_.prediction_horizon_s, handled);
        if (!blocking.has_value()) break;

        const FieldHazard &hazard = field.hazards[*blocking];
        const Pose2D center = predicted_center(hazard, settings_.prediction_horizon_s);

        // The latch belongs to one hazard. Rounding a different one is a fresh decision.
        const int side =
            (last_substituted_ || committed_hazard_ != *blocking) ? 0 : committed_side_;
        // Tangent to a circle a little wider than the keep-out, so arriving at the waypoint
        // leaves real clearance and the run from there to the goal is unblocked against the
        // keep-out itself. Tangent to the keep-out exactly would put the waypoint on the rim.
        const TangentWaypoint tangent = tangent_waypoint(
            our_pose, center, hazard.radius + settings_.waypoint_clearance_m, our_pose.yaw, side);

        // A hazard already routed around is excluded from the next pass. The new segment runs
        // tangent to it by construction, so re-testing it only produces a substitution loop.
        handled.push_back(*blocking);
        if (!tangent.inside) {
            committed_hazard_ = *blocking;
            committed_side_ = tangent.side;
        }
        last_substituted_ = true;
        waypoint = tangent.point;
    }

    if (!last_substituted_ && committed_side_ != 0) {
        // Release the latch only once the robot is clear of every hazard by the release margin,
        // not the instant the segment stops intersecting: at the boundary the test flickers and
        // the robot cuts back across what it was rounding.
        double nearest = std::numeric_limits<double>::infinity();
        for (const auto &hazard : field.hazards) {
            const Pose2D center = predicted_center(hazard, settings_.prediction_horizon_s);
            nearest = std::min(
                nearest, std::hypot(our_pose.x - center.x, our_pose.y - center.y) - hazard.radius);
        }
        if (nearest > settings_.side_release_m) committed_side_ = 0;
    }
    return waypoint;
}

double HazardAvoidance::cap_speed(const Pose2D &our_pose, const FieldDescription &field,
                                  double speed) const {
    if (!settings_.speed_cap_enable || field.hazards.empty()) return speed;
    return hazard_speed_cap(our_pose, field.hazards, settings_.max_yaw_rate, speed,
                            std::min(speed, settings_.speed_cap_floor),
                            settings_.prediction_horizon_s);
}

double HazardAvoidance::brake_speed(const Pose2D &our_pose, const FieldDescription &field,
                                    double speed) const {
    if (settings_.brake_distance <= 0.0 || field.hazards.empty()) return speed;
    return hazard_brake_speed(our_pose, field.hazards, settings_.brake_distance, speed,
                              settings_.prediction_horizon_s);
}

bool HazardAvoidance::apply_reverse(const Pose2D &our_pose, const FieldDescription &field,
                                    VelocityCommand &command) const {
    if (settings_.reverse_distance <= 0.0) return false;

    for (const auto &hazard : field.hazards) {
        const Pose2D center = predicted_center(hazard, settings_.prediction_horizon_s);
        const double dx = center.x - our_pose.x;
        const double dy = center.y - our_pose.y;
        const double gap = std::hypot(dx, dy) - hazard.radius;
        if (gap >= settings_.reverse_distance) continue;

        const double heading_err = std::abs(normalize_angle(std::atan2(dy, dx) - our_pose.yaw));
        if (heading_err >= settings_.heading_threshold) continue;

        command.linear_x =
            -std::max(std::abs(command.linear_x), std::abs(settings_.reverse_min_speed));
        return true;
    }
    return false;
}

}  // namespace auto_battlebot
