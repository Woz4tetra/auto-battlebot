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
    committed_side_ = PassSide::NONE;
    latched_hazard_index_ = 0;
    last_substituted_ = false;
}

Pose2D HazardAvoidance::steer_around(const Pose2D &our_pose, const Pose2D &goal,
                                     const FieldDescription &field) {
    last_substituted_ = false;
    if (!settings_.tangent_enable || field.hazards.empty()) {
        committed_side_ = PassSide::NONE;
        return goal;
    }

    Pose2D waypoint = goal;
    std::vector<size_t> routed_around;
    for (int iteration = 0; iteration < std::max(1, settings_.tangent_max_iterations);
         ++iteration) {
        const auto blocking = first_blocking_hazard(our_pose, waypoint, field.hazards,
                                                    settings_.prediction_horizon_s, routed_around);
        if (!blocking.has_value()) break;

        const FieldHazard &hazard = field.hazards[*blocking];
        const Pose2D center = predicted_center(hazard, settings_.prediction_horizon_s);

        const PassSide side = (last_substituted_ || latched_hazard_index_ != *blocking)
                                  ? PassSide::NONE
                                  : committed_side_;
        const TangentWaypoint tangent = tangent_waypoint(
            our_pose, center, hazard.inflated_radius_m + settings_.waypoint_clearance_m,
            our_pose.yaw, side);

        routed_around.push_back(*blocking);
        if (!tangent.inside) {
            latched_hazard_index_ = *blocking;
            committed_side_ = tangent.side;
        }
        last_substituted_ = true;
        waypoint = tangent.point;
    }

    if (!last_substituted_ && committed_side_ != PassSide::NONE) {
        double nearest = std::numeric_limits<double>::infinity();
        for (const auto &hazard : field.hazards) {
            const Pose2D center = predicted_center(hazard, settings_.prediction_horizon_s);
            nearest = std::min(nearest, std::hypot(our_pose.x - center.x, our_pose.y - center.y) -
                                            hazard.inflated_radius_m);
        }
        if (nearest > settings_.side_release_m) committed_side_ = PassSide::NONE;
    }
    return waypoint;
}

HazardBarrierResult HazardAvoidance::limit_command(const Pose2D &our_pose,
                                                   const FieldDescription &field, double v_actual,
                                                   VelocityCommand &command) const {
    HazardBarrierResult result;
    if (!settings_.barrier_enable || field.hazards.empty()) return result;
    if (settings_.max_linear_speed_fwd <= 0.0 || settings_.barrier_horizon_s <= 0.0) return result;

    constexpr double kEps = 1e-6;

    double u_min = -1.0;
    double u_max = 1.0;
    const double cos_yaw = std::cos(our_pose.yaw);
    const double sin_yaw = std::sin(our_pose.yaw);

    for (const auto &hazard : field.hazards) {
        const Pose2D center = predicted_center(hazard, settings_.prediction_horizon_s);
        const double dx = center.x - our_pose.x;
        const double dy = center.y - our_pose.y;
        const double distance = std::hypot(dx, dy);
        if (distance < kEps) continue;
        const double gap = distance - hazard.hard_radius;
        const double toward = (cos_yaw * dx + sin_yaw * dy) / distance;
        if (std::abs(toward) < kEps) continue;

        const double v_approach = v_actual * toward;
        const double bound = (gap - std::max(v_approach, 0.0) * settings_.barrier_delay_s) /
                             settings_.barrier_horizon_s;
        const double excess = v_approach - bound;

        if (toward > 0.0) {
            const double k = (bound >= 0.0) ? settings_.max_linear_speed_fwd
                                            : std::max(settings_.max_linear_speed_rev, kEps);
            u_max = std::min(u_max, bound / (k * toward));
            if (excess > 0.0) {
                u_max = std::min(u_max, -std::min(1.0, settings_.excess_gain * excess));
                result.braking = true;
            }
        } else {
            const double k = (bound >= 0.0) ? std::max(settings_.max_linear_speed_rev, kEps)
                                            : settings_.max_linear_speed_fwd;
            u_min = std::max(u_min, bound / (k * toward));
            if (excess > 0.0) {
                u_min = std::max(u_min, std::min(1.0, settings_.excess_gain * excess));
                result.braking = true;
            }
        }
        result.approach_speed_bound_mps = std::min(result.approach_speed_bound_mps, bound);
    }

    double limited;
    if (u_min > u_max) {
        limited = 0.5 * (u_min + u_max);
    } else {
        limited = std::clamp(command.forward_mps, u_min, u_max);
    }
    if (limited != command.forward_mps) {
        command.forward_mps = limited;
        result.engaged = true;
    }
    return result;
}

bool HazardAvoidance::apply_reverse(const Pose2D &our_pose, const FieldDescription &field,
                                    VelocityCommand &command) const {
    if (settings_.reverse_distance <= 0.0) return false;

    for (const auto &hazard : field.hazards) {
        const Pose2D center = predicted_center(hazard, settings_.prediction_horizon_s);
        const double dx = center.x - our_pose.x;
        const double dy = center.y - our_pose.y;
        const double gap = std::hypot(dx, dy) - hazard.inflated_radius_m;
        if (gap >= settings_.reverse_distance) continue;

        const double heading_err = std::abs(normalize_angle(std::atan2(dy, dx) - our_pose.yaw));
        if (heading_err >= settings_.heading_threshold) continue;

        command.forward_mps =
            -std::max(std::abs(command.forward_mps), std::abs(settings_.reverse_min_speed));
        return true;
    }
    return false;
}

}  // namespace auto_battlebot
