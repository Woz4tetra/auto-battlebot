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
            our_pose, center, hazard.inflated_radius + settings_.waypoint_clearance_m, our_pose.yaw,
            side);

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
            nearest = std::min(nearest, std::hypot(our_pose.x - center.x, our_pose.y - center.y) -
                                            hazard.inflated_radius);
        }
        if (nearest > settings_.side_release_m) committed_side_ = 0;
    }
    return waypoint;
}

HazardBarrierResult HazardAvoidance::limit_command(const Pose2D &our_pose,
                                                   const FieldDescription &field, double v_actual,
                                                   VelocityCommand &command) const {
    HazardBarrierResult result;
    if (!settings_.barrier_enable || field.hazards.empty()) return result;
    if (settings_.max_linear_speed_fwd <= 0.0 || settings_.barrier_horizon_s <= 0.0) return result;

    // Gain from measured excess approach speed (m/s) to an opposing normalized command. 1.5
    // recovers the 2026-08-28 recorded push-into-hole coast with margin in the counterfactual
    // sim; the envelope decides where braking starts, this only decides how hard the recovery
    // pulls once the envelope is already violated.
    constexpr double kExcessGain = 1.5;
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
        if (distance < kEps) continue;  // dead centre: no direction to limit against
        const double gap = distance - hazard.hard_radius;
        // Fraction of forward motion that approaches this hazard: +1 dead ahead, -1 dead
        // behind. A diff drive only translates along its heading, so this is the whole story.
        const double toward = (cos_yaw * dx + sin_yaw * dy) / distance;
        if (std::abs(toward) < kEps) continue;  // passing abeam costs nothing

        const double v_approach = v_actual * toward;
        // The approach speed a first-order plant can still shed by the hard rim, with the same
        // latency lead the goal brake schedule uses: the command bites delay_s late, after the
        // robot has already closed v*delay of the gap.
        const double bound = (gap - std::max(v_approach, 0.0) * settings_.barrier_delay_s) /
                             settings_.barrier_horizon_s;
        const double excess = v_approach - bound;

        // Constraint on the commanded wheel speed: u * k * toward <= bound. Solving for u gives
        // an upper limit when the hazard is ahead and a lower limit when it is behind, which is
        // the rear-clearance check falling out of the same rule. The gain k depends on the sign
        // of the binding command, since reverse is the weaker drive.
        if (toward > 0.0) {
            const double k = (bound >= 0.0) ? settings_.max_linear_speed_fwd
                                            : std::max(settings_.max_linear_speed_rev, kEps);
            u_max = std::min(u_max, bound / (k * toward));
            if (excess > 0.0) {
                u_max = std::min(u_max, -std::min(1.0, kExcessGain * excess));
                result.braking = true;
            }
        } else {
            const double k = (bound >= 0.0) ? std::max(settings_.max_linear_speed_rev, kEps)
                                            : settings_.max_linear_speed_fwd;
            u_min = std::max(u_min, bound / (k * toward));
            if (excess > 0.0) {
                u_min = std::max(u_min, std::min(1.0, kExcessGain * excess));
                result.braking = true;
            }
        }
        result.bound_mps = std::min(result.bound_mps, bound);
    }

    double limited;
    if (u_min > u_max) {
        // Squeezed between hazards ahead and behind: no command satisfies both envelopes. Split
        // the violation instead of picking a side and diving into the other one; with symmetric
        // pressure this stops the robot, which is the least-bad answer available.
        limited = 0.5 * (u_min + u_max);
    } else {
        limited = std::clamp(command.linear_x, u_min, u_max);
    }
    if (limited != command.linear_x) {
        command.linear_x = limited;
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
        const double gap = std::hypot(dx, dy) - hazard.inflated_radius;
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
