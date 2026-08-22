#include "target_selector/safest_point_target.hpp"

#include "target_selector/empty_circle_solver.hpp"
#include "transform_utils.hpp"

namespace auto_battlebot {

SafestPointTarget::SafestPointTarget(const SafestPointTargetConfiguration &config)
    : retarget_improvement_m_(config.retarget_improvement_m) {}

std::optional<TargetSelection> SafestPointTarget::get_target(const RobotDescriptionsStamped &robots,
                                                             const FieldDescription &field,
                                                             BehaviorMode mode) {
    if (mode == BehaviorMode::ATTACK) {
        // Clear on leaving RUN_AWAY, so re-entering starts fresh instead of resuming a stale
        // point.
        held_target_.reset();
        return attack_selector_.get_target(robots, field, mode);
    }

    // Hoist the opponent scan out of the solver so its hot path is pure arithmetic.
    std::vector<Pose2D> opponents;
    opponents.reserve(robots.descriptions.size());
    for (const auto &robot : robots.descriptions) {
        if (robot.group != Group::THEIRS) continue;
        opponents.push_back(pose_to_pose2d(robot.pose));
    }

    const std::optional<Pose2D> candidate = solve(field, opponents);
    if (candidate.has_value()) {
        if (!held_target_.has_value()) {
            held_target_ = candidate;
        } else {
            // Re-measure the held target each cycle so one gone bad is abandoned promptly, and
            // only retarget when the new candidate is a real improvement, not grid jitter.
            const double held_radius =
                circle_radius(held_target_->x, held_target_->y, field, opponents);
            const double candidate_radius =
                circle_radius(candidate->x, candidate->y, field, opponents);
            if (candidate_radius > held_radius + retarget_improvement_m_) {
                held_target_ = candidate;
            }
        }
    }

    if (!held_target_.has_value()) return std::nullopt;
    TargetSelection selection;
    selection.pose.x = held_target_->x;
    selection.pose.y = held_target_->y;
    // yaw stays 0.0: both navigation implementations ignore target yaw.
    return selection;
}

double SafestPointTarget::circle_radius(double x, double y, const FieldDescription &field,
                                        const std::vector<Pose2D> &opponents) const {
    return empty_circle_radius(field.size.size, x, y, opponents);
}

std::optional<Pose2D> SafestPointTarget::solve(const FieldDescription &,
                                               const std::vector<Pose2D> &) const {
    // Awaiting the solver pick from the run-away solver experiment; see the doc comment in
    // the header. Returning nullopt keeps RUN_AWAY on the previously selected target.
    return std::nullopt;
}

}  // namespace auto_battlebot
