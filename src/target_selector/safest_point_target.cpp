#include "target_selector/safest_point_target.hpp"

#include "enums/frame_id.hpp"
#include "target_selector/empty_circle_solver.hpp"
#include "transform_utils.hpp"

namespace auto_battlebot {

SafestPointTarget::SafestPointTarget(const SafestPointTargetConfiguration &config)
    : logger_(DiagnosticsLogger::get_logger("safest_point_target")),
      retarget_improvement_m_(config.retarget_improvement_m) {}

std::optional<TargetSelection> SafestPointTarget::get_target(const RobotDescriptionsStamped &robots,
                                                             const FieldDescription &field,
                                                             BehaviorMode mode) {
    if (mode == BehaviorMode::ATTACK) {
        // Clear on leaving RUN_AWAY, so re-entering starts fresh instead of resuming a stale
        // point.
        held_target_.reset();
        return attack_selector_.get_target(robots, field, mode);
    }

    // Hoist the opponent scan out of the solver so its hot path is pure arithmetic. Our pose
    // only feeds the tie-break; unlike NearestTarget, run-away keeps producing answers
    // without it.
    std::vector<Pose2D> opponents;
    opponents.reserve(robots.descriptions.size());
    std::optional<Pose2D> our_pose;
    for (const auto &robot : robots.descriptions) {
        if (robot.group == Group::THEIRS) {
            opponents.push_back(pose_to_pose2d(robot.pose));
        } else if (robot.frame_id == FrameId::OUR_ROBOT_1) {
            our_pose = pose_to_pose2d(robot.pose);
        }
    }

    const std::optional<Pose2D> candidate = solve(field, opponents, our_pose);
    if (candidate.has_value()) {
        if (!held_target_.has_value()) {
            held_target_ = candidate;
        } else {
            // Re-measure the held target each cycle so one gone bad is abandoned promptly, and
            // only retarget when the new candidate is a real improvement, not solver jitter.
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
    // The radius next to the target is the one number that says whether the chosen spot is
    // actually open or merely the best of a bad set.
    logger_->debug("run_away_target",
                   {{"x", held_target_->x},
                    {"y", held_target_->y},
                    {"radius", circle_radius(held_target_->x, held_target_->y, field, opponents)},
                    {"n_opponents", static_cast<int>(opponents.size())}});
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

std::optional<Pose2D> SafestPointTarget::solve(const FieldDescription &field,
                                               const std::vector<Pose2D> &opponents,
                                               const std::optional<Pose2D> &our_pose) const {
    const EmptyCircle best = solve_exact(field.size.size, opponents, our_pose);
    // Nonpositive radius means the field rectangle itself is degenerate (no measurement
    // yet); hold the previous target upstream rather than steering at garbage.
    if (best.radius <= 0.0) return std::nullopt;
    return best.center;
}

}  // namespace auto_battlebot
