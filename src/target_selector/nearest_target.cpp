#include "target_selector/nearest_target.hpp"

#include <cmath>
#include <limits>

#include "enums/frame_id.hpp"
#include "transform_utils.hpp"

namespace auto_battlebot {

std::optional<TargetSelection> NearestTarget::get_target(const RobotDescriptionsStamped &robots,
                                                         const FieldDescription &) {
    std::optional<Pose2D> our_pose;
    for (const auto &robot : robots.descriptions) {
        if (robot.frame_id == FrameId::OUR_ROBOT_1) {
            our_pose = pose_to_pose2d(robot.pose);
            break;
        }
    }
    if (!our_pose.has_value()) return std::nullopt;

    std::optional<TargetSelection> best;
    double best_dist = std::numeric_limits<double>::max();
    for (const auto &robot : robots.descriptions) {
        if (robot.group != Group::THEIRS) continue;
        const Pose2D target_pose = pose_to_pose2d(robot.pose);
        const double dx = target_pose.x - our_pose->x;
        const double dy = target_pose.y - our_pose->y;
        const double dist = std::sqrt(dx * dx + dy * dy);
        if (dist < best_dist) {
            best_dist = dist;
            best = TargetSelection{target_pose};
        }
    }
    return best;
}
}  // namespace auto_battlebot
