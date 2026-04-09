#include "target_selector/nearest_target.hpp"

#include <cmath>
#include <limits>

#include "transform_utils.hpp"

namespace auto_battlebot {
namespace {
bool is_our_robot(Label label) {
    switch (label) {
        case Label::MR_STABS_MK1:
        case Label::MR_STABS_MK2:
        case Label::MRS_BUFF_MK1:
        case Label::MRS_BUFF_MK2:
            return true;
        default:
            return false;
    }
}

bool is_target_robot(Label label) { return label == Label::OPPONENT || label == Label::HOUSE_BOT; }
}  // namespace

std::optional<TargetSelection> NearestTarget::get_target(const RobotDescriptionsStamped &robots,
                                                         const FieldDescription &) {
    std::optional<Pose2D> our_pose;
    for (const auto &robot : robots.descriptions) {
        if (is_our_robot(robot.label)) {
            our_pose = pose_to_pose2d(robot.pose);
            break;
        }
    }
    if (!our_pose.has_value()) return std::nullopt;

    std::optional<TargetSelection> best;
    double best_dist = std::numeric_limits<double>::max();
    for (const auto &robot : robots.descriptions) {
        if (!is_target_robot(robot.label)) continue;
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
