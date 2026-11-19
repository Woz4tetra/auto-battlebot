#pragma once

#include <cmath>
#include <optional>
#include <vector>

#include "data_structures/field.hpp"
#include "data_structures/pose.hpp"
#include "data_structures/robot.hpp"
#include "transform_utils.hpp"

namespace auto_battlebot {

inline double half_diagonal(const Size &size) {
    return 0.5 * std::sqrt(size.x * size.x + size.y * size.y);
}

inline Pose2D footprint_center(const Pose2D &pose, const Position2D &offset) {
    const double c = std::cos(pose.yaw);
    const double s = std::sin(pose.yaw);
    return Pose2D{pose.x + c * offset.x - s * offset.y, pose.y + s * offset.x + c * offset.y,
                  pose.yaw};
}

inline Pose2D footprint_center(const RobotDescription &robot) {
    return footprint_center(pose_to_pose2d(robot.pose), robot.footprint_center_offset);
}

Pose2D push_out_of_hazards(const Pose2D &pose, const std::vector<FieldHazard> &hazards,
                           double half_x, double half_y);

std::optional<size_t> first_blocking_hazard(const Pose2D &start, const Pose2D &goal,
                                            const std::vector<FieldHazard> &hazards,
                                            double predict_s, const std::vector<size_t> &skip = {});

Pose2D predicted_center(const FieldHazard &hazard, double seconds);

enum class PassSide { NONE = 0, LEFT = 1, RIGHT = -1 };

struct TangentWaypoint {
    Pose2D point;
    PassSide side = PassSide::NONE;
    bool inside = false;
};
TangentWaypoint tangent_waypoint(const Pose2D &start, const Pose2D &center,
                                 double inflated_radius_m, double heading, PassSide committed_side);

}  // namespace auto_battlebot
