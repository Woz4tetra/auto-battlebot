#pragma once

#include <cmath>
#include <optional>
#include <vector>

#include "data_structures/field.hpp"
#include "data_structures/pose.hpp"
#include "data_structures/robot.hpp"
#include "transform_utils.hpp"

namespace auto_battlebot {

/** Shared hazard geometry, used by both navigation implementations so a fix lands once.
 *
 * Every routine here treats a hazard as an already-inflated disc (see FieldHazard): "inside the
 * radius" means the robot is in trouble, with no further clearance arithmetic. */

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

/** Move `pose` to the nearest point outside every hazard, then back inside the field rectangle.
 *
 * Overlapping discs can push a point out of one and into another, so this iterates a few times
 * and gives up rather than looping: a goal that cannot be made legal is left at the least-bad
 * spot, which the navigation will still drive toward while the reverse backstop covers it.
 *
 * `half_x` and `half_y` are the clamp bounds already reduced by the navigation's boundary
 * margin; pass them as the navigation computes them so the two clamps agree.
 */
Pose2D push_out_of_hazards(const Pose2D &pose, const std::vector<FieldHazard> &hazards,
                           double half_x, double half_y);

/** Where a hazard blocks a straight run from `start` to `goal`.
 *
 * Returns the hazard nearest to `start` whose disc the segment enters, or nothing if the run is
 * clear. `predict_s` advances a TRACKED hazard along its filtered velocity before the test, so
 * the answer is about where the house bot will be when we get there, not where it is now.
 */
std::optional<size_t> first_blocking_hazard(const Pose2D &start, const Pose2D &goal,
                                            const std::vector<FieldHazard> &hazards,
                                            double predict_s, const std::vector<size_t> &skip = {});

/** Hazard centre advanced by its own velocity over `seconds`. STATIC hazards are returned
 *  unchanged, whatever their velocity field happens to hold. */
Pose2D predicted_center(const FieldHazard &hazard, double seconds);

struct TangentWaypoint {
    Pose2D point;
    int side = 0;  // +1 = pass on the left of the hazard, -1 = right
    bool inside = false;
};
TangentWaypoint tangent_waypoint(const Pose2D &start, const Pose2D &center, double radius,
                                 double heading, int committed_side);

}  // namespace auto_battlebot
