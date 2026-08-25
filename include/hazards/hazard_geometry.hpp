#pragma once

#include <cmath>
#include <optional>
#include <vector>

#include "data_structures/field.hpp"
#include "data_structures/pose.hpp"
#include "data_structures/robot.hpp"

namespace auto_battlebot {

/** Shared hazard geometry, used by both navigation implementations so a fix lands once.
 *
 * Every routine here treats a hazard as an already-inflated disc (see FieldHazard): "inside the
 * radius" means the robot is in trouble, with no further clearance arithmetic. */

/** Half the diagonal of a footprint, which is the radius of the smallest disc containing it.
 *
 * Using the diagonal rather than half the length keeps the keep-out conservative for a robot
 * approaching a hazard corner-first. The UI draws this same circle per robot so the inflation in
 * a keep-out ring is visible rather than implicit: a tracked ring is this for the hazard, plus
 * this for our own robot, plus the configured margin.
 */
inline double half_diagonal(const Size &size) {
    return 0.5 * std::sqrt(size.x * size.x + size.y * size.y);
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

/** A waypoint that steers around a hazard instead of at the true goal.
 *
 * The point is the tangency point on a circle of `radius` around `center`, which is what the
 * robot should drive at: once it arrives, the run from there to the true goal is tangent to that
 * circle and no longer blocked, so the goal comes back on its own. Pass a `radius` a little
 * larger than the keep-out so the waypoint leaves real clearance -- aiming at the keep-out rim
 * itself means arriving with zero clearance and grinding along the edge.
 *
 * Picks the side that costs less heading change from `heading`, so the robot does not cut back
 * across the hazard it committed to rounding. `committed_side` latches that choice: pass the
 * previous return's side back in (+1 or -1) to keep it, or 0 to choose freshly.
 *
 * Inside the circle there is no tangent, so the waypoint points straight out along the radius.
 * That is the only useful direction there, and the reverse backstop covers the rest.
 */
struct TangentWaypoint {
    Pose2D point;
    int side = 0;  // +1 = pass on the left of the hazard, -1 = right
    bool inside = false;
};
TangentWaypoint tangent_waypoint(const Pose2D &start, const Pose2D &center, double radius,
                                 double heading, int committed_side);

/** Speed cap that leaves enough turning authority to clear a hazard ahead.
 *
 * An arc starting tangent to the current heading deviates laterally by about L^2 / (2 r) at
 * along-track distance L, and the achievable radius is v / max_yaw_rate, so clearing a hazard of
 * radius R at range L needs v <= max_yaw_rate * L^2 / (2 R). Braking cannot be the primary
 * response on this plant -- full speed needs 0.86 m to stop against a 0.62 m turn-out radius --
 * so this caps speed only as much as the turn requires.
 *
 * `R` is scaled by how far off the heading ray the hazard centre sits, so a hazard well off to
 * one side does not throttle a clean run past it. Returns `max_speed` when nothing is in the
 * way, and never returns below `floor_speed`: a cap that reaches zero recreates the
 * stop-in-time behaviour the plant cannot deliver and strands the robot with the hazard in
 * front of it.
 */
double hazard_speed_cap(const Pose2D &our_pose, const std::vector<FieldHazard> &hazards,
                        double max_yaw_rate, double max_speed, double floor_speed,
                        double predict_s);

/** Forward speed limit that brings the robot to a stop at a blocking hazard's keep-out rim.
 *
 * Ported from PursuitNavigation's `brake_distance`, which ramps the commanded speed linearly to
 * zero inside a fixed distance of the target so the robot decelerates into it instead of driving
 * in at full speed and coasting past. That schedule is what made pursuit look better than the
 * motion profile near a hole: the motion profile's own brake is keyed to the goal, and in ATTACK
 * its terminal speed is full speed, so nothing slowed it for the hazard.
 *
 * Distinct from `hazard_speed_cap`, which asks "can I still steer clear" and grows as L^2. This
 * asks "can I still stop", and is linear in the clearance, so it bites earlier and harder.
 *
 * @param brake_distance Clearance (m) at which braking starts. 0 disables. It should cover the
 *   stopping lead: the distance travelled during actuation latency plus the coast that follows.
 * @return `max_speed`, or less when a hazard lies inside `brake_distance` along the swept path.
 */
double hazard_brake_speed(const Pose2D &our_pose, const std::vector<FieldHazard> &hazards,
                          double brake_distance, double max_speed, double predict_s);

}  // namespace auto_battlebot
