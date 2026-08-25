#pragma once

#include <memory>
#include <optional>

#include "data_structures/field.hpp"
#include "data_structures/pose.hpp"
#include "data_structures/velocity.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"

namespace auto_battlebot {

/** Settings for the four hazard layers, copied out of whichever navigation config owns them so
 *  both controllers run the identical behaviour. */
struct HazardAvoidanceSettings {
    bool tangent_enable = true;
    int tangent_max_iterations = 3;
    double side_release_m = 0.05;
    double waypoint_clearance_m = 0.06;
    bool speed_cap_enable = true;
    double speed_cap_floor = 0.35;
    double prediction_horizon_s = 0.25;
    double reverse_distance = 0.12;
    double heading_threshold = 1.047;
    double reverse_min_speed = 0.35;
    double max_yaw_rate = 7.93;
    /** Clearance (m) to a blocking hazard's keep-out rim at which braking starts.
     * 0 disables. Ported from pursuit's brake_distance. */
    double brake_distance = 0.0;
};

/** The hazard layers that both PursuitNavigation and MotionProfileNavigation share.
 *
 * Holds the only state the layers need: the latched pass side, so the robot does not re-pick a
 * side every tick and cut back across the hazard it was rounding.
 */
class HazardAvoidance {
   public:
    explicit HazardAvoidance(HazardAvoidanceSettings settings);

    /** Clear the latch. Call wherever the controller resets its own trajectory state. */
    void reset();

    /** Option 4: substitute a tangent waypoint when the straight run to `goal` enters a hazard.
     *  Returns `goal` unchanged when the run is clear or no tangent exists. */
    Pose2D steer_around(const Pose2D &our_pose, const Pose2D &goal, const FieldDescription &field);

    /** Option 3: cap forward speed to what the achievable turn radius can clear. Returns
     *  `speed` unchanged when nothing blocks the heading ray. */
    double cap_speed(const Pose2D &our_pose, const FieldDescription &field, double speed) const;

    /** Forward speed that still leaves room to stop at a blocking hazard's keep-out rim.
     *  Where `cap_speed` asks whether the robot can still steer clear, this asks whether it
     *  can still stop, so it bites earlier and falls off linearly rather than as L^2. */
    double brake_speed(const Pose2D &our_pose, const FieldDescription &field, double speed) const;

    /** Option 2: last-ditch reverse. Overrides `command.linear_x` when the robot is inside
     *  `reverse_distance` of a hazard edge and pointed at it. Returns true when it fired. */
    bool apply_reverse(const Pose2D &our_pose, const FieldDescription &field,
                       VelocityCommand &command) const;

    /** Whether the tangent layer substituted a waypoint on the last steer_around call, and
     *  which side it committed to. For diagnostics. */
    bool last_substituted() const { return last_substituted_; }
    int committed_side() const { return committed_side_; }

   private:
    HazardAvoidanceSettings settings_;
    int committed_side_ = 0;
    /** Index of the hazard the side latch belongs to. A different hazard gets a fresh choice. */
    size_t committed_hazard_ = 0;
    bool last_substituted_ = false;
};

}  // namespace auto_battlebot
