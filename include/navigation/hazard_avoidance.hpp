#pragma once

#include <limits>

#include "data_structures/field.hpp"
#include "data_structures/pose.hpp"
#include "data_structures/velocity.hpp"

namespace auto_battlebot {

/** Settings for the hazard layers, copied out of whichever navigation config owns them so
 *  both controllers run the identical behaviour. */
struct HazardAvoidanceSettings {
    bool tangent_enable = true;
    int tangent_max_iterations = 3;
    double side_release_m = 0.05;
    double waypoint_clearance_m = 0.06;
    double prediction_horizon_s = 0.25;

    // Last-ditch reverse. PursuitNavigation's only speed response; the motion profile's barrier
    // below supersedes it there.
    double reverse_distance = 0.12;
    double heading_threshold = 1.047;
    double reverse_min_speed = 0.35;

    // Velocity barrier, filled from the plant fit by MotionProfileNavigation. Zero speeds leave
    // the barrier off, which is what a controller without a fit gets.
    bool barrier_enable = true;
    double max_linear_speed_fwd = 0.0;
    double max_linear_speed_rev = 0.0;
    /** Stopping horizon (s): tau_lin_d + delay_s, the same first-order envelope the goal brake
     * schedule inverts. */
    double barrier_horizon_s = 0.0;
    double barrier_delay_s = 0.0;
};

/** What the barrier did to this tick's command, for diagnostics. */
struct HazardBarrierResult {
    /** The barrier changed linear_x this tick. */
    bool engaged = false;
    /** The measured approach speed exceeded the stopping envelope somewhere, so the change was
     * an active brake or reverse rather than a clip of the requested thrust. */
    bool braking = false;
    /** Tightest approach-speed bound (m/s) any hazard imposed. Negative means the robot is
     * inside a hard radius and must retreat. Infinity when nothing constrained. */
    double bound_mps = std::numeric_limits<double>::infinity();
};

/** The hazard layers that PursuitNavigation and MotionProfileNavigation share.
 *
 * Two aim layers and one speed rule. The aim layers (steer_around, plus the goal push-out the
 * controllers run in clamp_to_field) plan against the fat `inflated_radius`. The speed rule
 * (limit_command) is a directional velocity barrier against the thin `hard_radius`: per hazard,
 * the velocity component toward it may not exceed gap / (tau_decel + delay), which is exactly
 * the stopping envelope the plant can follow. One rule replaces the old steering speed cap, the
 * stop-at-rim brake, and the fixed hazard reverse, and it composes across hazards: each hazard
 * only constrains the velocity component toward itself, so backing away from one is
 * automatically speed-limited by the one behind.
 *
 * Holds the only cross-tick state the layers need: the latched pass side, so the robot does not
 * re-pick a side every tick and cut back across the hazard it was rounding.
 */
class HazardAvoidance {
   public:
    explicit HazardAvoidance(HazardAvoidanceSettings settings);

    /** Clear the latch. Call wherever the controller resets its own trajectory state. */
    void reset();

    /** Substitute a tangent waypoint when the straight run to `goal` enters a hazard.
     *  Returns `goal` unchanged when the run is clear or no tangent exists. */
    Pose2D steer_around(const Pose2D &our_pose, const Pose2D &goal, const FieldDescription &field);

    /** The velocity barrier: clamp `command.linear_x` so the commanded and measured approach
     * speed toward every hazard stays inside the stopping envelope to its hard radius.
     *
     * Applied to the FINAL command, after the angle gate and the wall reverse, against the
     * measured forward speed `v_actual` (m/s, signed along heading; pass the same held value the
     * speed loop uses so a stale track does not feed the barrier garbage). This placement is the
     * point: the 2026-08-28 AER recording showed a v_ref-side brake never reaches the plant when
     * the angle gate zeroes the channel, and the robot coasts into the hole with the brake
     * logged as active.
     *
     * Three behaviours fall out of the one rule:
     *  - approaching a hazard, the allowed thrust tapers with the remaining gap, so wheels are
     *    never spun up to a speed the gap cannot absorb (the blocked-push failure);
     *  - coasting or spun past the goal, measured excess over the envelope commands a
     *    proportional recovery AWAY from the hazard with full authority, whichever way the
     *    robot faces: a hazard ahead gets reverse, a hazard behind gets forward drive. The
     *    sign follows the hazard's bearing, never the robot's forward axis, so rear-first
     *    toward a hole is answered with forward, not with backing further in;
     *  - inside the hard radius the bound is negative, which is the old reverse backstop
     *    restated: an exit command scaled by depth, directed away from the centre, and limited
     *    by whatever other hazard lies on the exit path.
     */
    HazardBarrierResult limit_command(const Pose2D &our_pose, const FieldDescription &field,
                                      double v_actual, VelocityCommand &command) const;

    /** Last-ditch fixed reverse. Overrides `command.linear_x` when the robot is inside
     * `reverse_distance` of a hazard edge and pointed at it. Returns true when it fired.
     * PursuitNavigation only: the barrier supersedes this on the motion profile. */
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
