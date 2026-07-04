#pragma once

#include <memory>
#include <optional>

#include "diagnostics_logger/diagnostics_module_logger.hpp"
#include "navigation/config.hpp"
#include "time/clock_interface.hpp"

namespace auto_battlebot {
/**
 * @brief Motion-profiled tracker with inverse-plant feedforward and speed feedback.
 *
 * Stage 3 control law (control_improvement_plan.md, option 2). For a first-order drivetrain it
 * drives to the goal and arrives at a commanded terminal velocity without overrunning:
 *
 *  1. Distance-to-go velocity profile. The residual travel after commanding v_term on a first-order
 *     plant is ~ v*(tau_decel + latency), so the reference speed is capped at
 *     v_term + d/(tau_decel + latency): a coast-aware brake schedule derived from the measured
 *     plant, not a hand-tuned brake distance. Ramp-up is rate-limited for a clean launch.
 *  2. Inverse-plant feedforward. Maps the reference speed (m/s) back to a normalized command by
 *     dividing by max speed, adds tau_accel * dv/dt to overcome spin-up lag, and applies exact
 *     inverse-deadzone.
 *  3. Speed PID feedback. Corrects plant-model error using our measured speed (finite difference of
 *     our pose), so open-loop feedforward accuracy and disturbance rejection stay separated.
 *
 * Angular control (turn-lock hysteresis + capped PD) matches PursuitNavigation so heading behaviour
 * is comparable; it is reimplemented here to keep the baseline untouched for A/B comparison.
 */
class MotionProfileNavigation : public NavigationInterface {
   public:
    MotionProfileNavigation(const MotionProfileNavigationConfiguration &config,
                            std::shared_ptr<ClockInterface> clock);

    bool initialize() override;

    VelocityCommand update(RobotDescriptionsStamped robots, FieldDescription field,
                           const TargetSelection &target) override;

    const NavigationVisualization &get_last_visualization() const override {
        return last_visualization_;
    }

   private:
    /** @brief Find our robot in the robot descriptions. */
    std::optional<RobotDescription> find_our_robot(const RobotDescriptionsStamped &robots) const;

    /** @brief Compute the velocity command that tracks the goal. */
    VelocityCommand compute_command(const Pose2D &our_pose, const Pose2D &target_pose,
                                    const FieldDescription &field);

    /**
     * @brief Estimate our signed forward speed (m/s) from the pose delta since the last tick,
     * projected onto the current heading. Read-only; returns 0 on the first tick or a bad dt.
     */
    double estimate_forward_speed(const Pose2D &our_pose, double now_s) const;

    /**
     * @brief Reference forward speed (m/s) from the distance-to-go coast-aware brake schedule,
     * rate-limited on ramp-up. Read-only (uses prev_v_ref_ as last tick's reference).
     */
    double compute_reference_speed(double distance, double v_actual, double dt) const;

    /**
     * @brief Map a reference speed (m/s) to a normalized linear command via inverse-plant
     * feedforward (steady-state + tau_accel*dvdt lag term) plus speed feedback, then apply
     * inverse-deadzone. Updates the speed integrator.
     */
    double compute_linear_command(double v_ref, double dvdt, double v_actual, double dt);

    /** @brief Turn-direction hysteresis on the raw heading error (see PursuitNavigation). */
    double apply_hysteresis(double angle_error);

    /** @brief Capped PD angular velocity (normalized) on the heading error. */
    double compute_angular_command(double angle_error, double now_s);

    /** @brief If close to and facing a wall, override cmd.linear_x with a reverse command. */
    void apply_wall_reverse(const Pose2D &our_pose, const FieldDescription &field,
                            VelocityCommand &cmd) const;

    /** @brief Clamp a position to stay within the field boundaries (minus boundary_margin). */
    Pose2D clamp_to_field(const Pose2D &pose, const FieldDescription &field) const;

    static double distance_to_nearest_wall(const Pose2D &pose, const FieldDescription &field);
    static double wall_facing_angle(const Pose2D &pose, const FieldDescription &field);
    static double normalize_angle(double angle);

    void reset_state();

    // Plant parameters.
    double max_linear_speed_fwd_;
    double max_linear_speed_rev_;
    double tau_accel_;
    double tau_decel_;
    double latency_;
    double deadzone_;
    double accel_limit_;

    // Trajectory.
    double terminal_velocity_;
    double stop_distance_;

    // Speed feedback.
    double speed_kp_;
    double speed_ki_;

    // Angular control.
    double angular_kp_;
    double angular_kd_;
    double angle_threshold_;
    double max_angular_command_;
    bool enable_hysteresis_;

    // Misc / safety.
    double boundary_margin_;
    double max_linear_command_;
    double wall_reverse_distance_;
    double wall_reverse_min_speed_;
    double wall_heading_threshold_;

    std::shared_ptr<DiagnosticsModuleLogger> logger_;
    std::shared_ptr<ClockInterface> clock_;

    // Per-tick state.
    int committed_turn_sign_ = 0;
    double prev_angle_error_ = 0.0;
    double prev_angular_timestamp_ = 0.0;
    double prev_v_ref_ = 0.0;
    double speed_integral_ = 0.0;
    std::optional<Pose2D> prev_our_pose_;
    double prev_speed_timestamp_ = 0.0;
    std::optional<Pose2D> last_known_our_pose_;
    NavigationVisualization last_visualization_;
};

}  // namespace auto_battlebot
