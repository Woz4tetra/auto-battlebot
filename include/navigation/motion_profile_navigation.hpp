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
 *     dividing by max speed, adds tau_accel * dv/dt to overcome spin-up lag, divides out the
 *     steer-brake authority the turn is costing it, and applies exact inverse-deadzone.
 *  3. Speed PID feedback. Corrects plant-model error using our filtered speed, so open-loop
 *     feedforward accuracy and disturbance rejection stay separated. The feedback is dropped
 *     while our track is stale, because a dead-reckoned speed is a function of our own command
 *     and closing a loop on it is not feedback.
 *  4. Angular control in the same shape: turn-lock hysteresis, a PD on heading producing a
 *     yaw-rate reference in rad/s, then an inverse-plant feedforward through max_angular_speed
 *     with the regime-appropriate yaw time constant and the angular droop divided out.
 *
 * The two coupling compensations are mutually recursive: the droop term reads the linear command
 * and the steer-brake term reads the turn command. Each is driven by the previous tick's
 * UNCOMPENSATED counterpart, which breaks the loop. Feeding the compensated values back instead
 * has a loop gain above 1 and saturates both channels within a few ticks.
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

    // Pure functions of their arguments, public so the plant algebra can be tested directly
    // instead of inferred from a whole update() call.

    /**
     * @brief Divide out a coupling loss of the form (1 - coeff*|effect|), floored. Returns the
     * command scaled back up to the size the plant will actually deliver.
     */
    static double compensate_coupling(double command, double effect, double coeff, double floor);

    /** @brief Deadzone removal per sign, rescaled so full command still maps to full effect. */
    static double effective_command(double command, double deadzone_pos, double deadzone_neg);

   private:
    /** @brief Find our robot in the robot descriptions. */
    std::optional<RobotDescription> find_our_robot(const RobotDescriptionsStamped &robots) const;

    /** @brief Compute the velocity command that tracks the goal. */
    VelocityCommand compute_command(const Pose2D &our_pose, const Pose2D &target_pose,
                                    const FieldDescription &field,
                                    std::optional<double> measured_speed);

    /**
     * @brief Our signed forward speed (m/s): the filter's field-frame velocity projected onto our
     * heading. Returns nullopt when the estimate carries no measurement, which is the case on a
     * stale track (the filter is dead-reckoning our own command) or when our robot is missing
     * entirely.
     */
    std::optional<double> measure_forward_speed(const RobotDescription &robot,
                                                const Pose2D &our_pose) const;

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
    double compute_linear_command(double v_ref, double dvdt, double v_actual, double dt,
                                  bool speed_is_measured);

    /** @brief Turn-direction hysteresis on the raw heading error (see PursuitNavigation). */
    double apply_hysteresis(double angle_error);

    /**
     * @brief Yaw-rate reference (rad/s) from the capped PD on heading error. Read-only apart from
     * the derivative state.
     */
    double compute_yaw_reference(double angle_error, double now_s);

    /**
     * @brief Map a yaw-rate reference to a normalized turn command: inverse-plant feedforward
     * through max_angular_speed with the regime tau, then divide out the angular droop the
     * current linear command is causing.
     */
    double compute_angular_command(double w_ref, double dt);

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
    double steer_brake_coeff_;
    double steer_brake_floor_;
    double angular_deadzone_left_;
    double angular_deadzone_right_;

    // Trajectory.
    double terminal_velocity_;
    double stop_distance_;

    // Speed feedback.
    double speed_kp_;
    double speed_ki_;

    // Angular control.
    double max_angular_speed_;
    double tau_angular_accel_;
    double tau_angular_decel_;
    double angular_droop_coeff_;
    double angular_droop_floor_;
    double angular_kp_;
    double angular_kd_;
    double angle_threshold_;
    double max_yaw_rate_;
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
    double prev_speed_timestamp_ = 0.0;
    std::optional<Pose2D> last_known_our_pose_;
    /** Last speed that came from a measurement, held through stale stretches so the latency lead
     * keeps working while the speed feedback is off. */
    double last_measured_speed_ = 0.0;
    double prev_w_ref_ = 0.0;
    /**
     * Previous tick's commands BEFORE coupling compensation, which is what each channel's coupling
     * term reads. Using the compensated values instead closes a positive feedback loop between the
     * two compensations whose measured gain is above 1: both channels saturate within a few ticks.
     */
    double prev_linear_uncompensated_ = 0.0;
    double prev_angular_uncompensated_ = 0.0;
    NavigationVisualization last_visualization_;
};

}  // namespace auto_battlebot
