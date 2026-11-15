#pragma once

#include <memory>
#include <optional>

#include "diagnostics_logger/diagnostics_module_logger.hpp"
#include "enums/behavior_mode.hpp"
#include "navigation/config.hpp"
#include "navigation/hazard_avoidance.hpp"
#include "time/clock_interface.hpp"

namespace auto_battlebot {
/**
 * @brief Motion-profiled tracker with inverse-plant feedforward and speed feedback.
 *
 * Stage 3 control law (control_improvement_plan.md, option 2). For a first-order drivetrain it
 * drives to the goal and arrives at a commanded terminal velocity without overrunning:
 *
 * The terminal velocity is per behavior mode: ATTACK drives through the opponent at full speed,
 * RUN_AWAY arrives stopped at the safe point. Both are configurable, and a mode flip resets the
 * trajectory state because the mission changed.
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

    static double compensate_coupling(double command, double effect, double coeff, double floor);

   private:
    std::optional<RobotDescription> find_our_robot(const RobotDescriptionsStamped &robots) const;

    VelocityCommand compute_command(const Pose2D &our_pose, const Pose2D &target_pose,
                                    const FieldDescription &field,
                                    std::optional<double> measured_speed, BehaviorMode mode);

    /**
     * @brief Terminal speed (m/s) for the driver's current mode: the configured fraction,
     * clamped to [0, 1], times the plant's k_fwd. Normalized in config so a refit rescales
     * it instead of leaving a hand-copied speed stale.
     */
    double terminal_velocity_for(BehaviorMode mode) const;

    /**
     * @brief Our signed forward speed (m/s): the filter's field-frame velocity projected onto our
     * heading. Returns nullopt when the estimate carries no measurement, which is the case on a
     * stale track (the filter is dead-reckoning our own command) or when our robot is missing
     * entirely.
     */
    std::optional<double> measure_forward_speed(const RobotDescription &robot,
                                                const Pose2D &our_pose) const;

    double compute_reference_speed(double distance, double dt, double terminal_velocity) const;

    double compute_linear_command(double v_ref, double dvdt, double v_actual, double dt,
                                  bool speed_is_measured);

    double apply_hysteresis(double angle_error);

    double compute_yaw_reference(double angle_error, double now_s);

    double compute_angular_command(double w_ref, double dt);

    void override_linear_with_wall_reverse(const Pose2D &our_pose, const FieldDescription &field,
                                           VelocityCommand &cmd) const;

    Pose2D clamp_to_field(const Pose2D &pose, const FieldDescription &field) const;

    static double distance_to_nearest_wall(const Pose2D &pose, const FieldDescription &field);
    static double wall_facing_angle(const Pose2D &pose, const FieldDescription &field);
    static double normalize_angle(double angle);

    void reset_state();

    /**
     * @brief Clear the trajectory and integrator state that only makes sense within one mission:
     * turn commitment, reference speeds, the speed integral, and the coupling history. Runs when
     * a zero-velocity goal is reached and when the behavior mode flips, both of which start a new
     * mission. Leaves the pose and speed memory alone, since those describe the robot rather than
     * the mission.
     */
    void reset_trajectory_state();

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

    double attack_terminal_speed_fraction_;
    double run_away_terminal_speed_fraction_;
    double stop_distance_;
    HazardAvoidance hazards_;

    double speed_kp_;
    double speed_ki_;

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

    double boundary_margin_;
    double max_linear_command_;
    double wall_reverse_distance_;
    double wall_reverse_min_speed_;
    double wall_heading_threshold_;

    std::shared_ptr<DiagnosticsModuleLogger> logger_;
    std::shared_ptr<ClockInterface> clock_;

    int committed_turn_sign_ = 0;
    double prev_angle_error_ = 0.0;
    double prev_angular_timestamp_ = 0.0;
    double prev_v_ref_ = 0.0;
    double speed_integral_ = 0.0;
    double prev_speed_timestamp_ = 0.0;
    std::optional<Pose2D> last_known_our_pose_;
    Position2D our_footprint_center_offset_;
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
    BehaviorMode last_tick_mode_ = BehaviorMode::ATTACK;
    NavigationVisualization last_visualization_;
};

}  // namespace auto_battlebot
