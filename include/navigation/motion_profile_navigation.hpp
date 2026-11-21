#pragma once

#include <memory>
#include <optional>

#include "diagnostics_logger/diagnostics_module_logger.hpp"
#include "enums/behavior_mode.hpp"
#include "navigation/config.hpp"
#include "navigation/hazard_avoidance.hpp"
#include "time/clock_interface.hpp"

namespace auto_battlebot {
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

    double terminal_velocity_for(BehaviorMode mode) const;

    double measure_forward_speed(const RobotDescription &robot, const Pose2D &our_pose) const;

    double compute_reference_speed(double distance, double dt, double terminal_velocity) const;

    double compute_linear_command(double v_ref, double dvdt, double v_actual, double dt,
                                  bool speed_is_measured);

    double apply_hysteresis(double angle_error);

    double compute_yaw_reference(double angle_error, double now_s);

    double compute_angular_command(double w_ref, double dt);

    void override_linear_with_wall_reverse(const Pose2D &our_pose, const FieldDescription &field,
                                           StickCommand &stick) const;

    Pose2D clamp_to_field(const Pose2D &pose, const FieldDescription &field) const;

    static double distance_to_nearest_wall(const Pose2D &pose, const FieldDescription &field);
    static double wall_facing_angle(const Pose2D &pose, const FieldDescription &field);
    static double normalize_angle(double angle);

    void reset_state();

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
    double stop_distance_m_;
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
    std::optional<double> max_angular_command_;
    bool enable_hysteresis_;

    double boundary_margin_;
    int hazard_push_out_max_passes_;
    std::optional<double> max_linear_command_;
    std::optional<double> wall_reverse_distance_m_;
    double wall_reverse_min_speed_;
    double wall_heading_threshold_rad_;

    std::shared_ptr<DiagnosticsModuleLogger> logger_;
    std::shared_ptr<ClockInterface> clock_;
    JigPlantParams plant_;

    int committed_turn_sign_ = 0;
    double prev_angle_error_ = 0.0;
    double prev_angular_timestamp_ = 0.0;
    double prev_v_ref_ = 0.0;
    double speed_integral_ = 0.0;
    double prev_speed_timestamp_ = 0.0;
    std::optional<Pose2D> last_known_our_pose_;
    Position2D our_footprint_center_offset_;
    double last_measured_speed_ = 0.0;
    double prev_w_ref_ = 0.0;
    double prev_linear_uncompensated_ = 0.0;
    double prev_angular_uncompensated_ = 0.0;
    BehaviorMode last_tick_mode_ = BehaviorMode::ATTACK;
    NavigationVisualization last_visualization_;
};

}  // namespace auto_battlebot
