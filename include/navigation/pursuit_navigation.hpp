#pragma once

#include <memory>
#include <optional>

#include "diagnostics_logger/diagnostics_module_logger.hpp"
#include "navigation/config.hpp"
#include "navigation/hazard_avoidance.hpp"
#include "time/clock_interface.hpp"

namespace auto_battlebot {
class PursuitNavigation : public NavigationInterface {
   public:
    PursuitNavigation(const PursuitNavigationConfiguration &config,
                      std::shared_ptr<ClockInterface> clock);

    bool initialize() override;

    VelocityCommand update(RobotDescriptionsStamped robots, FieldDescription field,
                           const TargetSelection &target) override;

    const NavigationVisualization &get_last_visualization() const override {
        return last_visualization_;
    }

   private:
    std::optional<RobotDescription> find_our_robot(const RobotDescriptionsStamped &robots) const;

    VelocityCommand compute_pursuit_command(const Pose2D &our_pose, const Pose2D &target_pose,
                                            const FieldDescription &field);

    /**
     * @brief Apply turn-direction hysteresis to the raw angle error.
     *
     * When the target swings far behind us (above a commit threshold), latches a turn direction
     * to prevent dithering. The latch releases once we are roughly facing the target again.
     * Updates committed_turn_sign_ as a side effect and returns the (possibly sign-overridden)
     * angle error.
     */
    double apply_hysteresis(double angle_error);

    double compute_angular_velocity_rad_s(double angle_error);

    /**
     * @brief Compute forward linear velocity (m/s).
     *
     * Returns 0 unless we are roughly facing the target. Otherwise scales speed by a velocity
     * ramp (full speed near the target, reduced at distance), an angle-error penalty, and a
     * steer-brake that slows us when angular_z is large.
     */
    double compute_linear_velocity(double angle_error, double distance, double angular_z);

    void override_linear_with_wall_reverse(const Pose2D &our_pose, const FieldDescription &field,
                                           VelocityCommand &cmd);

    Pose2D clamp_to_field(const Pose2D &pose, const FieldDescription &field) const;

    static double distance_to_nearest_wall(const Pose2D &pose, const FieldDescription &field);

    static double wall_facing_angle(const Pose2D &pose, const FieldDescription &field);

    static double normalize_angle(double angle);

    static double distance_2d(const Pose2D &a, const Pose2D &b);

    double stop_distance_m_;
    double velocity_ramp_far_distance_;
    double velocity_ramp_near_distance_;
    double velocity_ramp_min_scale_;
    double brake_distance_;
    std::optional<double> wall_reverse_distance_m_;
    double wall_reverse_min_speed_;
    double wall_heading_threshold_rad_;
    std::optional<double> max_linear_command_;
    std::optional<double> max_angular_command_;
    double angular_kp_;
    double angular_kd_;
    double angle_threshold_;
    double lookahead_time_;
    double boundary_margin_;
    bool enable_hysteresis_;
    HazardAvoidance hazards_;
    std::shared_ptr<DiagnosticsModuleLogger> logger_;
    std::shared_ptr<ClockInterface> clock_;

    /** Latched turn direction (+1 or -1) to avoid dithering when target is behind us. 0 =
     * uncommitted. */
    int committed_turn_sign_ = 0;
    double prev_angle_error_ = 0.0;
    double prev_timestamp_ = 0.0;
    std::optional<Pose2D> last_known_our_pose_;
    Position2D our_footprint_offset_from_pose_;
    NavigationVisualization last_visualization_;
};

}  // namespace auto_battlebot
