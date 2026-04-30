#pragma once

#include <memory>
#include <optional>

#include "diagnostics_logger/diagnostics_module_logger.hpp"
#include "navigation/config.hpp"

namespace auto_battlebot {
/**
 * @brief Navigation that pursues opponent robots
 *
 * Implements pure pursuit-style navigation that:
 * 1. Identifies our robot and opponent robots from descriptions
 * 2. Predicts opponent position using velocity
 * 3. Computes velocity commands to intercept the opponent
 * 4. Respects field boundaries
 */
class PursuitNavigation : public NavigationInterface {
   public:
    explicit PursuitNavigation(const PursuitNavigationConfiguration &config);

    bool initialize() override;

    VelocityCommand update(RobotDescriptionsStamped robots, FieldDescription field,
                           const TargetSelection &target) override;

    const NavigationVisualization &get_last_visualization() const override {
        return last_visualization_;
    }

   private:
    /**
     * @brief Find our robot from the robot descriptions
     */
    std::optional<RobotDescription> find_our_robot(const RobotDescriptionsStamped &robots) const;

    /**
     * @brief Compute velocity command to pursue the target
     */
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

    /**
     * @brief Compute angular velocity (rad/s) using a PD controller on angle_error.
     *
     * Updates prev_angle_error_ and prev_timestamp_ for use on the next call.
     */
    double compute_angular_velocity(double angle_error);

    /**
     * @brief Compute forward linear velocity (m/s).
     *
     * Returns 0 unless we are roughly facing the target. Otherwise scales speed by a velocity
     * ramp (full speed near the target, reduced at distance), an angle-error penalty, and a
     * steer-brake that slows us when angular_z is large.
     */
    double compute_linear_velocity(double angle_error, double distance, double angular_z);

    /**
     * @brief If close to and facing a wall, override cmd.linear_x with a reverse command to
     * back away. Modifies cmd in place.
     */
    void apply_wall_reverse(const Pose2D &our_pose, const FieldDescription &field,
                            VelocityCommand &cmd);

    /**
     * @brief Clamp position to stay within field boundaries
     */
    Pose2D clamp_to_field(const Pose2D &pose, const FieldDescription &field) const;

    /**
     * @brief Distance from pose to the nearest field wall (ignoring boundary_margin)
     */
    static double distance_to_nearest_wall(const Pose2D &pose, const FieldDescription &field);

    /**
     * @brief Angle (rad) pointing from pose toward the nearest field wall
     */
    static double wall_facing_angle(const Pose2D &pose, const FieldDescription &field);

    /**
     * @brief Normalize angle to [-pi, pi]
     */
    static double normalize_angle(double angle);

    /**
     * @brief Compute distance between two 2D positions
     */
    static double distance_2d(const Pose2D &a, const Pose2D &b);

    double stop_distance_;
    double velocity_ramp_far_distance_;
    double velocity_ramp_near_distance_;
    double velocity_ramp_min_scale_;
    double wall_reverse_distance_;
    double wall_reverse_min_speed_;
    double wall_heading_threshold_;
    double max_linear_x_;
    double max_angular_z_;
    double angular_kp_;
    double angular_kd_;
    double angle_threshold_;
    double lookahead_time_;
    double boundary_margin_;
    bool enable_hysteresis_;
    std::shared_ptr<DiagnosticsModuleLogger> logger_;

    /** Latched turn direction (+1 or -1) to avoid dithering when target is behind us. 0 =
     * uncommitted. */
    int committed_turn_sign_ = 0;
    double prev_angle_error_ = 0.0;
    double prev_timestamp_ = 0.0;
    std::optional<Pose2D> last_known_our_pose_;
    NavigationVisualization last_visualization_;
};

}  // namespace auto_battlebot
