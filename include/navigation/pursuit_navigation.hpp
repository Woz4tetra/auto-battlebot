#pragma once

#include <memory>
#include <optional>

#include "diagnostics_logger/diagnostics_module_logger.hpp"
#include "navigation/config.hpp"
#include "navigation/navigation_robots_cache.hpp"

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
    std::optional<NavigationPathSegment> get_last_path() const override;

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
    mutable std::optional<NavigationPathSegment> last_path_;
    std::shared_ptr<DiagnosticsModuleLogger> logger_;

    /** Latched turn direction (+1 or -1) to avoid dithering when target is behind us. 0 =
     * uncommitted. */
    int committed_turn_sign_ = 0;
    double prev_angle_error_ = 0.0;
    double prev_timestamp_ = 0.0;
    std::optional<Pose2D> last_known_our_pose_;
    NavigationRobotsCache nav_robots_cache_;
};

}  // namespace auto_battlebot
