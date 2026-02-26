#pragma once

#include <optional>
#include <limits>
#include "navigation/config.hpp"

namespace auto_battlebot
{
    /**
     * @brief Navigation that pursues opponent robots
     *
     * Implements pure pursuit-style navigation that:
     * 1. Identifies our robot and opponent robots from descriptions
     * 2. Predicts opponent position using velocity
     * 3. Computes velocity commands to intercept the opponent
     * 4. Respects field boundaries
     */
    class PursuitNavigation : public NavigationInterface
    {
    public:
        explicit PursuitNavigation(const PursuitNavigationConfiguration &config);

        bool initialize() override;

        VelocityCommand update(RobotDescriptionsStamped robots, FieldDescription field) override;
        std::optional<NavigationPathSegment> get_last_path() const override;

    private:
        /**
         * @brief Find our robot from the robot descriptions
         */
        std::optional<RobotDescription> find_our_robot(const RobotDescriptionsStamped &robots) const;

        /**
         * @brief Find the closest opponent robot to pursue
         */
        std::optional<RobotDescription> find_target_robot(
            const RobotDescriptionsStamped &robots,
            const RobotDescription &our_robot) const;

        /**
         * @brief Predict where the target will be after lookahead_time
         */
        Pose2D predict_target_position(const RobotDescription &target) const;

        /**
         * @brief Compute velocity command to pursue the target
         */
        VelocityCommand compute_pursuit_command(
            const Pose2D &our_pose,
            const Pose2D &target_pose,
            const FieldDescription &field) const;

        /**
         * @brief Clamp position to stay within field boundaries
         */
        Pose2D clamp_to_field(const Pose2D &pose, const FieldDescription &field) const;

        /**
         * @brief Normalize angle to [-pi, pi]
         */
        static double normalize_angle(double angle);

        /**
         * @brief Compute distance between two 2D positions
         */
        static double distance_2d(const Pose2D &a, const Pose2D &b);

        // Configuration parameters
        double max_linear_velocity_;
        double max_angular_velocity_;
        double slowdown_distance_;
        double stop_distance_;
        double angular_kp_;
        double angle_threshold_;
        double lookahead_time_;
        double boundary_margin_;
        mutable std::optional<NavigationPathSegment> last_path_;
    };

} // namespace auto_battlebot
