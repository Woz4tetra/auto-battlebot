#include "navigation/pursuit_navigation.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

#include "transform_utils.hpp"

namespace auto_battlebot {

PursuitNavigation::PursuitNavigation(const PursuitNavigationConfiguration &config)
    : max_linear_velocity_(config.max_linear_velocity),
      max_angular_velocity_(config.max_angular_velocity),
      slowdown_distance_(config.slowdown_distance),
      stop_distance_(config.stop_distance),
      angular_kp_(config.angular_kp),
      angle_threshold_(config.angle_threshold),
      lookahead_time_(config.lookahead_time),
      boundary_margin_(config.boundary_margin) {}

bool PursuitNavigation::initialize() {
    std::cout << "PursuitNavigation initialized with:" << std::endl;
    std::cout << "  max_linear_velocity: " << max_linear_velocity_ << " m/s" << std::endl;
    std::cout << "  max_angular_velocity: " << max_angular_velocity_ << " rad/s" << std::endl;
    std::cout << "  lookahead_time: " << lookahead_time_ << " s" << std::endl;
    return true;
}

VelocityCommand PursuitNavigation::update(RobotDescriptionsStamped robots, FieldDescription field) {
    last_path_ = std::nullopt;
    // Find our robot
    auto our_robot_opt = find_our_robot(robots);
    if (!our_robot_opt.has_value()) {
        return VelocityCommand{0.0, 0.0, 0.0};
    }
    const auto &our_robot = our_robot_opt.value();

    auto target_opt = find_target_robot(robots, our_robot);
    if (!target_opt.has_value()) {
        return VelocityCommand{0.0, 0.0, 0.0};
    }
    const auto &target = target_opt.value();

    // Convert poses to 2D
    Pose2D our_pose = pose_to_pose2d(our_robot.pose);
    Pose2D target_pose = predict_target_position(target);

    last_path_ = NavigationPathSegment{our_pose.x, our_pose.y, target_pose.x, target_pose.y};

    // Compute and return pursuit command
    return compute_pursuit_command(our_pose, target_pose, field);
}

std::optional<NavigationPathSegment> PursuitNavigation::get_last_path() const { return last_path_; }

std::optional<RobotDescription> PursuitNavigation::find_our_robot(
    const RobotDescriptionsStamped &robots) const {
    // Look for robots with "MR_STABS" or "MRS_BUFF" labels (our robots)
    // These are typically the controlled robots
    for (const auto &robot : robots.descriptions) {
        switch (robot.label) {
            case Label::MR_STABS_MK1:
            case Label::MR_STABS_MK2:
            case Label::MRS_BUFF_MK1:
            case Label::MRS_BUFF_MK2:
                return robot;
            default:
                break;
        }
    }
    return std::nullopt;
}

std::optional<RobotDescription> PursuitNavigation::find_target_robot(
    const RobotDescriptionsStamped &robots, const RobotDescription &our_robot) const {
    std::optional<RobotDescription> closest_target;
    double min_distance = std::numeric_limits<double>::max();

    Pose2D our_pose = pose_to_pose2d(our_robot.pose);

    for (const auto &robot : robots.descriptions) {
        // Skip our own robots
        if (robot.label == our_robot.label) {
            continue;
        }

        // Target opponents and house bots
        if (robot.label == Label::OPPONENT || robot.label == Label::HOUSE_BOT) {
            Pose2D robot_pose = pose_to_pose2d(robot.pose);
            double dist = distance_2d(our_pose, robot_pose);

            if (dist < min_distance) {
                min_distance = dist;
                closest_target = robot;
            }
        }
    }

    return closest_target;
}

Pose2D PursuitNavigation::predict_target_position(const RobotDescription &target) const {
    Pose2D current_pose = pose_to_pose2d(target.pose);

    // Predict future position based on velocity
    current_pose.x += target.velocity.vx * lookahead_time_;
    current_pose.y += target.velocity.vy * lookahead_time_;
    current_pose.yaw += target.velocity.omega * lookahead_time_;
    current_pose.yaw = normalize_angle(current_pose.yaw);

    return current_pose;
}

VelocityCommand PursuitNavigation::compute_pursuit_command(const Pose2D &our_pose,
                                                           const Pose2D &target_pose,
                                                           const FieldDescription &field) const {
    // Clamp target to field boundaries
    Pose2D clamped_target = clamp_to_field(target_pose, field);

    // Compute vector to target
    double dx = clamped_target.x - our_pose.x;
    double dy = clamped_target.y - our_pose.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    // Compute angle to target
    double angle_to_target = std::atan2(dy, dx);
    double angle_error = normalize_angle(angle_to_target - our_pose.yaw);

    VelocityCommand cmd{0.0, 0.0, 0.0};

    // If we're at the target, stop
    if (distance < stop_distance_) {
        return cmd;
    }

    // Angular velocity - always try to face the target
    cmd.angular_z = angular_kp_ * angle_error;
    cmd.angular_z = std::clamp(cmd.angular_z, -max_angular_velocity_, max_angular_velocity_);

    // Linear velocity - only drive forward if roughly facing target
    if (std::abs(angle_error) < angle_threshold_) {
        // Scale velocity based on distance (slow down near target)
        double speed_scale = 1.0;
        if (distance < slowdown_distance_) {
            speed_scale = distance / slowdown_distance_;
        }

        // Also reduce speed when turning sharply
        double turn_scale = 1.0 - (std::abs(angle_error) / angle_threshold_) * 0.5;

        cmd.linear_x = max_linear_velocity_ * speed_scale * turn_scale;
    } else {
        // Turn in place if not facing target
        cmd.linear_x = 0.0;
    }

    return cmd;
}

Pose2D PursuitNavigation::clamp_to_field(const Pose2D &pose, const FieldDescription &field) const {
    Pose2D clamped = pose;

    // Get field half-sizes with margin
    double half_x = (field.size.size.x / 2.0) - boundary_margin_;
    double half_y = (field.size.size.y / 2.0) - boundary_margin_;

    // Clamp to field boundaries (field center is at origin)
    if (half_x > 0) {
        clamped.x = std::clamp(clamped.x, -half_x, half_x);
    }
    if (half_y > 0) {
        clamped.y = std::clamp(clamped.y, -half_y, half_y);
    }

    return clamped;
}

double PursuitNavigation::normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

double PursuitNavigation::distance_2d(const Pose2D &a, const Pose2D &b) {
    double dx = b.x - a.x;
    double dy = b.y - a.y;
    return std::sqrt(dx * dx + dy * dy);
}

}  // namespace auto_battlebot
