#include "navigation/pursuit_navigation.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>
#include <cmath>

#include "diagnostics_logger/diagnostics_logger.hpp"
#include "enum_to_string_lower.hpp"
#include "enums/frame_id.hpp"
#include "transform_utils.hpp"

namespace auto_battlebot {

PursuitNavigation::PursuitNavigation(const PursuitNavigationConfiguration &config)
    : max_linear_velocity_(config.max_linear_velocity),
      max_angular_velocity_(config.max_angular_velocity),
      slowdown_distance_(config.slowdown_distance),
      stop_distance_(config.stop_distance),
      angular_kp_(config.angular_kp),
      angular_kd_(config.angular_kd),
      angle_threshold_(config.angle_threshold),
      lookahead_time_(config.lookahead_time),
      boundary_margin_(config.boundary_margin),
      enable_hysteresis_(config.enable_hysteresis),
      logger_(DiagnosticsLogger::get_logger("pursuit_nav")) {}

bool PursuitNavigation::initialize() {
    spdlog::info(
        "PursuitNavigation initialized with: max_linear_velocity={} m/s, "
        "max_angular_velocity={} rad/s, lookahead_time={} s",
        max_linear_velocity_, max_angular_velocity_, lookahead_time_);
    return true;
}

VelocityCommand PursuitNavigation::update(RobotDescriptionsStamped robots, FieldDescription field,
                                          const TargetSelection &target) {
    last_path_ = std::nullopt;

    auto our_robot_opt = find_our_robot(robots);
    if (!our_robot_opt.has_value()) {
        logger_->debug("no_robot", "Our robot not found");
        return VelocityCommand{0.0, 0.0, 0.0};
    }
    const auto &our_robot = our_robot_opt.value();

    Pose2D our_pose = pose_to_pose2d(our_robot.pose);

    last_path_ = NavigationPathSegment{our_pose.x, our_pose.y, target.pose.x, target.pose.y};

    logger_->debug("poses", {
                                {"our_frame_id", enum_to_string_lower(our_robot.frame_id)},
                                {"our_x", our_pose.x},
                                {"our_y", our_pose.y},
                                {"our_yaw_deg", our_pose.yaw * 180.0 / M_PI},
                                {"target_x", target.pose.x},
                                {"target_y", target.pose.y},
                            });

    auto cmd = compute_pursuit_command(our_pose, target.pose, field);

    logger_->debug("command", {
                                  {"linear_x", cmd.linear_x},
                                  {"angular_z", cmd.angular_z},
                              });

    return cmd;
}

std::optional<NavigationPathSegment> PursuitNavigation::get_last_path() const { return last_path_; }

std::optional<RobotDescription> PursuitNavigation::find_our_robot(
    const RobotDescriptionsStamped &robots) const {
    for (const auto &robot : robots.descriptions) {
        if (robot.frame_id == FrameId::OUR_ROBOT_1) return robot;
    }
    return std::nullopt;
}

VelocityCommand PursuitNavigation::compute_pursuit_command(const Pose2D &our_pose,
                                                           const Pose2D &target_pose,
                                                           const FieldDescription &field) {
    Pose2D clamped_target = clamp_to_field(target_pose, field);

    double dx = clamped_target.x - our_pose.x;
    double dy = clamped_target.y - our_pose.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    double angle_to_target = std::atan2(dy, dx);
    double angle_error = normalize_angle(angle_to_target - our_pose.yaw);

    if (enable_hysteresis_) {
        constexpr double commit_threshold = M_PI * 0.75;  // 135 deg
        constexpr double release_threshold = M_PI * 0.5;  // 90 deg

        if (std::abs(angle_error) > commit_threshold) {
            int sign = (angle_error > 0) ? 1 : -1;
            if (committed_turn_sign_ == 0 || committed_turn_sign_ != sign) {
                committed_turn_sign_ = sign;
            }
        } else if (std::abs(angle_error) < release_threshold) {
            committed_turn_sign_ = 0;
        }

        if (committed_turn_sign_ != 0 && std::abs(angle_error) > release_threshold) {
            angle_error = std::abs(angle_error) * committed_turn_sign_;
        }
    } else {
        committed_turn_sign_ = 0;
    }

    VelocityCommand cmd{0.0, 0.0, 0.0};

    logger_->debug("pursuit",
                   {
                       {"distance", distance},
                       {"angle_to_target_deg", angle_to_target * 180.0 / M_PI},
                       {"angle_error_deg", angle_error * 180.0 / M_PI},
                       {"threshold_deg", angle_threshold_ * 180.0 / M_PI},
                       {"facing_target", std::abs(angle_error) < angle_threshold_ ? 1 : 0},
                       {"turn_commit", committed_turn_sign_},
                   });

    if (distance < stop_distance_) {
        committed_turn_sign_ = 0;
        prev_angle_error_ = 0.0;
        logger_->debug("pursuit", "Stopped: at target");
        return cmd;
    }

    // PD angular control (normalized to [-1, 1])
    double now_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    double d_term = 0.0;
    if (angular_kd_ != 0.0 && prev_timestamp_ > 0.0) {
        double dt = now_s - prev_timestamp_;
        if (dt > 0.0 && dt < 0.5) {
            d_term = angular_kd_ * normalize_angle(angle_error - prev_angle_error_) / dt;
        }
    }
    prev_angle_error_ = angle_error;
    prev_timestamp_ = now_s;

    cmd.angular_z = (angular_kp_ * angle_error + d_term) / max_angular_velocity_;
    cmd.angular_z = std::clamp(cmd.angular_z, -1.0, 1.0);

    // Linear velocity (normalized to [0, 1]) - only drive forward if roughly facing target
    if (std::abs(angle_error) < angle_threshold_) {
        double speed_scale = 1.0;
        if (distance < slowdown_distance_) {
            double t = distance / slowdown_distance_;
            speed_scale = t * t;
        }

        double turn_scale = 1.0 - (std::abs(angle_error) / angle_threshold_) * 0.5;

        // Reduce speed proportionally to how hard we're turning
        double steer_brake = 1.0 - 0.5 * std::abs(cmd.angular_z);

        cmd.linear_x = speed_scale * turn_scale * steer_brake;
    } else {
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
