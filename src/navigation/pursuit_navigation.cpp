#include "navigation/pursuit_navigation.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>
#include <cmath>

#include "diagnostics_logger/diagnostics_logger.hpp"
#include "enums/frame_id.hpp"
#include "transform_utils.hpp"

namespace {
enum class PoseSource { Live, Cached };
}

namespace auto_battlebot {

PursuitNavigation::PursuitNavigation(const PursuitNavigationConfiguration &config,
                                     std::shared_ptr<ClockInterface> clock)
    : stop_distance_(config.stop_distance),
      velocity_ramp_far_distance_(config.velocity_ramp_far_distance),
      velocity_ramp_near_distance_(config.velocity_ramp_near_distance),
      velocity_ramp_min_scale_(config.velocity_ramp_min_scale),
      brake_distance_(config.brake_distance),
      wall_reverse_distance_(config.wall_reverse_distance),
      wall_reverse_min_speed_(config.wall_reverse_min_speed),
      wall_heading_threshold_(config.wall_heading_threshold),
      max_linear_command_(config.max_linear_command),
      max_angular_command_(config.max_angular_command),
      angular_kp_(config.angular_kp),
      angular_kd_(config.angular_kd),
      angle_threshold_(config.angle_threshold),
      lookahead_time_(config.lookahead_time),
      boundary_margin_(config.boundary_margin),
      enable_hysteresis_(config.enable_hysteresis),
      logger_(DiagnosticsLogger::get_logger("pursuit_nav")),
      clock_(std::move(clock)) {}

bool PursuitNavigation::initialize() {
    committed_turn_sign_ = 0;
    prev_angle_error_ = 0.0;
    prev_timestamp_ = 0.0;
    last_known_our_pose_.reset();
    last_visualization_ = NavigationVisualization{};
    spdlog::info("PursuitNavigation initialized with lookahead_time={} s", lookahead_time_);
    return true;
}

VelocityCommand PursuitNavigation::update(RobotDescriptionsStamped robots, FieldDescription field,
                                          const TargetSelection &target) {
    // Continuity across momentary detection gaps is provided upstream by Runner's
    // RobotDescriptionsCache so that target selection and navigation see the same robot set.
    last_visualization_ = NavigationVisualization{};
    last_visualization_.header = robots.header;

    auto our_robot_opt = find_our_robot(robots);
    Pose2D our_pose;
    PoseSource pose_source = PoseSource::Live;
    if (our_robot_opt.has_value()) {
        our_pose = pose_to_pose2d(our_robot_opt->pose);
        last_known_our_pose_ = our_pose;
    } else if (last_known_our_pose_.has_value()) {
        our_pose = *last_known_our_pose_;
        pose_source = PoseSource::Cached;
        logger_->debug("no_robot", "Our robot not found; using cached pose");
    } else {
        logger_->debug("no_robot", "Our robot not found and no cached pose");
        last_visualization_.robots = std::move(robots);
        return VelocityCommand{0.0, 0.0, 0.0};
    }

    last_visualization_.path =
        NavigationPathSegment{our_pose.x, our_pose.y, target.pose.x, target.pose.y};

    logger_->debug("poses",
                   {
                       {"our_pose_source", pose_source == PoseSource::Cached ? "cached" : "live"},
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

    last_visualization_.command = cmd;
    // Last write to last_visualization_; this also retires effective_robots, which we no longer
    // need.
    last_visualization_.robots = std::move(robots);
    return cmd;
}

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
    const Pose2D clamped_target = clamp_to_field(target_pose, field);
    const double dx = clamped_target.x - our_pose.x;
    const double dy = clamped_target.y - our_pose.y;
    const double distance = std::sqrt(dx * dx + dy * dy);
    const double angle_to_target = std::atan2(dy, dx);
    const double raw_angle_error = normalize_angle(angle_to_target - our_pose.yaw);
    const double angle_error = apply_hysteresis(raw_angle_error);

    if (distance < stop_distance_) {
        committed_turn_sign_ = 0;
        prev_angle_error_ = 0.0;
        logger_->debug("pursuit",
                       {{"distance", distance}, {"angle_error_deg", angle_error * 180.0 / M_PI}},
                       "Stopped: at target");
        return VelocityCommand{0.0, 0.0, 0.0};
    }

    VelocityCommand cmd{0.0, 0.0, 0.0};
    cmd.angular_z = compute_angular_velocity(angle_error);
    cmd.linear_x = compute_linear_velocity(angle_error, distance, cmd.angular_z);
    apply_wall_reverse(our_pose, field, cmd);

    if (max_linear_command_ > 0.0) {
        cmd.linear_x = std::clamp(cmd.linear_x, -max_linear_command_, max_linear_command_);
    }
    if (max_angular_command_ > 0.0) {
        cmd.angular_z = std::clamp(cmd.angular_z, -max_angular_command_, max_angular_command_);
    }

    logger_->debug("pursuit",
                   {
                       {"distance", distance},
                       {"angle_to_target_deg", angle_to_target * 180.0 / M_PI},
                       {"angle_error_deg", angle_error * 180.0 / M_PI},
                       {"threshold_deg", angle_threshold_ * 180.0 / M_PI},
                       {"facing_target", std::abs(angle_error) < angle_threshold_ ? 1 : 0},
                       {"turn_commit", committed_turn_sign_},
                   });

    return cmd;
}

double PursuitNavigation::apply_hysteresis(double angle_error) {
    if (!enable_hysteresis_) {
        committed_turn_sign_ = 0;
        return angle_error;
    }

    constexpr double commit_threshold = M_PI * 0.75;  // 135 deg
    constexpr double release_threshold = M_PI * 0.5;  // 90 deg

    // Commit to a turn direction on a large error, then HOLD it until the error falls below the
    // release threshold. The previous code re-committed whenever the error's sign changed, which
    // chattered near 180 deg: there the shortest-turn sign flips on tiny perturbations (and the
    // actuation latency keeps it flipping), so the robot jittered left-right and never completed
    // the turn instead of committing to one direction and rotating through. Only commit when not
    // already committed.
    if (committed_turn_sign_ == 0) {
        if (std::abs(angle_error) > commit_threshold) {
            committed_turn_sign_ = (angle_error > 0) ? 1 : -1;
        }
    } else if (std::abs(angle_error) < release_threshold) {
        committed_turn_sign_ = 0;
    }

    if (committed_turn_sign_ != 0 && std::abs(angle_error) > release_threshold) {
        return std::abs(angle_error) * committed_turn_sign_;
    }
    return angle_error;
}

double PursuitNavigation::compute_angular_velocity(double angle_error) {
    // Logical time (frame stamp), not wall-clock: the PD derivative dt must track the time between
    // the frames the controller actually processed, so it stays correct under sim/playback time
    // scaling.
    const double now_s = clock_->now();
    double d_term = 0.0;
    if (angular_kd_ != 0.0 && prev_timestamp_ > 0.0) {
        const double dt = now_s - prev_timestamp_;
        if (dt > 0.0 && dt < 0.5) {
            d_term = angular_kd_ * normalize_angle(angle_error - prev_angle_error_) / dt;
        }
    }
    prev_angle_error_ = angle_error;
    prev_timestamp_ = now_s;

    return angular_kp_ * angle_error + d_term;
}

double PursuitNavigation::compute_linear_velocity(double angle_error, double distance,
                                                  double angular_z) {
    if (std::abs(angle_error) >= angle_threshold_) return 0.0;

    // Velocity ramp: full speed below near_distance, min_scale at far_distance and beyond,
    // linear interpolation in between.
    double speed_scale = 1.0;
    if (distance > velocity_ramp_near_distance_) {
        if (distance >= velocity_ramp_far_distance_) {
            speed_scale = velocity_ramp_min_scale_;
        } else {
            const double t = (distance - velocity_ramp_near_distance_) /
                             (velocity_ramp_far_distance_ - velocity_ramp_near_distance_);
            speed_scale = 1.0 - t * (1.0 - velocity_ramp_min_scale_);
        }
    }

    // Coast-aware brake: within brake_distance of the target, ramp the commanded speed linearly to
    // zero so the robot decelerates into the goal instead of driving in at full speed and coasting
    // past it. brake_distance should cover the stopping lead (latency travel + coast). 0 disables
    // (ram behavior).
    if (brake_distance_ > 0.0 && distance < brake_distance_) {
        speed_scale *= distance / brake_distance_;
    }

    const double turn_scale = 1.0 - (std::abs(angle_error) / angle_threshold_) * 0.5;
    // Reduce speed when commanding a hard turn. Brake on the angular command as a fraction of the
    // configured limit so the term is dimensionless and bounded; with no limit set, fall back to
    // the legacy form but clamp it so a large angular command can never flip linear into reverse
    // thrust.
    double steer_brake;
    if (max_angular_command_ > 0.0) {
        steer_brake = 1.0 - 0.5 * std::min(1.0, std::abs(angular_z) / max_angular_command_);
    } else {
        steer_brake = std::max(0.0, 1.0 - 0.5 * std::abs(angular_z));
    }

    return speed_scale * turn_scale * steer_brake;
}

void PursuitNavigation::apply_wall_reverse(const Pose2D &our_pose, const FieldDescription &field,
                                           VelocityCommand &cmd) {
    if (wall_reverse_distance_ <= 0.0) return;

    const double wall_dist = distance_to_nearest_wall(our_pose, field);
    if (wall_dist >= wall_reverse_distance_) return;

    const double angle_to_wall = wall_facing_angle(our_pose, field);
    const double heading_err = std::abs(normalize_angle(angle_to_wall - our_pose.yaw));
    if (heading_err >= wall_heading_threshold_) return;

    cmd.linear_x = -std::max(std::abs(cmd.linear_x), wall_reverse_min_speed_);
    logger_->debug("wall_reverse", {{"wall_dist", wall_dist},
                                    {"angle_to_wall_deg", angle_to_wall * 180.0 / M_PI},
                                    {"heading_err_deg", heading_err * 180.0 / M_PI}});
}

double PursuitNavigation::distance_to_nearest_wall(const Pose2D &pose,
                                                   const FieldDescription &field) {
    double half_x = field.size.size.x / 2.0;
    double half_y = field.size.size.y / 2.0;
    double dist_x = half_x - std::abs(pose.x);
    double dist_y = half_y - std::abs(pose.y);
    return std::min(dist_x, dist_y);
}

double PursuitNavigation::wall_facing_angle(const Pose2D &pose, const FieldDescription &field) {
    double half_x = field.size.size.x / 2.0;
    double half_y = field.size.size.y / 2.0;
    double dist_x = half_x - std::abs(pose.x);
    double dist_y = half_y - std::abs(pose.y);
    if (dist_x <= dist_y) {
        // Nearest wall is left or right; normal points in ±X direction
        return (pose.x >= 0.0) ? 0.0 : M_PI;
    } else {
        // Nearest wall is top or bottom; normal points in ±Y direction
        return (pose.y >= 0.0) ? M_PI / 2.0 : -M_PI / 2.0;
    }
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
