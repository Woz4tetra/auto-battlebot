#include "navigation/motion_profile_navigation.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

#include "diagnostics_logger/diagnostics_logger.hpp"
#include "enums/frame_id.hpp"
#include "transform_utils.hpp"

namespace {
enum class PoseSource { Live, Cached };
}

namespace auto_battlebot {

MotionProfileNavigation::MotionProfileNavigation(const MotionProfileNavigationConfiguration &config,
                                                 std::shared_ptr<ClockInterface> clock)
    : max_linear_speed_fwd_(config.max_linear_speed_fwd),
      max_linear_speed_rev_(config.max_linear_speed_rev),
      tau_accel_(config.tau_accel),
      tau_decel_(config.tau_decel),
      latency_(config.latency),
      deadzone_(config.deadzone),
      accel_limit_(config.accel_limit),
      terminal_velocity_(config.terminal_velocity),
      stop_distance_(config.stop_distance),
      speed_kp_(config.speed_kp),
      speed_ki_(config.speed_ki),
      angular_kp_(config.angular_kp),
      angular_kd_(config.angular_kd),
      angle_threshold_(config.angle_threshold),
      max_angular_command_(config.max_angular_command),
      enable_hysteresis_(config.enable_hysteresis),
      boundary_margin_(config.boundary_margin),
      max_linear_command_(config.max_linear_command),
      wall_reverse_distance_(config.wall_reverse_distance),
      wall_reverse_min_speed_(config.wall_reverse_min_speed),
      wall_heading_threshold_(config.wall_heading_threshold),
      logger_(DiagnosticsLogger::get_logger("motion_profile_nav")),
      clock_(std::move(clock)) {}

void MotionProfileNavigation::reset_state() {
    committed_turn_sign_ = 0;
    prev_angle_error_ = 0.0;
    prev_angular_timestamp_ = 0.0;
    prev_v_ref_ = 0.0;
    speed_integral_ = 0.0;
    prev_our_pose_.reset();
    prev_speed_timestamp_ = 0.0;
    last_known_our_pose_.reset();
}

bool MotionProfileNavigation::initialize() {
    reset_state();
    last_visualization_ = NavigationVisualization{};
    spdlog::info(
        "MotionProfileNavigation initialized: v_max_fwd={} m/s, terminal_velocity={} m/s, "
        "brake_horizon={} s",
        max_linear_speed_fwd_, terminal_velocity_, tau_decel_ + latency_);
    return true;
}

VelocityCommand MotionProfileNavigation::update(RobotDescriptionsStamped robots,
                                                FieldDescription field,
                                                const TargetSelection &target) {
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

    auto cmd = compute_command(our_pose, target.pose, field);

    logger_->debug("command", {
                                  {"linear_x", cmd.linear_x},
                                  {"angular_z", cmd.angular_z},
                              });

    last_visualization_.command = cmd;
    last_visualization_.robots = std::move(robots);
    return cmd;
}

std::optional<RobotDescription> MotionProfileNavigation::find_our_robot(
    const RobotDescriptionsStamped &robots) const {
    for (const auto &robot : robots.descriptions) {
        if (robot.frame_id == FrameId::OUR_ROBOT_1) return robot;
    }
    return std::nullopt;
}

VelocityCommand MotionProfileNavigation::compute_command(const Pose2D &our_pose,
                                                         const Pose2D &target_pose,
                                                         const FieldDescription &field) {
    const double now_s = clock_->now();
    const Pose2D clamped_target = clamp_to_field(target_pose, field);
    const double dx = clamped_target.x - our_pose.x;
    const double dy = clamped_target.y - our_pose.y;
    const double distance = std::sqrt(dx * dx + dy * dy);
    const double angle_to_target = std::atan2(dy, dx);
    const double raw_angle_error = normalize_angle(angle_to_target - our_pose.yaw);
    const double angle_error = apply_hysteresis(raw_angle_error);

    // Logical dt between frames (guards against the accelerated free-run collapsing dt); compute
    // from the previous stamp before the speed estimator's state is refreshed below.
    double dt = 0.0;
    if (prev_speed_timestamp_ > 0.0) {
        const double d = now_s - prev_speed_timestamp_;
        if (d > 0.0 && d < 0.5) dt = d;
    }
    const double v_actual = estimate_forward_speed(our_pose, now_s);
    prev_our_pose_ = our_pose;
    prev_speed_timestamp_ = now_s;

    // Zero-velocity mission complete: cut the command and reset the tracker so the next mission
    // starts clean. Ram missions (terminal_velocity > 0) drive through the goal, never stopping.
    if (terminal_velocity_ <= 0.0 && distance < stop_distance_) {
        committed_turn_sign_ = 0;
        prev_angle_error_ = 0.0;
        prev_v_ref_ = 0.0;
        speed_integral_ = 0.0;
        logger_->debug("stop", {{"distance", distance}}, "Reached goal; stopping");
        return VelocityCommand{0.0, 0.0, 0.0};
    }

    const double v_ref = compute_reference_speed(distance, v_actual, dt);
    const double dvdt = (dt > 0.0) ? (v_ref - prev_v_ref_) / dt : 0.0;

    VelocityCommand cmd{0.0, 0.0, 0.0};
    cmd.angular_z = compute_angular_command(angle_error, now_s);

    // Only drive forward once roughly facing the target; otherwise turn in place and bleed the
    // linear state so the robot does not lunge the instant the heading gate opens.
    if (std::abs(angle_error) < angle_threshold_) {
        cmd.linear_x = compute_linear_command(v_ref, dvdt, v_actual, dt);
        prev_v_ref_ = v_ref;
    } else {
        prev_v_ref_ = 0.0;
        speed_integral_ = 0.0;
    }

    apply_wall_reverse(our_pose, field, cmd);

    if (max_linear_command_ > 0.0) {
        cmd.linear_x = std::clamp(cmd.linear_x, -max_linear_command_, max_linear_command_);
    }
    if (max_angular_command_ > 0.0) {
        cmd.angular_z = std::clamp(cmd.angular_z, -max_angular_command_, max_angular_command_);
    }

    logger_->debug("profile",
                   {
                       {"distance", distance},
                       {"v_ref", v_ref},
                       {"v_actual", v_actual},
                       {"angle_error_deg", angle_error * 180.0 / M_PI},
                       {"facing_target", std::abs(angle_error) < angle_threshold_ ? 1 : 0},
                   });
    return cmd;
}

double MotionProfileNavigation::estimate_forward_speed(const Pose2D &our_pose, double now_s) const {
    if (!prev_our_pose_.has_value() || prev_speed_timestamp_ <= 0.0) return 0.0;
    const double dt = now_s - prev_speed_timestamp_;
    if (dt <= 0.0 || dt >= 0.5) return 0.0;
    const double dx = our_pose.x - prev_our_pose_->x;
    const double dy = our_pose.y - prev_our_pose_->y;
    // Project the displacement onto the current heading to get a signed forward speed.
    const double forward = dx * std::cos(our_pose.yaw) + dy * std::sin(our_pose.yaw);
    return forward / dt;
}

double MotionProfileNavigation::compute_reference_speed(double distance, double v_actual,
                                                        double dt) const {
    // Coast-aware brake schedule for a first-order plant. Two measured effects set where braking
    // must begin:
    //   - Latency lead: the brake command bites `latency` seconds late, after the robot has already
    //     travelled v*latency, so the schedule is evaluated at that future distance.
    //   - Coast horizon: after the command bites, the residual travel to shed to v_term is ~
    //     v*tau_decel, so keeping v <= v_term + d_eff/tau_decel guarantees arrival at v_term.
    // Together: v_ref = v_term + max(0, d - v*latency) / tau_decel. This is the inverse of the
    // measured plant, not a hand-set brake distance.
    const double d_eff = std::max(0.0, distance - std::abs(v_actual) * latency_);
    const double brake_horizon = std::max(tau_decel_ + latency_, 1e-3);
    double v_ref = terminal_velocity_ + d_eff / brake_horizon;
    v_ref = std::clamp(v_ref, 0.0, max_linear_speed_fwd_);

    // Rate-limit ramp-up only, for a clean launch and a bounded feedforward derivative. Braking
    // (a falling reference) must stay immediate.
    if (dt > 0.0 && accel_limit_ > 0.0) {
        const double max_rise = prev_v_ref_ + accel_limit_ * dt;
        v_ref = std::min(v_ref, max_rise);
    }
    return v_ref;
}

double MotionProfileNavigation::compute_linear_command(double v_ref, double dvdt, double v_actual,
                                                       double dt) {
    const double v_max = (v_ref >= 0.0) ? max_linear_speed_fwd_ : max_linear_speed_rev_;
    if (v_max <= 1e-6) return 0.0;

    // Inverse-plant feedforward: steady-state fraction plus the lag term tau * dv/dt, using BOTH
    // signs of dv/dt and the regime-appropriate time constant. A rising reference (dv/dt > 0) adds
    // thrust to overcome spin-up lag; a falling reference (dv/dt < 0) reduces throttle or commands
    // reverse, which is what a first-order plant needs to actually track a deceleration instead of
    // coasting past the goal.
    const double tau = (dvdt >= 0.0) ? tau_accel_ : tau_decel_;
    const double u_ff = (v_ref + tau * dvdt) / v_max;

    // Speed feedback closes the loop on plant-model error.
    const double err = v_ref - v_actual;
    if (speed_ki_ > 0.0) {
        speed_integral_ += err * dt;
        // Anti-windup: bound the integral so its command contribution stays within [-1, 1].
        const double limit = 1.0 / speed_ki_;
        speed_integral_ = std::clamp(speed_integral_, -limit, limit);
    }
    const double u_fb = speed_kp_ * err + speed_ki_ * speed_integral_;

    double u = u_ff + u_fb;

    // Exact inverse-deadzone: the plant applies (|c| - dz)/(1 - dz), so pre-distort the command so
    // the desired fraction survives. No-op when deadzone_ == 0 (the sim / real-robot default, since
    // the transmitter lifts the physical deadzone upstream).
    if (deadzone_ > 0.0 && std::abs(u) > 1e-6) {
        u = std::copysign(std::abs(u) * (1.0 - deadzone_) + deadzone_, u);
    }
    return std::clamp(u, -1.0, 1.0);
}

double MotionProfileNavigation::apply_hysteresis(double angle_error) {
    if (!enable_hysteresis_) {
        committed_turn_sign_ = 0;
        return angle_error;
    }

    constexpr double commit_threshold = M_PI * 0.75;  // 135 deg
    constexpr double release_threshold = M_PI * 0.5;  // 90 deg

    // Commit to a turn direction on a large error, then HOLD it until the error falls below the
    // release threshold. Near 180 deg the shortest-turn sign flips on tiny perturbations (and the
    // actuation latency keeps it flipping), so without the latch the robot dithers left-right and
    // never completes the turn. Only commit when not already committed.
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

double MotionProfileNavigation::compute_angular_command(double angle_error, double now_s) {
    // Logical time (frame stamp), not wall-clock: the PD derivative dt must track the time between
    // frames the controller actually processed so it stays correct under sim / playback time
    // scaling.
    double d_term = 0.0;
    if (angular_kd_ != 0.0 && prev_angular_timestamp_ > 0.0) {
        const double dt = now_s - prev_angular_timestamp_;
        if (dt > 0.0 && dt < 0.5) {
            d_term = angular_kd_ * normalize_angle(angle_error - prev_angle_error_) / dt;
        }
    }
    prev_angle_error_ = angle_error;
    prev_angular_timestamp_ = now_s;

    return angular_kp_ * angle_error + d_term;
}

void MotionProfileNavigation::apply_wall_reverse(const Pose2D &our_pose,
                                                 const FieldDescription &field,
                                                 VelocityCommand &cmd) const {
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

double MotionProfileNavigation::distance_to_nearest_wall(const Pose2D &pose,
                                                         const FieldDescription &field) {
    double half_x = field.size.size.x / 2.0;
    double half_y = field.size.size.y / 2.0;
    double dist_x = half_x - std::abs(pose.x);
    double dist_y = half_y - std::abs(pose.y);
    return std::min(dist_x, dist_y);
}

double MotionProfileNavigation::wall_facing_angle(const Pose2D &pose,
                                                  const FieldDescription &field) {
    double half_x = field.size.size.x / 2.0;
    double half_y = field.size.size.y / 2.0;
    double dist_x = half_x - std::abs(pose.x);
    double dist_y = half_y - std::abs(pose.y);
    if (dist_x <= dist_y) {
        // Nearest wall is left or right; normal points in the ±X direction.
        return (pose.x >= 0.0) ? 0.0 : M_PI;
    }
    // Nearest wall is top or bottom; normal points in the ±Y direction.
    return (pose.y >= 0.0) ? M_PI / 2.0 : -M_PI / 2.0;
}

Pose2D MotionProfileNavigation::clamp_to_field(const Pose2D &pose,
                                               const FieldDescription &field) const {
    Pose2D clamped = pose;
    double half_x = (field.size.size.x / 2.0) - boundary_margin_;
    double half_y = (field.size.size.y / 2.0) - boundary_margin_;
    if (half_x > 0) {
        clamped.x = std::clamp(clamped.x, -half_x, half_x);
    }
    if (half_y > 0) {
        clamped.y = std::clamp(clamped.y, -half_y, half_y);
    }
    return clamped;
}

double MotionProfileNavigation::normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

}  // namespace auto_battlebot
