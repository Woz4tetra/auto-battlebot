#include "navigation/motion_profile_navigation.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <magic_enum.hpp>

#include "diagnostics_logger/diagnostics_logger.hpp"
#include "enums/frame_id.hpp"
#include "hazards/hazard_geometry.hpp"
#include "transform_utils.hpp"

namespace {
enum class PoseSource { Live, Cached };

auto_battlebot::HazardAvoidanceSettings hazard_settings_from(
    const auto_battlebot::MotionProfileNavigationConfiguration &config) {
    auto_battlebot::HazardAvoidanceSettings settings;
    settings.tangent_enable = config.hazard_tangent_enable;
    settings.tangent_max_iterations = config.hazard_tangent_max_iterations;
    settings.waypoint_clearance_m = config.hazard_waypoint_clearance_m;
    settings.side_release_m = config.hazard_side_release_m;
    settings.speed_cap_enable = config.hazard_speed_cap_enable;
    settings.speed_cap_floor = config.hazard_speed_cap_floor;
    settings.prediction_horizon_s = config.hazard_prediction_horizon_s;
    settings.reverse_distance = config.hazard_reverse_distance;
    settings.heading_threshold = config.hazard_heading_threshold;
    settings.reverse_min_speed = config.hazard_reverse_min_speed;
    settings.max_yaw_rate = config.max_yaw_rate;
    return settings;
}
}  // namespace

namespace auto_battlebot {

MotionProfileNavigation::MotionProfileNavigation(const MotionProfileNavigationConfiguration &config,
                                                 std::shared_ptr<ClockInterface> clock)
    : max_linear_speed_fwd_(config.plant.k_fwd),
      max_linear_speed_rev_(config.plant.k_rev),
      tau_accel_(config.plant.tau_lin_a),
      tau_decel_(config.plant.tau_lin_d),
      latency_(config.plant.delay_s),
      deadzone_(config.deadzone),
      accel_limit_(config.accel_limit),
      steer_brake_coeff_(config.plant.c_sb),
      steer_brake_floor_(config.steer_brake_floor),
      angular_deadzone_left_(config.plant.dz_ang_l),
      angular_deadzone_right_(config.plant.dz_ang_r),
      attack_terminal_speed_fraction_(config.attack_terminal_speed_fraction),
      run_away_terminal_speed_fraction_(config.run_away_terminal_speed_fraction),
      stop_distance_(config.stop_distance),
      hazards_(hazard_settings_from(config)),
      speed_kp_(config.speed_kp),
      speed_ki_(config.speed_ki),
      max_angular_speed_(config.plant.k_ang),
      tau_angular_accel_(config.plant.tau_ang_a),
      tau_angular_decel_(config.plant.tau_ang_d),
      // The coefficient is measured, the compensation is a choice: off means the droop term
      // simply never divides back out, which compensate_coupling already does at zero.
      angular_droop_coeff_(config.compensate_angular_droop ? config.plant.c_ad : 0.0),
      angular_droop_floor_(config.angular_droop_floor),
      angular_kp_(config.angular_kp),
      angular_kd_(config.angular_kd),
      angle_threshold_(config.angle_threshold),
      max_yaw_rate_(config.max_yaw_rate),
      max_angular_command_(config.max_angular_command),
      enable_hysteresis_(config.enable_hysteresis),
      boundary_margin_(config.boundary_margin),
      max_linear_command_(config.max_linear_command),
      wall_reverse_distance_(config.wall_reverse_distance),
      wall_reverse_min_speed_(config.wall_reverse_min_speed),
      wall_heading_threshold_(config.wall_heading_threshold),
      logger_(DiagnosticsLogger::get_logger("motion_profile_nav")),
      clock_(std::move(clock)) {}

void MotionProfileNavigation::reset_trajectory_state() {
    hazards_.reset();
    committed_turn_sign_ = 0;
    prev_angle_error_ = 0.0;
    prev_v_ref_ = 0.0;
    prev_w_ref_ = 0.0;
    speed_integral_ = 0.0;
    prev_linear_uncompensated_ = 0.0;
    prev_angular_uncompensated_ = 0.0;
}

void MotionProfileNavigation::reset_state() {
    reset_trajectory_state();
    prev_angular_timestamp_ = 0.0;
    prev_speed_timestamp_ = 0.0;
    last_known_our_pose_.reset();
    last_measured_speed_ = 0.0;
    prev_mode_ = BehaviorMode::ATTACK;
}

double MotionProfileNavigation::terminal_velocity_for(BehaviorMode mode) const {
    const double configured = (mode == BehaviorMode::RUN_AWAY) ? run_away_terminal_speed_fraction_
                                                               : attack_terminal_speed_fraction_;
    // Scale the fraction by the fitted top speed. The controller only ever drives toward the
    // goal, so a negative fraction has nothing to mean and clamps to a stop.
    return std::clamp(configured, 0.0, 1.0) * max_linear_speed_fwd_;
}

bool MotionProfileNavigation::initialize() {
    reset_state();
    last_visualization_ = NavigationVisualization{};
    spdlog::info(
        "MotionProfileNavigation initialized: v_max_fwd={} m/s, w_max={} rad/s, "
        "terminal_velocity attack={} m/s run_away={} m/s, brake_horizon={} s, c_sb={}, c_ad={}",
        max_linear_speed_fwd_, max_angular_speed_, terminal_velocity_for(BehaviorMode::ATTACK),
        terminal_velocity_for(BehaviorMode::RUN_AWAY), tau_decel_ + latency_, steer_brake_coeff_,
        angular_droop_coeff_);
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
    std::optional<double> measured_speed;
    if (our_robot_opt.has_value()) {
        our_pose = pose_to_pose2d(our_robot_opt->pose);
        last_known_our_pose_ = our_pose;
        measured_speed = measure_forward_speed(*our_robot_opt, our_pose);
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
                       {"speed_measured", measured_speed.has_value() ? 1 : 0},
                   });

    auto cmd = compute_command(our_pose, target.pose, field, measured_speed, target.mode);

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
                                                         const FieldDescription &field,
                                                         std::optional<double> measured_speed,
                                                         BehaviorMode mode) {
    const double now_s = clock_->now();
    const double terminal_velocity = terminal_velocity_for(mode);

    // A flipped switch is a new mission: the goal jumps from the opponent to the safe point (or
    // back) and the terminal speed changes with it. Carrying the old reference across the flip
    // would feed the feedforward a dv/dt step that the plant never asked for.
    if (mode != prev_mode_) {
        reset_trajectory_state();
        logger_->debug("mode",
                       {{"mode", std::string(magic_enum::enum_name(mode))},
                        {"terminal_velocity", terminal_velocity}},
                       "Behavior mode changed; resetting the trajectory");
    }
    prev_mode_ = mode;
    // Option 1 (a legal goal) then option 4 (steer around what blocks the run to it). The
    // tangent point is itself clamped, so a hazard hugging a wall cannot push the waypoint out
    // of the arena.
    const Pose2D legal_target = clamp_to_field(target_pose, field);
    const Pose2D clamped_target =
        clamp_to_field(hazards_.steer_around(our_pose, legal_target, field), field);
    const double dx = clamped_target.x - our_pose.x;
    const double dy = clamped_target.y - our_pose.y;
    const double distance = std::sqrt(dx * dx + dy * dy);
    const double angle_to_target = std::atan2(dy, dx);
    const double raw_angle_error = normalize_angle(angle_to_target - our_pose.yaw);
    const double angle_error = apply_hysteresis(raw_angle_error);

    // Logical dt between frames (guards against the accelerated free-run collapsing dt).
    double dt = 0.0;
    if (prev_speed_timestamp_ > 0.0) {
        const double d = now_s - prev_speed_timestamp_;
        if (d > 0.0 && d < 0.5) dt = d;
    }
    prev_speed_timestamp_ = now_s;

    // Without a measurement, carry the last measured speed forward rather than reporting zero. A
    // zero here would tell the brake schedule there is no latency lead to subtract and tell the
    // speed feedback the robot is stopped, so the controller would command MORE thrust in exactly
    // the stretch where it is blind.
    const bool speed_is_measured = measured_speed.has_value();
    if (speed_is_measured) last_measured_speed_ = *measured_speed;
    const double v_actual = last_measured_speed_;

    // Zero-velocity mission complete: cut the command and reset the tracker so the next mission
    // starts clean. A drive-through mission (terminal velocity > 0) never stops.
    if (terminal_velocity <= 0.0 && distance < stop_distance_) {
        reset_trajectory_state();
        logger_->debug("stop", {{"distance", distance}}, "Reached goal; stopping");
        return VelocityCommand{0.0, 0.0, 0.0};
    }

    double v_ref = compute_reference_speed(distance, v_actual, dt, terminal_velocity);
    // Option 3: slow down only as much as the turn requires. Applied after the brake schedule so
    // it can only ever lower the reference, never raise it above what distance-to-go allows.
    const double v_ref_uncapped = v_ref;
    v_ref = std::min(v_ref, hazards_.cap_speed(our_pose, field, v_ref));
    const double dvdt = (dt > 0.0) ? (v_ref - prev_v_ref_) / dt : 0.0;

    VelocityCommand cmd{0.0, 0.0, 0.0};
    const double w_ref = compute_yaw_reference(angle_error, now_s);
    cmd.angular_z = compute_angular_command(w_ref, dt);
    if (max_angular_command_ > 0.0) {
        cmd.angular_z = std::clamp(cmd.angular_z, -max_angular_command_, max_angular_command_);
    }
    prev_w_ref_ = w_ref;

    // Only drive forward once roughly facing the target; otherwise turn in place and bleed the
    // linear state so the robot does not lunge the instant the heading gate opens.
    if (std::abs(angle_error) < angle_threshold_) {
        cmd.linear_x = compute_linear_command(v_ref, dvdt, v_actual, dt, speed_is_measured);
        prev_v_ref_ = v_ref;
    } else {
        prev_v_ref_ = 0.0;
        speed_integral_ = 0.0;
    }

    apply_wall_reverse(our_pose, field, cmd);
    const bool hazard_reverse = hazards_.apply_reverse(our_pose, field, cmd);

    if (max_linear_command_ > 0.0) {
        cmd.linear_x = std::clamp(cmd.linear_x, -max_linear_command_, max_linear_command_);
    }

    logger_->debug("profile",
                   {
                       {"distance", distance},
                       {"v_ref", v_ref},
                       {"v_actual", v_actual},
                       {"speed_is_measured", speed_is_measured ? 1 : 0},
                       {"w_ref", w_ref},
                       {"angle_error_deg", angle_error * 180.0 / M_PI},
                       {"facing_target", std::abs(angle_error) < angle_threshold_ ? 1 : 0},
                       {"terminal_velocity", terminal_velocity},
                       {"behavior_mode", std::string(magic_enum::enum_name(mode))},
                       {"hazard_count", static_cast<int>(field.hazards.size())},
                       {"hazard_waypoint", hazards_.last_substituted() ? 1 : 0},
                       {"hazard_side", hazards_.committed_side()},
                       {"hazard_speed_capped", v_ref < v_ref_uncapped - 1e-9 ? 1 : 0},
                       {"hazard_reverse", hazard_reverse ? 1 : 0},
                       {"target_x_steered", clamped_target.x},
                       {"target_y_steered", clamped_target.y},
                   });
    return cmd;
}

std::optional<double> MotionProfileNavigation::measure_forward_speed(const RobotDescription &robot,
                                                                     const Pose2D &our_pose) const {
    // A stale track is the filter propagating our own last command, so its velocity is a function
    // of what this controller asked for. Feeding that back is not feedback.
    if (robot.is_stale) return std::nullopt;
    // Project the field-frame velocity onto our heading for a signed forward speed.
    return robot.velocity.vx * std::cos(our_pose.yaw) + robot.velocity.vy * std::sin(our_pose.yaw);
}

double MotionProfileNavigation::compensate_coupling(double command, double effect, double coeff,
                                                    double floor) {
    if (coeff <= 0.0) return command;
    // The plant scales the channel by (1 - coeff*|effect|). Below the floor it delivers so little
    // that dividing by the modelled multiplier is extrapolation, not compensation: for the
    // steer-brake term the floor sits just under 1/c_sb, where the fitted linear loss reaches zero
    // and the fit stops describing the plant. Saturating there keeps the command finite and stops
    // the controller from believing it has authority it does not have.
    const double authority = std::max(floor, 1.0 - coeff * std::abs(effect));
    return command / authority;
}

double MotionProfileNavigation::compute_reference_speed(double distance, double v_actual, double dt,
                                                        double terminal_velocity) const {
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
    double v_ref = terminal_velocity + d_eff / brake_horizon;
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
                                                       double dt, bool speed_is_measured) {
    if (max_linear_speed_fwd_ <= 1e-6) return 0.0;

    // Inverse-plant feedforward: steady-state fraction plus the lag term tau * dv/dt, using BOTH
    // signs of dv/dt and the regime-appropriate time constant. A rising reference (dv/dt > 0) adds
    // thrust to overcome spin-up lag; a falling reference (dv/dt < 0) reduces throttle or commands
    // reverse, which is what a first-order plant needs to actually track a deceleration instead of
    // coasting past the goal.
    const double tau = (dvdt >= 0.0) ? tau_accel_ : tau_decel_;
    const double u_ff = (v_ref + tau * dvdt) / max_linear_speed_fwd_;

    // Speed feedback closes the loop on plant-model error, but only while the speed is a
    // measurement. On a stale track the integrator is frozen and the proportional term dropped;
    // the feedforward is open-loop by construction and stays valid.
    double u_fb = 0.0;
    if (speed_is_measured) {
        const double err = v_ref - v_actual;
        if (speed_ki_ > 0.0) {
            speed_integral_ += err * dt;
            // Anti-windup: bound the integral so its command contribution stays within [-1, 1].
            const double limit = 1.0 / speed_ki_;
            speed_integral_ = std::clamp(speed_integral_, -limit, limit);
        }
        u_fb = speed_kp_ * err + speed_ki_ * speed_integral_;
    }

    double u = u_ff + u_fb;

    // The feedforward divided by the forward gain, but reverse is a weaker drive than forward, so
    // a braking command normalized against k_fwd arrives smaller than intended. Rescale once the
    // sign is known. This is the branch the old sign-of-v_ref test could never reach: v_ref is
    // clamped non-negative, and it is the derivative term that drives the command negative.
    if (u < 0.0 && max_linear_speed_rev_ > 1e-6) {
        u *= max_linear_speed_fwd_ / max_linear_speed_rev_;
    }

    // Remember the command before compensation. Each channel's coupling term is driven by the
    // other channel's UNCOMPENSATED command, which breaks what is otherwise a positive feedback
    // loop: compensating the droop raises the turn command, which raises the steer-brake
    // compensation, which raises the droop compensation again. Measured loop gain is above 1, so
    // feeding the compensated values back saturates both channels within a few ticks.
    prev_linear_uncompensated_ = std::clamp(u, -1.0, 1.0);

    // Steer-brake: the plant multiplies forward authority by (1 - c_sb*|u_ang_eff|).
    const double u_ang_eff = plant_effective_command(
        prev_angular_uncompensated_, angular_deadzone_left_, angular_deadzone_right_);
    u = compensate_coupling(u, u_ang_eff, steer_brake_coeff_, steer_brake_floor_);

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

double MotionProfileNavigation::compute_yaw_reference(double angle_error, double now_s) {
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

    double w_ref = angular_kp_ * angle_error + d_term;
    if (max_yaw_rate_ > 0.0) w_ref = std::clamp(w_ref, -max_yaw_rate_, max_yaw_rate_);
    return w_ref;
}

double MotionProfileNavigation::compute_angular_command(double w_ref, double dt) {
    if (max_angular_speed_ <= 1e-6) return 0.0;

    // Same inverse-plant shape as the linear channel. The yaw spin-up constant is about twice the
    // coast constant, so the feedforward has to know which way the reference is moving: a turn
    // that is winding up needs extra command, a turn that is stopping needs the command pulled
    // back early or the heading sails past.
    const double dwdt = (dt > 0.0) ? (w_ref - prev_w_ref_) / dt : 0.0;
    const double tau =
        (std::abs(w_ref) > std::abs(prev_w_ref_)) ? tau_angular_accel_ : tau_angular_decel_;
    double u = (w_ref + tau * dwdt) / max_angular_speed_;
    prev_angular_uncompensated_ = std::clamp(u, -1.0, 1.0);

    // Angular droop: the plant multiplies yaw authority by (1 - c_ad*|u_lin_eff|), so the heading
    // loop is weaker at speed than in place. Driven by the uncompensated linear command; see the
    // loop-gain note in compute_linear_command.
    const double u_lin_eff =
        plant_effective_command(prev_linear_uncompensated_, deadzone_, deadzone_);
    u = compensate_coupling(u, u_lin_eff, angular_droop_coeff_, angular_droop_floor_);

    return std::clamp(u, -1.0, 1.0);
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
    // Option 1: a goal inside a hazard makes every layer below spend the match fighting it, so
    // push it to the nearest legal point before anything else runs.
    return push_out_of_hazards(clamped, field.hazards, half_x, half_y);
}

double MotionProfileNavigation::normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

}  // namespace auto_battlebot
