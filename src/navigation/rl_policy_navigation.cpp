#include "navigation/rl_policy_navigation.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <limits>
#include <magic_enum.hpp>
#include <utility>

#include "diagnostics_logger/diagnostics_logger.hpp"
#include "directories.hpp"
#include "hazards/hazard_geometry.hpp"
#include "plant/plant_params.hpp"
#include "transform_utils.hpp"

namespace auto_battlebot {
namespace {

std::filesystem::path resolve_bundle(const std::string &value) {
    std::filesystem::path path(value);
    if (path.is_absolute()) return path;
    return get_project_root() / path;
}

bool is_opponent(FrameId frame_id) {
    return frame_id == FrameId::THEIR_ROBOT_1 || frame_id == FrameId::THEIR_ROBOT_2 ||
           frame_id == FrameId::THEIR_ROBOT_3;
}

double squared_distance(double ax, double ay, double bx, double by) {
    const double dx = ax - bx;
    const double dy = ay - by;
    return dx * dx + dy * dy;
}

struct SpeedAndYawRate {
    double speed;
    double yaw_rate;
};

SpeedAndYawRate measured_speed_and_yaw_rate(const RobotDescription &our_robot) {
    const double yaw = pose_to_pose2d(our_robot.pose).yaw;
    const double speed =
        our_robot.velocity.vx_mps * std::cos(yaw) + our_robot.velocity.vy_mps * std::sin(yaw);
    return {speed, our_robot.velocity.omega_radps};
}

}  // namespace

RlPolicyNavigation::RlPolicyNavigation(const RlPolicyNavigationConfiguration &config,
                                       std::shared_ptr<ClockInterface> clock)
    : config_(config),
      clock_(std::move(clock)),
      logger_(DiagnosticsLogger::get_logger("rl_policy_nav")) {
    if (config_.hazard_barrier_enable) {
        HazardAvoidanceSettings settings;
        settings.tangent_enable = false;
        settings.reverse_distance = 0.0;
        settings.prediction_horizon_s = config_.hazard_prediction_horizon_s;
        settings.max_linear_speed_fwd = config_.plant.k_fwd;
        settings.max_linear_speed_rev = config_.plant.k_rev;
        settings.barrier_horizon_s = config_.plant.tau_lin_d + config_.plant.delay_s;
        settings.barrier_delay_s = config_.plant.delay_s;
        if (settings.barrier_horizon_s <= 0.0) {
            spdlog::warn(
                "RlPolicyNavigation hazard barrier is off: [plant] tau_lin_d + delay_s is 0, so "
                "there is no stopping envelope");
        }
        hazards_.emplace(settings);
    }
}

void RlPolicyNavigation::load() {
    if (!config_.has_opponent_filter) {
        throw ConfigValidationError(
            "RlPolicyNavigation needs [robot_filter.motion_estimator] type = "
            "'KalmanMotionEstimator' (6-state opponent filter); found " +
            config_.opponent_filter_source);
    }

    const rl_policy::PolicyBundle bundle =
        rl_policy::load_policy_bundle(resolve_bundle(config_.policy.bundle));

    auto builder = rl_policy::make_observation_builder(bundle.obs_spec_version);
    if (!builder) {
        throw rl_policy::PolicyBundleError(
            "policy bundle " + bundle.directory.string() + ": obs_spec_version " +
            std::to_string(bundle.obs_spec_version) +
            " has no observation builder in this build; retrain on a spec it has");
    }

    auto policy = rl_policy::make_policy(bundle.policy_kind);
    policy->load(bundle);

    const std::vector<rl_policy::TensorSpec> made = builder->output_specs();
    for (const rl_policy::TensorSpec &needed : policy->input_specs()) {
        const rl_policy::TensorSpec *match = nullptr;
        for (const auto &spec : made) {
            if (spec.name == needed.name) match = &spec;
        }
        if (match == nullptr) {
            throw rl_policy::PolicyBundleError(
                "policy bundle " + bundle.directory.string() + ": the network reads " +
                needed.to_string() + " but the obs_spec_version " +
                std::to_string(bundle.obs_spec_version) + " builder makes " +
                rl_policy::specs_to_string(made));
        }
        if (match->shape != needed.shape) {
            throw rl_policy::PolicyBundleError(
                "policy bundle " + bundle.directory.string() + ": the network reads " +
                needed.to_string() + " but the obs_spec_version " +
                std::to_string(bundle.obs_spec_version) + " builder makes " + match->to_string());
        }
    }

    observation_ = rl_policy::TensorSet(made);
    history_.emplace(builder->history_policy_steps());
    builder_ = std::move(builder);
    policy_ = std::move(policy);
    run_id_ = bundle.run_id;
    policy_period_ticks_ = bundle.policy_period_control_ticks;
    const auto selftest = policy_->selftest_result();
    selftest_summary_ = selftest ? "passed: " + selftest->summary() : "none";
    loaded_ = true;
    spdlog::info(
        "RlPolicyNavigation loaded bundle {} ({} policy, obs spec {}, every {} ticks), "
        "self-test {}",
        run_id_, magic_enum::enum_name(bundle.policy_kind), bundle.obs_spec_version,
        policy_period_ticks_, selftest_summary_);
}

bool RlPolicyNavigation::initialize() {
    if (!loaded_) load();
    policy_->reset();
    history_->clear();
    action_ = rl_policy::ActionCommand{};
    ticks_until_policy_step_ = 0;
    target_frame_id_ = FrameId::EMPTY;
    last_visualization_ = NavigationVisualization{};
    return true;
}

const RobotDescription *RlPolicyNavigation::find_target_opponent(
    const RobotDescriptionsStamped &robots, const RobotDescription &our_robot,
    const TargetSelection &target) {
    double anchor_x = target.pose.x;
    double anchor_y = target.pose.y;
    if (target.mode == BehaviorMode::RUN_AWAY) {
        anchor_x = our_robot.pose.position.x;
        anchor_y = our_robot.pose.position.y;
    }
    const RobotDescription *nearest = nullptr;
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (const auto &robot : robots.descriptions) {
        if (!is_opponent(robot.frame_id)) continue;
        const double distance =
            squared_distance(robot.pose.position.x, robot.pose.position.y, anchor_x, anchor_y);
        if (distance < nearest_distance) {
            nearest_distance = distance;
            nearest = &robot;
        }
    }
    return nearest;
}

VelocityCommand RlPolicyNavigation::update(RobotDescriptionsStamped robots, FieldDescription field,
                                           const TargetSelection &target) {
    last_visualization_ = NavigationVisualization{};
    last_visualization_.header = robots.header;

    const RobotDescription *our_robot = nullptr;
    for (const auto &robot : robots.descriptions) {
        if (robot.frame_id == FrameId::OUR_ROBOT_1) our_robot = &robot;
    }
    const double snapshot_age_s = std::max(0.0, clock_->now() - robots.header.stamp);
    if (!loaded_ || our_robot == nullptr) {
        if (!loaded_) {
            logger_->debug("not_initialized", "No bundle loaded; command zero");
        } else {
            logger_->debug("no_robot", "Our robot not found; command zero");
        }
        ticks_until_policy_step_ = 0;
        last_visualization_.robots = std::move(robots);
        return VelocityCommand{0.0, 0.0, 0.0};
    }

    const RobotDescription *opponent = find_target_opponent(robots, *our_robot, target);
    const FrameId target_frame_id = opponent != nullptr ? opponent->frame_id : FrameId::EMPTY;
    if (target_frame_id != target_frame_id_) {
        policy_->reset();
        target_frame_id_ = target_frame_id;
    }

    if (ticks_until_policy_step_ <= 0) {
        const rl_policy::ObservationContext context{*our_robot, opponent, field, target, *history_};
        builder_->build(context, observation_);
        const auto start = std::chrono::steady_clock::now();
        action_ = policy_->act(observation_);
        const double inference_us =
            std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - start)
                .count();
        if (!std::isfinite(action_.linear) || !std::isfinite(action_.angular)) {
            logger_->error("non_finite_action",
                           "Policy returned a non-finite action; command zero");
            action_ = rl_policy::ActionCommand{};
        }
        const auto [speed, yaw_rate] = measured_speed_and_yaw_rate(*our_robot);
        history_->push(rl_policy::HistoryEntry{action_.linear, action_.angular, speed, yaw_rate});
        ticks_until_policy_step_ = policy_period_ticks_;
        log_step(snapshot_age_s, *our_robot, opponent, field, inference_us);
    }
    --ticks_until_policy_step_;

    VelocityCommand command{action_.linear, 0.0, action_.angular};
    const Pose2D our_pose = pose_to_pose2d(our_robot->pose);
    if (hazards_) {
        const double v_actual = measured_speed_and_yaw_rate(*our_robot).speed;
        const auto barrier =
            hazards_->limit_command(footprint_center(our_pose, our_robot->footprint_center_offset),
                                    field, v_actual, command);
        if (barrier.engaged) {
            logger_->debug("hazard_barrier",
                           {{"policy_linear", action_.linear},
                            {"linear", command.forward_mps},
                            {"v_actual", v_actual},
                            {"braking", barrier.braking ? 1 : 0},
                            {"bound", std::min(barrier.approach_speed_bound_mps, 99.0)}});
        }
    }
    command = plant_stick_to_body_velocity(
        {.forward = command.forward_mps, .lateral = 0.0, .yaw = command.yaw_rate_radps},
        config_.plant);
    last_visualization_.path =
        NavigationPathSegment{our_pose.x, our_pose.y, target.pose.x, target.pose.y};
    last_visualization_.command = command;
    last_visualization_.robots = std::move(robots);
    return command;
}

void RlPolicyNavigation::log_step(double observation_age_s, const RobotDescription &our_robot,
                                  const RobotDescription *opponent, const FieldDescription &field,
                                  double inference_us) {
    const Pose2D our_pose = pose_to_pose2d(our_robot.pose);
    DiagnosticsData values{
        {"linear", action_.linear},
        {"angular", action_.angular},
        {"inference_us", inference_us},
        {"observation_age_s", observation_age_s},
        {"our_x", our_pose.x},
        {"our_y", our_pose.y},
        {"our_yaw", our_pose.yaw},
        {"our_speed", our_robot.velocity.vx_mps * std::cos(our_pose.yaw) +
                          our_robot.velocity.vy_mps * std::sin(our_pose.yaw)},
        {"our_yaw_rate", our_robot.velocity.omega_radps},
        {"target_present", opponent != nullptr ? 1 : 0},
        {"run_id", run_id_},
        {"selftest", selftest_summary_},
    };
    if (!field.hazards.empty()) {
        double nearest = std::numeric_limits<double>::infinity();
        for (const auto &hazard : field.hazards) {
            const double distance = std::hypot(hazard.center.x - our_robot.pose.position.x,
                                               hazard.center.y - our_robot.pose.position.y) -
                                    hazard.object_radius;
            nearest = std::min(nearest, distance);
        }
        values["nearest_hazard_edge_m"] = nearest;
    }
    logger_->debug("policy", values);
}

}  // namespace auto_battlebot
