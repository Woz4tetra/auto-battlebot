#include "robot_filter/kalman_motion_estimator.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <magic_enum.hpp>
#include <set>
#include <stdexcept>

#include "diagnostics_logger/diagnostics_logger.hpp"
#include "transform_utils.hpp"

namespace {
void clip_to_field_bounds(auto_battlebot::Position &position,
                          const auto_battlebot::FieldDescription &field, double margin_meters) {
    const double width = field.size.size.x;
    const double height = field.size.size.y;
    if (width <= 0.0 || height <= 0.0) return;
    const double half_x = width * 0.5 + margin_meters;
    const double half_y = height * 0.5 + margin_meters;
    position.x = std::clamp(position.x, -half_x, half_x);
    position.y = std::clamp(position.y, -half_y, half_y);
}

void stop_at_field_bounds(const auto_battlebot::Position &position, double &vx, double &vy,
                          const auto_battlebot::FieldDescription &field, double margin_meters) {
    const double width = field.size.size.x;
    const double height = field.size.size.y;
    if (width <= 0.0 || height <= 0.0) return;
    const double half_x = width * 0.5 + margin_meters;
    const double half_y = height * 0.5 + margin_meters;
    if ((position.x >= half_x && vx > 0.0) || (position.x <= -half_x && vx < 0.0)) vx = 0.0;
    if ((position.y >= half_y && vy > 0.0) || (position.y <= -half_y && vy < 0.0)) vy = 0.0;
}

constexpr int kPx = 0;
constexpr int kPy = 1;
constexpr int kVx = 2;
constexpr int kVy = 3;
constexpr int kTheta = 4;
constexpr int kOmega = 5;
constexpr std::array<bool, 6> kOpponentWrappedAngleStates = {false, false, false,
                                                             false, true,  false};
constexpr double kMaxHeadingVariance = M_PI * M_PI;
constexpr double kFlipTestMaxVariance = (M_PI / 4.0) * (M_PI / 4.0);

auto_battlebot::PositionCovariance position_covariance(
    const auto_battlebot::ekf::Mat<2, 2> &covariance) {
    return {covariance(0, 0), 0.5 * (covariance(0, 1) + covariance(1, 0)), covariance(1, 1)};
}

double position_sigma(const auto_battlebot::ekf::Mat<2, 2> &covariance) {
    const Eigen::SelfAdjointEigenSolver<auto_battlebot::ekf::Mat<2, 2>> eigen(
        covariance, Eigen::EigenvaluesOnly);
    return std::sqrt(std::max(0.0, eigen.eigenvalues().maxCoeff()));
}
}  // namespace

namespace auto_battlebot {

namespace {
JigPlantParams required_plant(const KalmanMotionEstimatorConfiguration &config) {
    if (!config.plant.has_value()) {
        throw std::invalid_argument("KalmanMotionEstimator requires the [plant] table");
    }
    return *config.plant;
}
}  // namespace

KalmanMotionEstimator::KalmanMotionEstimator(const KalmanMotionEstimatorConfiguration &config)
    : config_(config),
      diagnostics_logger_(DiagnosticsLogger::get_logger("kalman_motion_estimator")),
      plant_model_(std::make_unique<JigPlantModel>(required_plant(config), config_.plant_noise)) {}

void KalmanMotionEstimator::reset() {
    opponent_tracks_.clear();
    our_ekf_tracks_.clear();
    command_feedback_ = CommandFeedback{};
    command_history_.clear();
    plant_model_->reset();
    last_field_ = FieldDescription{};
    has_field_ = false;
    last_field_margin_ = 0.0;
    last_clock_gap_warning_ = -std::numeric_limits<double>::infinity();
    min_coast_span_s_ = 0.0;
}

std::optional<KalmanMotionEstimator::OpponentFilterState>
KalmanMotionEstimator::opponent_filter_state(FrameId frame_id) const {
    const auto it = opponent_tracks_.find(frame_id);
    if (it == opponent_tracks_.end()) return std::nullopt;
    return OpponentFilterState{it->second.state, it->second.covariance, it->second.state_stamp,
                               it->second.heading_initialized};
}

void KalmanMotionEstimator::predict(double now, const CommandFeedback &command_feedback) {
    command_feedback_ = command_feedback;
    const auto cmd_it = command_feedback.stick_commands.find(FrameId::OUR_ROBOT_1);
    if (cmd_it != command_feedback.stick_commands.end()) {
        command_history_.push(TimedCommand{now, cmd_it->second});
    }
}

void KalmanMotionEstimator::push_snapshot(OpponentTrack &track) {
    size_t index;
    if (track.snapshot_count == kSnapshotCapacity) {
        index = track.snapshot_start;
        track.snapshot_start = (track.snapshot_start + 1) % kSnapshotCapacity;
    } else {
        index = (track.snapshot_start + track.snapshot_count) % kSnapshotCapacity;
        ++track.snapshot_count;
    }
    track.snapshots[index] =
        Snapshot{track.state_stamp, track.state, track.covariance, track.heading_initialized};
}

bool KalmanMotionEstimator::rewind_opponent(OpponentTrack &track, double target_stamp) {
    size_t found = kSnapshotCapacity;
    for (size_t i = 0; i < track.snapshot_count; ++i) {
        const Snapshot &snapshot = track.snapshots[(track.snapshot_start + i) % kSnapshotCapacity];
        if (snapshot.stamp > target_stamp) break;
        found = i;
    }
    if (found == kSnapshotCapacity) return false;

    const Snapshot &snapshot = track.snapshots[(track.snapshot_start + found) % kSnapshotCapacity];
    track.state = snapshot.state;
    track.covariance = snapshot.covariance;
    track.heading_initialized = snapshot.heading_initialized;
    track.state_stamp = snapshot.stamp;
    track.snapshot_count = found + 1;
    return true;
}

KalmanMotionEstimator::OpponentEstimate KalmanMotionEstimator::predict_opponent(
    const OpponentTrack &track, double target_stamp) const {
    OpponentEstimate estimate{track.state, track.covariance, track.state_stamp};
    const double dt_total = target_stamp - track.state_stamp;
    if (dt_total <= 0.0) return estimate;
    const double dt = std::min(dt_total, kMaxIntegrationStepS);

    const double age = track.state_stamp - track.last_measured_stamp;
    const double move = std::min(dt, std::max(0.0, config_.max_coast_s - age));

    OpponentMat transition = OpponentMat::Identity();
    transition(kPx, kVx) = dt;
    transition(kPy, kVy) = dt;
    transition(kTheta, kOmega) = dt;

    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    OpponentMat process_noise = OpponentMat::Zero();
    const double q = config_.opponent_accel_psd;
    process_noise(kPx, kPx) = process_noise(kPy, kPy) = q * dt3 / 3.0;
    process_noise(kPx, kVx) = process_noise(kVx, kPx) = q * dt2 / 2.0;
    process_noise(kPy, kVy) = process_noise(kVy, kPy) = q * dt2 / 2.0;
    process_noise(kVx, kVx) = process_noise(kVy, kVy) = q * dt;
    const double q_yaw = config_.opponent_yaw_accel_psd;
    process_noise(kTheta, kTheta) = q_yaw * dt3 / 3.0;
    process_noise(kTheta, kOmega) = process_noise(kOmega, kTheta) = q_yaw * dt2 / 2.0;
    process_noise(kOmega, kOmega) = q_yaw * dt;

    if (move > 0.0) {
        OpponentMat motion = OpponentMat::Identity();
        motion(kPx, kVx) = move;
        motion(kPy, kVy) = move;
        motion(kTheta, kOmega) = move;
        estimate.state = motion * estimate.state;
        estimate.state(kTheta) = ekf::wrap_angle(estimate.state(kTheta));
    }
    ekf::propagate_covariance<kOpponentStates>(estimate.covariance, transition, process_noise);
    const double velocity_cap = config_.initial_velocity_sigma * config_.initial_velocity_sigma;
    const double yaw_rate_cap = config_.initial_yaw_rate_sigma * config_.initial_yaw_rate_sigma;
    for (const auto &[index, cap] :
         {std::pair{kVx, velocity_cap}, std::pair{kVy, velocity_cap},
          std::pair{kTheta, kMaxHeadingVariance}, std::pair{kOmega, yaw_rate_cap}}) {
        if (estimate.covariance(index, index) > cap) {
            const double scale = std::sqrt(cap / estimate.covariance(index, index));
            estimate.covariance.row(index) *= scale;
            estimate.covariance.col(index) *= scale;
        }
    }
    estimate.stamp = target_stamp;
    return estimate;
}

void KalmanMotionEstimator::advance_opponent(OpponentTrack &track, double target_stamp) {
    if (target_stamp <= track.state_stamp) return;
    const OpponentEstimate estimate = predict_opponent(track, target_stamp);
    track.state = estimate.state;
    track.covariance = estimate.covariance;
    track.state_stamp = estimate.stamp;
    push_snapshot(track);
}

void KalmanMotionEstimator::advance_our_ekf(OurEkfTrack &track, double target_stamp) {
    const double dt_total = target_stamp - track.stamp;
    if (dt_total <= 0.0) return;
    const double dt = std::min(dt_total, kMaxIntegrationStepS);

    const double age = track.stamp - track.last_measured_stamp;
    const double allowed = std::max(0.0, config_.max_coast_s - age);
    const double move = dt_total > kMaxIntegrationStepS ? 0.0 : std::min(dt, allowed);

    std::vector<TimedCommand> commands;
    command_history_.gather(track.stamp - config_.plant->delay_s, track.stamp + dt, commands);

    ekf::Mat<5, 5> jacobian = ekf::Mat<5, 5>::Identity();
    if (move > 0.0) {
        track.state = plant_model_->propagate(track.state, commands, track.stamp,
                                              track.stamp + move, jacobian);
    }
    ekf::propagate_covariance<5>(track.covariance, jacobian,
                                 plant_model_->process_noise(track.state, commands, dt));
    track.stamp = target_stamp;
}

void KalmanMotionEstimator::init_our_ekf(OurEkfTrack &track, const RobotDescription &measurement,
                                         double stamp, double position_variance) {
    const Pose2D pose = pose_to_pose2d(measurement.pose);
    track.state = PlantState{pose.x, pose.y, pose.yaw, 0.0, 0.0};
    track.covariance = ekf::Mat<5, 5>::Zero();
    track.covariance(0, 0) = track.covariance(1, 1) = position_variance;
    track.covariance(2, 2) =
        config_.keypoint_heading_sigma_rad * config_.keypoint_heading_sigma_rad;
    track.covariance(3, 3) = config_.initial_velocity_sigma * config_.initial_velocity_sigma;
    track.covariance(4, 4) = config_.plant->k_ang * config_.plant->k_ang;
    track.stamp = stamp;
    track.last_measured_stamp = stamp;
    track.consecutive_rejects = 0;
    track.output_template = measurement;
    track.speed_ref_x = pose.x;
    track.speed_ref_y = pose.y;
    track.speed_ref_yaw = pose.yaw;
    track.speed_ref_stamp = stamp;
    track.has_speed_ref = true;
}

RobotDescription KalmanMotionEstimator::render_our_ekf(const OurEkfTrack &track,
                                                       const PlantState &state, bool moving) const {
    RobotDescription description = track.output_template;
    const double z = description.pose.position.z;
    description.pose = pose2d_to_pose(Pose2D{state.x, state.y, state.theta});
    description.pose.position.z = z;
    clip_to_field_bounds(description.pose.position, last_field_, last_field_margin_);
    if (moving) {
        const Eigen::Vector2d velocity_field = body_velocity_to_field(state.v, 0.0, state.theta);
        description.velocity = Velocity2D{velocity_field.x(), velocity_field.y(), state.w};
    } else {
        description.velocity = Velocity2D{};
    }
    description.uncertainty.position_sigma_major_axis_m =
        position_sigma(track.covariance.block<2, 2>(0, 0));
    description.uncertainty.position_covariance_m2 =
        position_covariance(track.covariance.block<2, 2>(0, 0));
    return description;
}

void KalmanMotionEstimator::init_opponent_heading(OpponentTrack &track, double yaw) const {
    track.state(kTheta) = ekf::wrap_angle(yaw);
    track.state(kOmega) = 0.0;
    for (int i : {kTheta, kOmega}) {
        track.covariance.row(i).setZero();
        track.covariance.col(i).setZero();
    }
    track.covariance(kTheta, kTheta) =
        config_.opponent_heading_sigma_rad * config_.opponent_heading_sigma_rad;
    track.covariance(kOmega, kOmega) =
        config_.initial_yaw_rate_sigma * config_.initial_yaw_rate_sigma;
    track.heading_initialized = true;
    track.consecutive_heading_flips = 0;
}

void KalmanMotionEstimator::init_opponent(OpponentTrack &track, const RobotDescription &measurement,
                                          double stamp, double position_variance) {
    track.state = OpponentVec::Zero();
    track.state(kPx) = measurement.pose.position.x;
    track.state(kPy) = measurement.pose.position.y;
    track.covariance = OpponentMat::Zero();
    track.covariance(kPx, kPx) = track.covariance(kPy, kPy) = position_variance;
    const double velocity_variance =
        config_.initial_velocity_sigma * config_.initial_velocity_sigma;
    track.covariance(kVx, kVx) = track.covariance(kVy, kVy) = velocity_variance;
    init_opponent_heading(track, pose_to_pose2d(measurement.pose).yaw);
    track.state_stamp = stamp;
    track.last_measured_stamp = stamp;
    track.consecutive_rejects = 0;
    track.output_template = measurement;
    track.snapshot_start = 0;
    track.snapshot_count = 0;
    push_snapshot(track);
}

ekf::Mat<2, 2> KalmanMotionEstimator::measurement_noise_for(
    const RobotDescription &measurement, const MotionEstimatorContext &context) const {
    double sigma = measurement.group == Group::OURS ? config_.keypoint_position_sigma_m
                                                    : config_.opponent_position_sigma_m;
    if (config_.position_sigma_per_meter > 0.0) {
        const Eigen::Vector3d camera_position =
            context.tf_fieldcenter_from_camera.block<3, 1>(0, 3);
        const Eigen::Vector3d measured_position(
            measurement.pose.position.x, measurement.pose.position.y, measurement.pose.position.z);
        sigma += config_.position_sigma_per_meter * (measured_position - camera_position).norm();
    }
    return ekf::Mat<2, 2>::Identity() * (sigma * sigma);
}

RobotDescription KalmanMotionEstimator::render_opponent(const OpponentTrack &track,
                                                        const OpponentEstimate &estimate,
                                                        const FieldDescription &field,
                                                        bool moving) const {
    const OpponentVec &state = estimate.state;
    const OpponentMat &covariance = estimate.covariance;
    RobotDescription description = track.output_template;
    description.pose.position.x = state(kPx);
    description.pose.position.y = state(kPy);
    double vx = moving ? state(kVx) : 0.0;
    double vy = moving ? state(kVy) : 0.0;
    stop_at_field_bounds(description.pose.position, vx, vy, field, last_field_margin_);
    clip_to_field_bounds(description.pose.position, field, last_field_margin_);

    description.uncertainty.position_sigma_major_axis_m =
        position_sigma(covariance.block<2, 2>(kPx, kPx));
    description.uncertainty.position_covariance_m2 =
        position_covariance(covariance.block<2, 2>(kPx, kPx));

    double omega = 0.0;
    if (track.heading_initialized) {
        const double z = description.pose.position.z;
        description.pose = pose2d_to_pose(
            Pose2D{description.pose.position.x, description.pose.position.y, state(kTheta)});
        description.pose.position.z = z;
        omega = moving ? state(kOmega) : 0.0;
        description.uncertainty.heading_sigma_rad =
            std::sqrt(std::max(0.0, covariance(kTheta, kTheta)));
        description.uncertainty.yaw_rate_sigma_rad_s =
            std::sqrt(std::max(0.0, covariance(kOmega, kOmega)));
    }
    description.velocity = Velocity2D{vx, vy, omega};
    return description;
}

std::vector<RobotDescription> KalmanMotionEstimator::update(
    std::vector<RobotDescription> measurements, double timestamp,
    FrameIdAssigner &frame_id_assigner, const FieldDescription &field,
    const MotionEstimatorContext &context) {
    last_field_ = field;
    has_field_ = true;
    last_field_margin_ = context.field_bounds_margin_meters;

    std::set<FrameId> measured_frame_ids;
    std::set<FrameId> gated_opponent_ids;
    int num_gated = 0;
    int num_reinit = 0;
    int num_rewind_missed = 0;
    int num_heading_flips = 0;
    int num_opponent_heading_flips = 0;
    int num_opponent_heading_reinit = 0;

    for (auto &input : measurements) {
        if (input.frame_id == FrameId::EMPTY) continue;
        measured_frame_ids.insert(input.frame_id);

        if (input.group == Group::OURS) {
            const ekf::Mat<2, 2> measurement_noise = measurement_noise_for(input, context);
            auto [it, inserted] = our_ekf_tracks_.try_emplace(input.frame_id);
            OurEkfTrack &track = it->second;
            if (inserted) {
                init_our_ekf(track, input, timestamp, measurement_noise(0, 0));
                continue;
            }
            advance_our_ekf(track, timestamp);

            const Pose2D measured_pose = pose_to_pose2d(input.pose);
            ekf::Vec<5> state_vec;
            state_vec << track.state.x, track.state.y, track.state.theta, track.state.v,
                track.state.w;

            const bool has_heading =
                std::abs(ekf::wrap_angle(measured_pose.yaw - track.state.theta)) <= M_PI / 2.0;
            if (!has_heading) ++num_heading_flips;

            diagnostics_logger_->debug(
                "our_heading",
                {{"frame_id", std::string(magic_enum::enum_name(input.frame_id))},
                 {"accepted", has_heading ? 1 : 0},
                 {"state_theta_deg", track.state.theta * 180.0 / M_PI},
                 {"state_w_dps", track.state.w * 180.0 / M_PI},
                 {"state_v", track.state.v},
                 {"measured_yaw_deg", measured_pose.yaw * 180.0 / M_PI},
                 {"innovation_deg",
                  ekf::wrap_angle(measured_pose.yaw - track.state.theta) * 180.0 / M_PI},
                 {"cov_theta_deg", std::sqrt(std::max(0.0, track.covariance(2, 2))) * 180.0 / M_PI},
                 {"cov_w_dps", std::sqrt(std::max(0.0, track.covariance(4, 4))) * 180.0 / M_PI},
                 {"stamp_age_s", timestamp - track.last_measured_stamp}});

            double speed_measurement = 0.0;
            double speed_variance = 0.0;
            bool has_speed = false;
            if (has_heading && track.has_speed_ref) {
                const double speed_dt = timestamp - track.speed_ref_stamp;
                if (speed_dt > config_.speed_measurement_max_dt_s) {
                    track.has_speed_ref = false;
                } else if (speed_dt >= config_.speed_measurement_min_dt_s) {
                    const double dx = measured_pose.x - track.speed_ref_x;
                    const double dy = measured_pose.y - track.speed_ref_y;
                    const double mid_yaw =
                        track.speed_ref_yaw +
                        0.5 * ekf::wrap_angle(measured_pose.yaw - track.speed_ref_yaw);
                    speed_measurement =
                        (dx * std::cos(mid_yaw) + dy * std::sin(mid_yaw)) / speed_dt;
                    const double floor_var =
                        config_.speed_sigma_floor_mps * config_.speed_sigma_floor_mps;
                    const double endpoint_var =
                        config_.speed_position_sigma_m * config_.speed_position_sigma_m;
                    speed_variance =
                        std::max(floor_var, 2.0 * endpoint_var / (speed_dt * speed_dt));
                    has_speed = true;
                }
            }
            if (has_speed) {
                diagnostics_logger_->debug(
                    "our_speed", {{"measured_v", speed_measurement},
                                  {"state_v", track.state.v},
                                  {"innovation", speed_measurement - track.state.v},
                                  {"sigma", std::sqrt(speed_variance)},
                                  {"baseline_s", timestamp - track.speed_ref_stamp},
                                  {"cov_v", std::sqrt(std::max(0.0, track.covariance(3, 3)))}});
            }

            ekf::UpdateOutcome outcome;
            if (has_heading) {
                ekf::Mat<3, 5> observation = ekf::Mat<3, 5>::Zero();
                observation(0, 0) = observation(1, 1) = observation(2, 2) = 1.0;
                ekf::Mat<3, 3> noise = ekf::Mat<3, 3>::Zero();
                noise(0, 0) = noise(1, 1) = measurement_noise(0, 0);
                noise(2, 2) =
                    config_.keypoint_heading_sigma_rad * config_.keypoint_heading_sigma_rad;
                const ekf::Vec<3> measured(measured_pose.x, measured_pose.y, measured_pose.yaw);
                const ekf::Vec<3> predicted(track.state.x, track.state.y, track.state.theta);
                outcome = ekf::ekf_update<5, 3>(
                    state_vec, track.covariance, measured, predicted, observation, noise,
                    config_.gate_nis, {false, false, true}, {false, false, true, false, false},
                    config_.covariance_floor);
            } else {
                ekf::Mat<2, 5> observation = ekf::Mat<2, 5>::Zero();
                observation(0, 0) = observation(1, 1) = 1.0;
                const ekf::Vec<2> measured(measured_pose.x, measured_pose.y);
                const ekf::Vec<2> predicted(track.state.x, track.state.y);
                outcome = ekf::ekf_update<5, 2>(state_vec, track.covariance, measured, predicted,
                                                observation, measurement_noise, config_.gate_nis,
                                                {false, false}, {false, false, true, false, false},
                                                config_.covariance_floor);
            }

            if (outcome.accepted && has_speed) {
                ekf::Mat<1, 5> observation = ekf::Mat<1, 5>::Zero();
                observation(0, 3) = 1.0;
                ekf::Mat<1, 1> noise;
                noise(0, 0) = speed_variance;
                ekf::Vec<1> measured;
                measured(0) = speed_measurement;
                ekf::Vec<1> predicted;
                predicted(0) = state_vec(3);
                ekf::ekf_update<5, 1>(state_vec, track.covariance, measured, predicted, observation,
                                      noise, config_.speed_gate_nis, {false},
                                      {false, false, true, false, false}, config_.covariance_floor);
            }

            if (outcome.accepted) {
                track.state = PlantState{state_vec(0), state_vec(1), state_vec(2), state_vec(3),
                                         state_vec(4)};
                track.last_measured_stamp = timestamp;
                track.consecutive_rejects = 0;
                track.output_template = input;
                if (has_speed || !track.has_speed_ref) {
                    track.speed_ref_x = measured_pose.x;
                    track.speed_ref_y = measured_pose.y;
                    track.speed_ref_yaw = measured_pose.yaw;
                    track.speed_ref_stamp = timestamp;
                    track.has_speed_ref = true;
                }
            } else {
                ++num_gated;
                ++track.consecutive_rejects;
                if (track.consecutive_rejects >= config_.reinit_after_rejects) {
                    init_our_ekf(track, input, timestamp, measurement_noise(0, 0));
                    ++num_reinit;
                }
            }
            continue;
        }

        diagnostics_logger_->debug(
            "opponent_measurement_" + std::string(magic_enum::enum_name(input.frame_id)),
            {{"stamp", timestamp},
             {"x", input.pose.position.x},
             {"y", input.pose.position.y},
             {"yaw", pose_to_pose2d(input.pose).yaw},
             {"label", std::string(magic_enum::enum_name(input.label))}});

        const ekf::Mat<2, 2> measurement_noise = measurement_noise_for(input, context);
        auto [it, inserted] = opponent_tracks_.try_emplace(input.frame_id);
        OpponentTrack &track = it->second;
        if (inserted) {
            init_opponent(track, input, timestamp, measurement_noise(0, 0));
            continue;
        }

        if (timestamp < track.state_stamp && !rewind_opponent(track, timestamp)) {
            ++num_rewind_missed;
        }
        advance_opponent(track, timestamp);

        const ekf::Vec<2> measured_position(input.pose.position.x, input.pose.position.y);
        const ekf::Vec<2> predicted_position(track.state(kPx), track.state(kPy));
        ekf::Mat<2, kOpponentStates> observation = ekf::Mat<2, kOpponentStates>::Zero();
        observation(0, kPx) = 1.0;
        observation(1, kPy) = 1.0;
        const ekf::UpdateOutcome outcome = ekf::ekf_update<kOpponentStates, 2>(
            track.state, track.covariance, measured_position, predicted_position, observation,
            measurement_noise, config_.gate_nis, {false, false}, kOpponentWrappedAngleStates,
            config_.covariance_floor);

        if (outcome.accepted) {
            const double measured_yaw = pose_to_pose2d(input.pose).yaw;
            if (!track.heading_initialized) {
                init_opponent_heading(track, measured_yaw);
            } else if (std::abs(ekf::wrap_angle(measured_yaw - track.state(kTheta))) <=
                           M_PI / 2.0 ||
                       track.covariance(kTheta, kTheta) >= kFlipTestMaxVariance) {
                ekf::Mat<1, kOpponentStates> heading_observation =
                    ekf::Mat<1, kOpponentStates>::Zero();
                heading_observation(0, kTheta) = 1.0;
                ekf::Mat<1, 1> heading_noise;
                heading_noise(0, 0) =
                    config_.opponent_heading_sigma_rad * config_.opponent_heading_sigma_rad;
                ekf::Vec<1> measured_heading;
                measured_heading(0) = measured_yaw;
                ekf::Vec<1> predicted_heading;
                predicted_heading(0) = track.state(kTheta);
                ekf::ekf_update<kOpponentStates, 1>(
                    track.state, track.covariance, measured_heading, predicted_heading,
                    heading_observation, heading_noise, 0.0, {true}, kOpponentWrappedAngleStates,
                    config_.covariance_floor);
                track.consecutive_heading_flips = 0;
            } else {
                ++num_opponent_heading_flips;
                if (++track.consecutive_heading_flips >= config_.heading_flip_reinit_count) {
                    init_opponent_heading(track, measured_yaw);
                    ++num_opponent_heading_reinit;
                }
            }
        }

        if (outcome.accepted) {
            track.last_measured_stamp = timestamp;
            track.consecutive_rejects = 0;
            track.output_template = input;
            push_snapshot(track);
        } else {
            ++num_gated;
            ++track.consecutive_rejects;
            if (track.consecutive_rejects >= config_.reinit_after_rejects) {
                init_opponent(track, input, timestamp, measurement_noise(0, 0));
                ++num_reinit;
            } else {
                gated_opponent_ids.insert(input.frame_id);
            }
        }
    }

    for (auto &[frame_id, track] : our_ekf_tracks_) {
        if (measured_frame_ids.count(frame_id) != 0) continue;
        advance_our_ekf(track, timestamp);
        Position predicted_position{track.state.x, track.state.y,
                                    track.output_template.pose.position.z};
        clip_to_field_bounds(predicted_position, field, context.field_bounds_margin_meters);
        frame_id_assigner.set_last_position(frame_id, predicted_position);
    }

    for (auto &[frame_id, track] : opponent_tracks_) {
        if (measured_frame_ids.count(frame_id) == 0 || gated_opponent_ids.count(frame_id) != 0) {
            advance_opponent(track, timestamp);
            Position predicted_position{track.state(kPx), track.state(kPy),
                                        track.output_template.pose.position.z};
            clip_to_field_bounds(predicted_position, field, context.field_bounds_margin_meters);
            frame_id_assigner.set_last_position(frame_id, predicted_position);
        }
    }

    const int num_expired = expire_tracks(frame_id_assigner);

    std::vector<RobotDescription> outputs;
    outputs.reserve(our_ekf_tracks_.size() + opponent_tracks_.size());
    for (const auto &[frame_id, track] : our_ekf_tracks_) {
        outputs.push_back(
            render_our_ekf(track, track.state, is_moving(track.stamp, track.last_measured_stamp)));
    }
    for (const auto &[frame_id, track] : opponent_tracks_) {
        const OpponentEstimate current{track.state, track.covariance, track.state_stamp};
        outputs.push_back(render_opponent(track, current, field,
                                          is_moving(track.state_stamp, track.last_measured_stamp)));
        const PoseUncertainty &uncertainty = outputs.back().uncertainty;
        DiagnosticsData fields{
            {"position_sigma_major_axis_m", uncertainty.position_sigma_major_axis_m.value_or(0.0)},
            {"consecutive_heading_flips", track.consecutive_heading_flips}};
        if (uncertainty.heading_sigma_rad) {
            fields["heading_sigma_rad"] = *uncertainty.heading_sigma_rad;
            fields["yaw_rate_sigma_rad_s"] = uncertainty.yaw_rate_sigma_rad_s.value_or(0.0);
        }
        diagnostics_logger_->debug("opponent_" + std::string(magic_enum::enum_name(frame_id)),
                                   fields);
    }

    diagnostics_logger_->debug({{"num_our_tracks", static_cast<int>(our_ekf_tracks_.size())},
                                {"num_opponent_tracks", static_cast<int>(opponent_tracks_.size())},
                                {"num_measurements", static_cast<int>(measurements.size())},
                                {"num_gated", num_gated},
                                {"num_reinit", num_reinit},
                                {"num_expired", num_expired},
                                {"num_rewind_missed", num_rewind_missed},
                                {"num_heading_flips", num_heading_flips},
                                {"num_opponent_heading_flips", num_opponent_heading_flips},
                                {"num_opponent_heading_reinit", num_opponent_heading_reinit},
                                {"min_coast_span_s", min_coast_span_s_}});
    return outputs;
}

bool KalmanMotionEstimator::is_moving(double render_stamp, double last_measured_stamp) const {
    return render_stamp - last_measured_stamp < config_.max_coast_s;
}

int KalmanMotionEstimator::expire_tracks(FrameIdAssigner &frame_id_assigner) {
    int num_expired = 0;
    const auto expire = [&](auto &tracks, const auto &sigma_of) {
        for (auto it = tracks.begin(); it != tracks.end();) {
            if (sigma_of(it->second) <= config_.max_position_sigma_m) {
                ++it;
                continue;
            }
            frame_id_assigner.clear_last_position(it->first);
            it = tracks.erase(it);
            ++num_expired;
        }
    };
    expire(our_ekf_tracks_, [](const OurEkfTrack &track) {
        return position_sigma(track.covariance.block<2, 2>(0, 0));
    });
    expire(opponent_tracks_, [](const OpponentTrack &track) {
        return position_sigma(track.covariance.block<2, 2>(kPx, kPx));
    });
    return num_expired;
}

std::optional<std::vector<RobotDescription>> KalmanMotionEstimator::coast(double now) {
    std::vector<RobotDescription> outputs;
    outputs.reserve(our_ekf_tracks_.size() + opponent_tracks_.size());

    const double target = now + render_lead_s();
    double worst_span = std::numeric_limits<double>::infinity();

    for (const auto &[frame_id, track] : our_ekf_tracks_) {
        PlantState state = track.state;
        const double dt = target - track.stamp;
        worst_span = std::min(worst_span, dt);
        const double age = track.stamp - track.last_measured_stamp;
        const double allowed = std::max(0.0, config_.max_coast_s - age);
        const double move = dt > kMaxIntegrationStepS ? 0.0 : std::min(dt, allowed);
        if (move > 0.0) {
            std::vector<TimedCommand> commands;
            command_history_.gather(track.stamp - config_.plant->delay_s, track.stamp + move,
                                    commands);
            state =
                plant_model_->propagate_unwrapped(state, commands, track.stamp, track.stamp + move);
            state.theta = ekf::wrap_angle(state.theta);
        }
        outputs.push_back(
            render_our_ekf(track, state, is_moving(target, track.last_measured_stamp)));
    }

    for (const auto &[frame_id, track] : opponent_tracks_) {
        worst_span = std::min(worst_span, target - track.state_stamp);
        const OpponentEstimate predicted = predict_opponent(track, target);
        outputs.push_back(render_opponent(track, predicted, last_field_,
                                          is_moving(predicted.stamp, track.last_measured_stamp)));
    }

    if (std::isfinite(worst_span)) {
        min_coast_span_s_ = worst_span;
        const bool too_far = worst_span > kMaxIntegrationStepS;
        const bool no_span = worst_span <= 0.0;
        if ((too_far || no_span) &&
            now - last_clock_gap_warning_ > config_.clock_gap_warn_period_s) {
            last_clock_gap_warning_ = now;
            spdlog::warn(
                "Coast span {:.3f}s ({}): render target {:.3f} vs newest track stamp {:.3f}; the "
                "control clock is out of step with the camera frame stamps",
                worst_span, too_far ? "past the integration cap" : "not positive", target,
                target - worst_span);
        }
    }
    return outputs;
}

}  // namespace auto_battlebot
