#include "robot_filter/kalman_motion_estimator.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <magic_enum.hpp>
#include <set>

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

/**
 * Zeroes the velocity components pointing into a wall the unclipped center has reached. Same
 * bounds and margin as clip_to_field_bounds, so the two agree on where the wall is: an opponent
 * pinned on a wall renders as stopped instead of still driving into it.
 */
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
/** theta is the one angle state. */
constexpr std::array<bool, 6> kOpponentAngleStates = {false, false, false, false, true, false};
/** Heading variance cap: past pi^2 the heading is unknown, and growing further means nothing. */
constexpr double kMaxHeadingVariance = M_PI * M_PI;
/** The flip test only means something while the heading's own 2-sigma is inside its pi/2: past
 * that, a measurement more than pi/2 away is as likely a turn the track did not see as a flip. */
constexpr double kFlipTestMaxVariance = (M_PI / 4.0) * (M_PI / 4.0);
}  // namespace

namespace auto_battlebot {

KalmanMotionEstimator::KalmanMotionEstimator(const KalmanMotionEstimatorConfiguration &config)
    : config_(config),
      diagnostics_logger_(DiagnosticsLogger::get_logger("kalman_motion_estimator")),
      our_dr_plant_(config.plant.value_or(JigPlantParams{})) {
    // Config validation requires the plant table in both our-robot modes: EKF propagates through
    // JigPlantModel, DEAD_RECKONING scales stick with our_dr_plant_.
    if (config_.our_robot_mode == OurRobotMode::EKF && config_.plant.has_value()) {
        plant_model_ = std::make_unique<JigPlantModel>(*config_.plant, config_.plant_noise);
    }
}

void KalmanMotionEstimator::reset() {
    opponent_tracks_.clear();
    held_opponent_tracks_.clear();
    our_tracks_.clear();
    our_ekf_tracks_.clear();
    command_feedback_ = CommandFeedback{};
    command_history_.clear();
    if (plant_model_) plant_model_->reset();
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
    return OpponentFilterState{it->second.state, it->second.covariance, it->second.stamp,
                               it->second.heading_initialized};
}

void KalmanMotionEstimator::predict(double now, const CommandFeedback &command_feedback) {
    command_feedback_ = command_feedback;
    // The plant model consumes the normalized [-1, 1] stick command directly: it applies the
    // fitted gains itself, so anything pre-scaled to m/s would apply them twice (a 40x yaw-rate
    // error at the Mrs Buff Mk3 scaling).
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
        Snapshot{track.stamp, track.state, track.covariance, track.heading_initialized};
}

bool KalmanMotionEstimator::rewind_opponent(OpponentTrack &track, double target_stamp) {
    // Newest snapshot at or before the measurement time. Snapshots are stored in time order.
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
    track.stamp = snapshot.stamp;
    // Snapshots after the restored one describe a future that no longer exists.
    track.snapshot_count = found + 1;
    return true;
}

void KalmanMotionEstimator::advance_opponent(OpponentTrack &track, double target_stamp) {
    const double dt_total = target_stamp - track.stamp;
    if (dt_total <= 0.0) return;
    const double dt = std::min(dt_total, kMaxIntegrationStepS);

    // Past the coast horizon (or across an absurd clock gap) hold position and heading:
    // predicting a multi-second gap with a constant-rate model is fiction. A step that crosses the
    // horizon integrates only its part before it; the first coast after a measurement spans the
    // whole perception latency plus the render lead, so deciding per step would overshoot any
    // horizon shorter than that. The covariance still propagates through the full transition, so
    // the position sigma keeps growing with the velocity uncertainty. Holding it too would leave a
    // sigma of centimetres on a track a second stale, and the gate would then reject the
    // reacquisition until reinit_after_rejects ran out.
    const double age = track.stamp - track.last_measured_stamp;
    const double move = dt_total > kMaxIntegrationStepS
                            ? 0.0
                            : std::min(dt, std::max(0.0, config_.opponent_max_coast_s - age));

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
        track.state = motion * track.state;
        track.state(kTheta) = ekf::wrap_angle(track.state(kTheta));
    }
    ekf::propagate_covariance<kOpponentStates>(track.covariance, transition, process_noise);
    // Caps: past pi the heading is unknown, and an unmeasured track's velocity and yaw rate are
    // no less known than a newborn track's. Without the rate caps the position variance of a track
    // unseen for minutes grows with t^3. Scaling a row and its column together keeps P positive
    // semidefinite.
    const double velocity_cap = config_.initial_velocity_sigma * config_.initial_velocity_sigma;
    const double yaw_rate_cap = config_.initial_yaw_rate_sigma * config_.initial_yaw_rate_sigma;
    for (const auto &[index, cap] :
         {std::pair{kVx, velocity_cap}, std::pair{kVy, velocity_cap},
          std::pair{kTheta, kMaxHeadingVariance}, std::pair{kOmega, yaw_rate_cap}}) {
        if (track.covariance(index, index) > cap) {
            const double scale = std::sqrt(cap / track.covariance(index, index));
            track.covariance.row(index) *= scale;
            track.covariance.col(index) *= scale;
        }
    }
    track.stamp = target_stamp;
    push_snapshot(track);
}

void KalmanMotionEstimator::advance_our_ekf(OurEkfTrack &track, double target_stamp) {
    const double dt_total = target_stamp - track.stamp;
    if (dt_total <= 0.0) return;
    const double dt = std::min(dt_total, kMaxIntegrationStepS);

    // Pose integration stops at the coast horizon (predicting further with this model is
    // fiction, same as the opponent arm), but only for the span past it: a step that crosses
    // the horizon still integrates its measured-side portion.
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
    // Process noise covers the whole step, held or not: time passing without measurements is
    // uncertainty regardless of whether the pose is allowed to move.
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
    // Full-stick yaw rate as the initial yaw-rate sigma: the fitted gain is the scale of what
    // the drivetrain can be doing while unobserved.
    track.covariance(4, 4) = config_.plant->k_ang * config_.plant->k_ang;
    track.stamp = stamp;
    track.last_measured_stamp = stamp;
    track.consecutive_rejects = 0;
    track.output_stale = false;
    track.description = measurement;
    track.speed_ref_x = pose.x;
    track.speed_ref_y = pose.y;
    track.speed_ref_yaw = pose.yaw;
    track.speed_ref_stamp = stamp;
    track.has_speed_ref = !measurement.keypoints.empty();
}

RobotDescription KalmanMotionEstimator::render_our_ekf(const OurEkfTrack &track,
                                                       const PlantState &state, bool stale) const {
    RobotDescription description = track.description;
    const double z = description.pose.position.z;
    description.pose = pose2d_to_pose(Pose2D{state.x, state.y, state.theta});
    description.pose.position.z = z;
    clip_to_field_bounds(description.pose.position, last_field_, last_field_margin_);
    const Eigen::Vector2d velocity_field = body_velocity_to_field(state.v, 0.0, state.theta);
    description.velocity = Velocity2D{velocity_field.x(), velocity_field.y(), state.w};
    description.is_stale = stale;
    return description;
}

bool KalmanMotionEstimator::measures_heading(const RobotDescription &measurement) const {
    if (measurement.keypoints.empty()) return false;
    const auto &blind = config_.heading_blind_labels;
    return std::find(blind.begin(), blind.end(), measurement.label) == blind.end();
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
    track.consecutive_flips = 0;
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
    if (measures_heading(measurement)) {
        init_opponent_heading(track, pose_to_pose2d(measurement.pose).yaw);
    } else {
        // Unknown heading: variance at the cap, and the render reports it as such.
        track.covariance(kTheta, kTheta) = kMaxHeadingVariance;
        track.covariance(kOmega, kOmega) =
            config_.initial_yaw_rate_sigma * config_.initial_yaw_rate_sigma;
        track.heading_initialized = false;
        track.consecutive_flips = 0;
    }
    track.stamp = stamp;
    track.last_measured_stamp = stamp;
    track.consecutive_rejects = 0;
    track.output_stale = false;
    track.description = measurement;
    track.snapshot_start = 0;
    track.snapshot_count = 0;
    push_snapshot(track);
}

ekf::Mat<2, 2> KalmanMotionEstimator::measurement_noise_for(
    const RobotDescription &measurement, const MotionEstimatorContext &context) const {
    // Keypoint-derived poses carry their front/back points; blob centroids do not.
    const bool is_blob = measurement.keypoints.empty();
    const double keypoint_sigma = measurement.group == Group::OURS
                                      ? config_.keypoint_position_sigma_m
                                      : config_.opponent_position_sigma_m;
    double sigma = is_blob ? config_.blob_position_sigma_m : keypoint_sigma;
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
                                                        const FieldDescription &field,
                                                        bool stale) const {
    RobotDescription description = track.description;
    description.pose.position.x = track.state(kPx);
    description.pose.position.y = track.state(kPy);
    double vx = track.state(kVx);
    double vy = track.state(kVy);
    stop_at_field_bounds(description.pose.position, vx, vy, field, last_field_margin_);
    clip_to_field_bounds(description.pose.position, field, last_field_margin_);

    const Eigen::SelfAdjointEigenSolver<ekf::Mat<2, 2>> position_eigen(
        track.covariance.block<2, 2>(kPx, kPx), Eigen::EigenvaluesOnly);
    description.uncertainty.position_sigma_m =
        std::sqrt(std::max(0.0, position_eigen.eigenvalues().maxCoeff()));

    // A heading-blind label or a never-measured heading has nothing to render: the rotation stays
    // the latest measurement's and the sigmas stay infinite.
    const bool heading_known =
        track.heading_initialized &&
        std::find(config_.heading_blind_labels.begin(), config_.heading_blind_labels.end(),
                  description.label) == config_.heading_blind_labels.end();
    double omega = 0.0;
    if (heading_known) {
        const double z = description.pose.position.z;
        description.pose = pose2d_to_pose(
            Pose2D{description.pose.position.x, description.pose.position.y, track.state(kTheta)});
        description.pose.position.z = z;
        omega = track.state(kOmega);
        description.uncertainty.heading_sigma_rad =
            std::sqrt(std::max(0.0, track.covariance(kTheta, kTheta)));
        description.uncertainty.yaw_rate_sigma_rad_s =
            std::sqrt(std::max(0.0, track.covariance(kOmega, kOmega)));
    }
    description.velocity = Velocity2D{vx, vy, omega};
    description.is_stale = stale;
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
            if (!plant_model_) {
                HeldTrack &track = our_tracks_[input.frame_id];
                track.description = input;
                track.last_measured_stamp = timestamp;
                continue;
            }

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

            // Blob centroids carry no heading, and a keypoint heading flipped past pi/2 is
            // the front/back converter mislabeling, not the robot teleporting: both correct
            // the position rows only.
            const bool has_heading =
                !input.keypoints.empty() &&
                std::abs(ekf::wrap_angle(measured_pose.yaw - track.state.theta)) <= M_PI / 2.0;
            if (!input.keypoints.empty() && !has_heading) ++num_heading_flips;

            // Heading is the input the motion profile brakes on: a rejected keypoint yaw leaves
            // theta running on the plant model alone, so record the state and the innovation that
            // was thrown away. Without both sides here a diverged theta is indistinguishable from a
            // bad measurement.
            if (!input.keypoints.empty()) {
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
                     {"cov_theta_deg",
                      std::sqrt(std::max(0.0, track.covariance(2, 2))) * 180.0 / M_PI},
                     {"cov_w_dps", std::sqrt(std::max(0.0, track.covariance(4, 4))) * 180.0 / M_PI},
                     {"stamp_age_s", timestamp - track.last_measured_stamp}});
            }

            // Forward speed differenced against an earlier accepted pose. The state's v row is
            // otherwise unobserved, so it tracks the plant model rather than the robot, and the
            // motion profile brakes on that number. Only attempted alongside an accepted heading:
            // the difference has to be projected onto a direction, and a rejected yaw is not one.
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
                    // Project onto the heading halfway through the baseline. Over an arc the
                    // endpoint heading points off the chord, and this drivetrain turns fast
                    // enough for that to bias the projection.
                    const double mid_yaw =
                        track.speed_ref_yaw +
                        0.5 * ekf::wrap_angle(measured_pose.yaw - track.speed_ref_yaw);
                    speed_measurement =
                        (dx * std::cos(mid_yaw) + dy * std::sin(mid_yaw)) / speed_dt;
                    // Independent position noise at both ends, propagated through the divide,
                    // held above a floor for the part that is common-mode.
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

            // Speed corrects in a second update rather than as another row above. Sharing the
            // gate would let one noisy finite difference discard the position fix that arrived
            // with it, and position is the measurement this track cannot do without. Its own
            // gate can drop the speed row alone.
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
                track.output_stale = false;
                track.description = input;
                // Only move the reference once its baseline has been spent, or when there is
                // none to spend. Advancing every frame would pin the baseline at one frame and
                // put the speed row back in the noise it was widened to escape.
                if (has_speed || !track.has_speed_ref) {
                    track.speed_ref_x = measured_pose.x;
                    track.speed_ref_y = measured_pose.y;
                    track.speed_ref_yaw = measured_pose.yaw;
                    track.speed_ref_stamp = timestamp;
                    track.has_speed_ref = !input.keypoints.empty();
                }
            } else {
                ++num_gated;
                track.output_stale = true;
                ++track.consecutive_rejects;
                if (track.consecutive_rejects >= config_.reinit_after_rejects) {
                    init_our_ekf(track, input, timestamp, measurement_noise(0, 0));
                    ++num_reinit;
                }
            }
            continue;
        }

        // Every opponent measurement as the estimator received it, in either mode, so a replay
        // recording can be run back through the Python mirror offline.
        diagnostics_logger_->debug(
            "opponent_measurement_" + std::string(magic_enum::enum_name(input.frame_id)),
            {{"stamp", timestamp},
             {"x", input.pose.position.x},
             {"y", input.pose.position.y},
             {"yaw", pose_to_pose2d(input.pose).yaw},
             {"keypoint", input.keypoints.empty() ? 0 : 1},
             {"label", std::string(magic_enum::enum_name(input.label))}});

        if (config_.opponent_mode == OpponentMode::HOLD) {
            HeldTrack &track = held_opponent_tracks_[input.frame_id];
            track.description = input;
            track.stamp = timestamp;
            track.last_measured_stamp = timestamp;
            track.output_stale = false;
            continue;
        }

        const ekf::Mat<2, 2> measurement_noise = measurement_noise_for(input, context);
        auto [it, inserted] = opponent_tracks_.try_emplace(input.frame_id);
        OpponentTrack &track = it->second;
        if (inserted) {
            init_opponent(track, input, timestamp, measurement_noise(0, 0));
            continue;
        }

        // A measurement stamped before the track state (perception latency behind the control
        // clock) is folded in at its own time: rewind to a snapshot, correct, and let the next
        // coast() re-propagate. Without an old-enough snapshot the correction happens at the
        // track's current state instead, which biases against the direction of motion; count it.
        if (timestamp < track.stamp && !rewind_opponent(track, timestamp)) {
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
            measurement_noise, config_.gate_nis, {false, false}, kOpponentAngleStates,
            config_.covariance_floor);

        // The heading row of the [px, py, theta] update. The translation and heading blocks
        // never correlate (block-diagonal F, Q, initial P, and every observation touches one
        // block), so correcting them in sequence is the joint 3-row update. Applied separately so
        // the chi-square gate stays on position: a keypoint heading off by 60 degrees must not
        // cost the track its position fix. The flip test at pi/2 is the heading's gate, applied
        // only while the heading is known well enough to call a flip (kFlipTestMaxVariance).
        if (outcome.accepted && measures_heading(input)) {
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
                ekf::ekf_update<kOpponentStates, 1>(track.state, track.covariance, measured_heading,
                                                    predicted_heading, heading_observation,
                                                    heading_noise, 0.0, {true},
                                                    kOpponentAngleStates, config_.covariance_floor);
                track.consecutive_flips = 0;
            } else {
                ++num_opponent_heading_flips;
                if (++track.consecutive_flips >= config_.heading_flip_reinit_count) {
                    init_opponent_heading(track, measured_yaw);
                    ++num_opponent_heading_reinit;
                }
            }
        }

        if (outcome.accepted) {
            track.last_measured_stamp = timestamp;
            track.consecutive_rejects = 0;
            track.output_stale = false;
            track.description = input;
            push_snapshot(track);
        } else {
            ++num_gated;
            track.output_stale = true;
            ++track.consecutive_rejects;
            if (track.consecutive_rejects >= config_.reinit_after_rejects) {
                // Fast recovery beats a graceful wrong answer: after this many consistent
                // rejects the world moved, not the measurement.
                init_opponent(track, input, timestamp, measurement_noise(0, 0));
                ++num_reinit;
            }
        }
    }

    // Our robot: dead-reckon unmeasured tracks with the last commanded velocity, exactly as
    // RobotTemporalMotionFilter does.
    for (auto &[frame_id, track] : our_tracks_) {
        if (measured_frame_ids.count(frame_id) != 0) {
            track.output_stale = false;
        } else {
            track.output_stale = true;
            const auto cmd_it = command_feedback_.stick_commands.find(frame_id);
            const double dt = timestamp - track.stamp;
            if (cmd_it != command_feedback_.stick_commands.end() && dt > 0.0 && dt <= 1.0) {
                const VelocityCommand cmd =
                    plant_stick_to_body_velocity(cmd_it->second, our_dr_plant_);
                Pose2D predicted_pose = pose_to_pose2d(track.description.pose);
                const Eigen::Vector2d velocity_field =
                    body_velocity_to_field(cmd.linear_x, cmd.linear_y, predicted_pose.yaw);
                predicted_pose.x += velocity_field.x() * dt;
                predicted_pose.y += velocity_field.y() * dt;
                predicted_pose.yaw = ekf::wrap_angle(predicted_pose.yaw + cmd.angular_z * dt);
                track.description.pose = pose2d_to_pose(predicted_pose);
                clip_to_field_bounds(track.description.pose.position, field,
                                     context.field_bounds_margin_meters);
                frame_id_assigner.set_last_position(frame_id, track.description.pose.position);
            }
        }
        track.stamp = timestamp;
    }

    // Our robot in ekf mode: propagate unmeasured tracks through the plant model, which is
    // the coast behavior the jig fit exists to make honest.
    for (auto &[frame_id, track] : our_ekf_tracks_) {
        if (measured_frame_ids.count(frame_id) != 0) continue;
        track.output_stale = true;
        advance_our_ekf(track, timestamp);
        Position predicted_position{track.state.x, track.state.y,
                                    track.description.pose.position.z};
        clip_to_field_bounds(predicted_position, field, context.field_bounds_margin_meters);
        frame_id_assigner.set_last_position(frame_id, predicted_position);
    }

    // Hold-mode opponents without a measurement this frame: stale, pinned in place, and the
    // assigner keeps associating against the held pose.
    for (auto &[frame_id, track] : held_opponent_tracks_) {
        if (measured_frame_ids.count(frame_id) != 0) continue;
        track.output_stale = true;
        track.stamp = timestamp;
        frame_id_assigner.set_last_position(frame_id, track.description.pose.position);
    }

    // Opponents without an accepted measurement this frame: predict to the frame time and keep
    // the assigner's association anchor on the predicted position.
    for (auto &[frame_id, track] : opponent_tracks_) {
        if (measured_frame_ids.count(frame_id) == 0) track.output_stale = true;
        if (track.output_stale) {
            advance_opponent(track, timestamp);
            Position predicted_position{track.state(kPx), track.state(kPy),
                                        track.description.pose.position.z};
            clip_to_field_bounds(predicted_position, field, context.field_bounds_margin_meters);
            frame_id_assigner.set_last_position(frame_id, predicted_position);
        }
    }

    // Stale-identity decay for our robot, matching RobotTemporalMotionFilter: drop the track
    // once its last confirmation is older than the hold window instead of predicting forever.
    if (context.our_robot_hold_window_s > 0.0) {
        for (auto it = our_tracks_.begin(); it != our_tracks_.end();) {
            const bool expired =
                it->second.output_stale &&
                (timestamp - it->second.last_measured_stamp) > context.our_robot_hold_window_s;
            it = expired ? our_tracks_.erase(it) : ++it;
        }
        for (auto it = our_ekf_tracks_.begin(); it != our_ekf_tracks_.end();) {
            const bool expired =
                it->second.output_stale &&
                (timestamp - it->second.last_measured_stamp) > context.our_robot_hold_window_s;
            it = expired ? our_ekf_tracks_.erase(it) : ++it;
        }
    }

    std::vector<RobotDescription> outputs;
    outputs.reserve(our_tracks_.size() + our_ekf_tracks_.size() + opponent_tracks_.size() +
                    held_opponent_tracks_.size());
    for (const auto &[frame_id, track] : our_tracks_) {
        RobotDescription output = track.description;
        output.is_stale = track.output_stale;
        outputs.push_back(std::move(output));
    }
    for (const auto &[frame_id, track] : our_ekf_tracks_) {
        outputs.push_back(render_our_ekf(track, track.state, track.output_stale));
    }
    for (const auto &[frame_id, track] : opponent_tracks_) {
        outputs.push_back(render_opponent(track, field, track.output_stale));
        const PoseUncertainty &uncertainty = outputs.back().uncertainty;
        // Infinite sigmas (heading never measured, heading-blind label) are left out rather
        // than written as a non-finite number.
        DiagnosticsData fields{{"position_sigma_m", uncertainty.position_sigma_m},
                               {"consecutive_heading_flips", track.consecutive_flips}};
        if (std::isfinite(uncertainty.heading_sigma_rad)) {
            fields["heading_sigma_rad"] = uncertainty.heading_sigma_rad;
            fields["yaw_rate_sigma_rad_s"] = uncertainty.yaw_rate_sigma_rad_s;
        }
        diagnostics_logger_->debug("opponent_" + std::string(magic_enum::enum_name(frame_id)),
                                   fields);
    }
    for (const auto &[frame_id, track] : held_opponent_tracks_) {
        RobotDescription output = track.description;
        output.is_stale = track.output_stale;
        outputs.push_back(std::move(output));
    }

    diagnostics_logger_->debug(
        {{"num_our_tracks", static_cast<int>(our_tracks_.size() + our_ekf_tracks_.size())},
         {"num_opponent_tracks",
          static_cast<int>(opponent_tracks_.size() + held_opponent_tracks_.size())},
         {"num_measurements", static_cast<int>(measurements.size())},
         {"num_gated", num_gated},
         {"num_reinit", num_reinit},
         {"num_rewind_missed", num_rewind_missed},
         {"num_heading_flips", num_heading_flips},
         {"num_opponent_heading_flips", num_opponent_heading_flips},
         {"num_opponent_heading_reinit", num_opponent_heading_reinit},
         {"min_coast_span_s", min_coast_span_s_}});
    return outputs;
}

std::optional<std::vector<RobotDescription>> KalmanMotionEstimator::coast(double now) {
    std::vector<RobotDescription> outputs;
    outputs.reserve(our_tracks_.size() + our_ekf_tracks_.size() + opponent_tracks_.size() +
                    held_opponent_tracks_.size());

    // Two lags separate the shutter from the wheels, and the render has to cover both. `now` is
    // already past the frame stamp by the perception latency, and a command computed from this
    // render bites another plant.delay_s later. Rendering at `now + delay_s` puts every track
    // where it will be when the command lands, which is the only state target selection and
    // navigation can steer by without aiming at the past.
    //
    // The extra span is not extrapolation into unknown commands: JigPlantModel reads each
    // substep's command at t - delay_s, so integrating to now + delay_s consumes exactly the
    // commands already issued up to `now`, the ones in flight.
    const double target = now + render_lead_s();
    // The smallest span any track is propagated over. Both failure modes below emit an estimate
    // frozen at the shutter while looking exactly like a healthy filter that is not moving, so
    // this is worth naming rather than inferring: a span past kMaxIntegrationStepS short-circuits
    // to "hold position", and a non-positive span propagates nothing at all. Both mean the
    // control clock and the frame stamps are not on the same timeline, which is what SVO playback
    // does under SystemClock. The minimum is the right reducer for that question, and it goes out
    // in update()'s diagnostics so a recording answers "did anything propagate, and by how much"
    // without needing a rebuild. It is not the render lead: opponent tracks commit every control
    // tick and so report one tick, while the our-robot track only commits in update() and so
    // reports a whole frame period plus the lead.
    double worst_span = std::numeric_limits<double>::infinity();
    // Staleness stays on the wall clock: it answers "how long since a real measurement", which
    // the lead does not change.
    const auto is_stale = [&](double last_measured_stamp) {
        return (now - last_measured_stamp) > kCoastStaleAgeS;
    };

    // Rendered on a copy, like the dead-reckoning arm: the committed state only advances in
    // update(), so measurements always fold in at or after the track stamp and the our-robot
    // arm needs no snapshot rewind. propagate_unwrapped skips the finite-difference Jacobian
    // (10 extra plant integrations) that a covariance-free render never uses.
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
        const bool stale = track.output_stale || is_stale(track.last_measured_stamp);
        outputs.push_back(render_our_ekf(track, state, stale));
    }

    for (const auto &[frame_id, track] : our_tracks_) {
        RobotDescription output = track.description;
        const auto cmd_it = command_feedback_.stick_commands.find(frame_id);
        const double dt = target - track.stamp;
        if (cmd_it != command_feedback_.stick_commands.end() && dt > 0.0 &&
            dt <= kMaxIntegrationStepS) {
            // The same dead-reckoning step update() will commit at the next perception frame,
            // rendered ahead of time on a copy so the output moves between frames.
            const VelocityCommand cmd = plant_stick_to_body_velocity(cmd_it->second, our_dr_plant_);
            Pose2D predicted_pose = pose_to_pose2d(output.pose);
            const Eigen::Vector2d velocity_field =
                body_velocity_to_field(cmd.linear_x, cmd.linear_y, predicted_pose.yaw);
            predicted_pose.x += velocity_field.x() * dt;
            predicted_pose.y += velocity_field.y() * dt;
            predicted_pose.yaw = ekf::wrap_angle(predicted_pose.yaw + cmd.angular_z * dt);
            output.pose = pose2d_to_pose(predicted_pose);
            clip_to_field_bounds(output.pose.position, last_field_, last_field_margin_);
        }
        output.is_stale = track.output_stale || is_stale(track.last_measured_stamp);
        outputs.push_back(std::move(output));
    }

    for (auto &[frame_id, track] : opponent_tracks_) {
        worst_span = std::min(worst_span, target - track.stamp);
        advance_opponent(track, target);
        const bool stale = track.output_stale || is_stale(track.last_measured_stamp);
        outputs.push_back(render_opponent(track, last_field_, stale));
    }

    for (const auto &[frame_id, track] : held_opponent_tracks_) {
        RobotDescription output = track.description;
        output.is_stale = track.output_stale || is_stale(track.last_measured_stamp);
        outputs.push_back(std::move(output));
    }

    if (std::isfinite(worst_span)) {
        min_coast_span_s_ = worst_span;
        const bool too_far = worst_span > kMaxIntegrationStepS;
        const bool no_span = worst_span <= 0.0;
        if ((too_far || no_span) && now - last_clock_gap_warning_ > kClockGapWarnPeriodS) {
            last_clock_gap_warning_ = now;
            spdlog::warn(
                "Coast span is {:.3f}s ({}): no forward propagation is happening, so the emitted "
                "estimate sits at the shutter instead of leading it. The control clock and the "
                "frame stamps are not on the same timeline; SVO playback needs [clock] type = "
                "'CameraFollowingClock'.",
                worst_span, too_far ? "past the integration cap" : "not positive");
        }
    }
    return outputs;
}

}  // namespace auto_battlebot
