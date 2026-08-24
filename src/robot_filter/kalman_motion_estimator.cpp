#include "robot_filter/kalman_motion_estimator.hpp"

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
    track.snapshots[index] = Snapshot{track.stamp, track.state, track.covariance};
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
    track.stamp = snapshot.stamp;
    // Snapshots after the restored one describe a future that no longer exists.
    track.snapshot_count = found + 1;
    return true;
}

void KalmanMotionEstimator::advance_opponent(OpponentTrack &track, double target_stamp) {
    const double dt_total = target_stamp - track.stamp;
    if (dt_total <= 0.0) return;
    const double dt = std::min(dt_total, kMaxIntegrationStepS);

    // Past the coast horizon (or across an absurd clock gap) hold position and only grow the
    // covariance: predicting a multi-second gap with a constant-velocity model is fiction.
    const double age = track.stamp - track.last_measured_stamp;
    const bool hold_position = age >= config_.max_coast_s || dt_total > kMaxIntegrationStepS;

    ekf::Mat<4, 4> transition = ekf::Mat<4, 4>::Identity();
    if (!hold_position) {
        transition(0, 2) = dt;
        transition(1, 3) = dt;
    }

    const double q = config_.opponent_accel_psd;
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    ekf::Mat<4, 4> process_noise = ekf::Mat<4, 4>::Zero();
    process_noise(0, 0) = process_noise(1, 1) = q * dt3 / 3.0;
    process_noise(0, 2) = process_noise(2, 0) = q * dt2 / 2.0;
    process_noise(1, 3) = process_noise(3, 1) = q * dt2 / 2.0;
    process_noise(2, 2) = process_noise(3, 3) = q * dt;

    track.state = transition * track.state;
    ekf::propagate_covariance<4>(track.covariance, transition, process_noise);
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

void KalmanMotionEstimator::init_opponent(OpponentTrack &track, const RobotDescription &measurement,
                                          double stamp, double position_variance) {
    track.state << measurement.pose.position.x, measurement.pose.position.y, 0.0, 0.0;
    track.covariance = ekf::Mat<4, 4>::Zero();
    track.covariance(0, 0) = track.covariance(1, 1) = position_variance;
    const double velocity_variance =
        config_.initial_velocity_sigma * config_.initial_velocity_sigma;
    track.covariance(2, 2) = track.covariance(3, 3) = velocity_variance;
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
    double sigma = is_blob ? config_.blob_position_sigma_m : config_.keypoint_position_sigma_m;
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
    description.pose.position.x = track.state(0);
    description.pose.position.y = track.state(1);
    clip_to_field_bounds(description.pose.position, field, last_field_margin_);
    description.velocity = Velocity2D{track.state(2), track.state(3), 0.0};
    // While coasting, a fast track's heading comes from its velocity vector; the measured
    // rotation is kept when a fresh measurement exists or the track is too slow for the
    // velocity direction to mean anything.
    const double speed = std::hypot(track.state(2), track.state(3));
    if (stale && speed >= config_.min_heading_speed) {
        const Pose2D heading_pose{description.pose.position.x, description.pose.position.y,
                                  std::atan2(track.state(3), track.state(2))};
        description.pose.rotation = pose2d_to_pose(heading_pose).rotation;
    }
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
        const ekf::Vec<2> predicted_position(track.state(0), track.state(1));
        ekf::Mat<2, 4> observation = ekf::Mat<2, 4>::Zero();
        observation(0, 0) = 1.0;
        observation(1, 1) = 1.0;
        const ekf::UpdateOutcome outcome = ekf::ekf_update<4, 2>(
            track.state, track.covariance, measured_position, predicted_position, observation,
            measurement_noise, config_.gate_nis, {false, false}, {false, false, false, false},
            config_.covariance_floor);

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
            Position predicted_position{track.state(0), track.state(1),
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
         {"num_heading_flips", num_heading_flips}});
    return outputs;
}

std::optional<std::vector<RobotDescription>> KalmanMotionEstimator::coast(double now) {
    std::vector<RobotDescription> outputs;
    outputs.reserve(our_tracks_.size() + our_ekf_tracks_.size() + opponent_tracks_.size() +
                    held_opponent_tracks_.size());

    // Rendered on a copy, like the dead-reckoning arm: the committed state only advances in
    // update(), so measurements always fold in at or after the track stamp and the our-robot
    // arm needs no snapshot rewind. propagate_unwrapped skips the finite-difference Jacobian
    // (10 extra plant integrations) that a covariance-free render never uses.
    for (const auto &[frame_id, track] : our_ekf_tracks_) {
        PlantState state = track.state;
        const double dt = now - track.stamp;
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
        const bool stale =
            track.output_stale || (now - track.last_measured_stamp) > kCoastStaleAgeS;
        outputs.push_back(render_our_ekf(track, state, stale));
    }

    for (const auto &[frame_id, track] : our_tracks_) {
        RobotDescription output = track.description;
        const auto cmd_it = command_feedback_.stick_commands.find(frame_id);
        const double dt = now - track.stamp;
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
        output.is_stale = track.output_stale || (now - track.last_measured_stamp) > kCoastStaleAgeS;
        outputs.push_back(std::move(output));
    }

    for (auto &[frame_id, track] : opponent_tracks_) {
        advance_opponent(track, now);
        const bool stale =
            track.output_stale || (now - track.last_measured_stamp) > kCoastStaleAgeS;
        outputs.push_back(render_opponent(track, last_field_, stale));
    }

    for (const auto &[frame_id, track] : held_opponent_tracks_) {
        RobotDescription output = track.description;
        output.is_stale = track.output_stale || (now - track.last_measured_stamp) > kCoastStaleAgeS;
        outputs.push_back(std::move(output));
    }
    return outputs;
}

}  // namespace auto_battlebot
