#include "robot_filter/robot_front_back_simple_filter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "data_structures/robot.hpp"
#include "enums/frame_id.hpp"

namespace {
double position_distance(const auto_battlebot::Position &a, const auto_battlebot::Position &b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}
}  // namespace

namespace auto_battlebot {
RobotFrontBackSimpleFilter::RobotFrontBackSimpleFilter(
    RobotFrontBackSimpleFilterConfiguration &config)
    : label_to_frame_ids_(config.label_to_frame_ids),
      default_frame_id_(config.default_frame_id),
      velocity_ema_alpha_(config.velocity_ema_alpha),
      robot_mask_detector_(config.robot_mask_detector_config) {
    diagnostics_logger_ = DiagnosticsLogger::get_logger("robot_front_back_simple_filter");

    FrontBackKeypointConverterConfig converter_config;
    converter_config.front_keypoints = config.front_keypoints;
    converter_config.back_keypoints = config.back_keypoints;
    keypoint_converter_ = std::make_unique<FrontBackKeypointConverter>(converter_config);
}

bool RobotFrontBackSimpleFilter::initialize(const std::vector<RobotConfig> &robots) {
    robot_configs_.clear();
    velocity_state_per_frame_id_.clear();
    robot_mask_detector_.reset();
    for (const auto &robot : robots) {
        robot_configs_[robot.label] = robot;
    }
    return true;
}

bool RobotFrontBackSimpleFilter::set_opponent_count(int count) {
    if (count < 1 || count > 3) {
        return false;
    }
    static const std::vector<FrameId> opponent_frame_ids = {
        FrameId::THEIR_ROBOT_1, FrameId::THEIR_ROBOT_2, FrameId::THEIR_ROBOT_3};
    label_to_frame_ids_[Label::OPPONENT].clear();
    for (int i = 0; i < count; ++i) {
        label_to_frame_ids_[Label::OPPONENT].push_back(opponent_frame_ids[static_cast<size_t>(i)]);
    }
    return true;
}

RobotDescriptionsStamped RobotFrontBackSimpleFilter::update(KeypointsStamped keypoints,
                                                            FieldDescription field,
                                                            CameraInfo camera_info,
                                                            MaskStamped robot_mask,
                                                            CommandFeedback command_feedback) {
    RobotDescriptionsStamped result;
    result.header.frame_id = FrameId::FIELD;
    result.header.stamp = keypoints.header.stamp;

    std::vector<RobotDescription> filter_measurements =
        convert_keypoints_to_measurements(keypoints, field, camera_info);
    diagnostics_logger_->debug({{"num_keypoint_measurements", (int)filter_measurements.size()}});

    if (!robot_mask.mask.mask.empty() && field.child_frame_id != FrameId::EMPTY) {
        auto opponent_measurements = robot_mask_detector_.detect(
            robot_mask.mask.mask, field, camera_info, filter_measurements,
            keypoints.header.stamp);
        diagnostics_logger_->debug(
            {{"num_robot_mask_opponents", (int)opponent_measurements.size()}});
        for (auto &m : opponent_measurements) {
            filter_measurements.push_back(std::move(m));
        }
    }

    result.descriptions = update_filter(filter_measurements, command_feedback);
    estimate_velocities(result.descriptions, result.header.stamp, command_feedback);

    return result;
}

std::vector<RobotDescription> RobotFrontBackSimpleFilter::convert_keypoints_to_measurements(
    KeypointsStamped keypoints, FieldDescription field, CameraInfo camera_info) {
    Eigen::Matrix4d tf_fieldcenter_from_camera = field.tf_camera_from_fieldcenter.tf.inverse();
    std::vector<RobotDescription> filter_measurements;
    auto front_back_mapping = keypoint_converter_->convert(keypoints, field, camera_info);

    // Clear stale last_position for single-FrameId labels that have no detections this frame,
    // so when they reappear we assign without bias from an old position.
    for (const auto &[label, frame_ids] : label_to_frame_ids_) {
        if (frame_ids.size() != 1) continue;
        if (front_back_mapping.count(label) == 0) last_position_per_frame_id_.erase(frame_ids[0]);
    }

    for (const auto &[label, assignments_with_conf] : front_back_mapping) {
        std::vector<MeasurementWithConfidence> valid_measurements =
            build_valid_measurements(tf_fieldcenter_from_camera, label, assignments_with_conf);
        std::vector<FrameId> frame_ids = get_frame_ids_for_label(label);

        if (valid_measurements.empty()) {
            if (frame_ids.size() == 1) last_position_per_frame_id_.erase(frame_ids[0]);
            continue;
        }

        std::sort(valid_measurements.begin(), valid_measurements.end(),
                  [](const MeasurementWithConfidence &a, const MeasurementWithConfidence &b) {
                      return a.first > b.first;
                  });

        const size_t N = std::min(valid_measurements.size(), frame_ids.size());
        if (N == 0) continue;

        std::vector<RobotDescription> assigned =
            assign_frame_ids_to_measurements(valid_measurements, frame_ids);
        for (auto &desc : assigned) filter_measurements.push_back(std::move(desc));
    }
    return filter_measurements;
}

std::vector<RobotFrontBackSimpleFilter::MeasurementWithConfidence>
RobotFrontBackSimpleFilter::build_valid_measurements(
    const Eigen::Matrix4d &tf_fieldcenter_from_camera, Label label,
    const std::vector<std::pair<FrontBackAssignment, double>> &assignments_with_conf) {
    std::vector<MeasurementWithConfidence> valid_measurements;
    std::string label_str = std::string(magic_enum::enum_name(label));

    for (const auto &[assignment, confidence] : assignments_with_conf) {
        Eigen::Vector3d front_keypoint_in_field = FrontBackKeypointConverter::transform_point(
            tf_fieldcenter_from_camera, assignment.front);
        Eigen::Vector3d back_keypoint_in_field = FrontBackKeypointConverter::transform_point(
            tf_fieldcenter_from_camera, assignment.back);
        double length = (front_keypoint_in_field - back_keypoint_in_field).norm();

        Transform tf_field_from_robot;
        bool pose_found = keypoint_converter_->get_pose_from_points(
            front_keypoint_in_field, back_keypoint_in_field, tf_field_from_robot);
        diagnostics_logger_->debug(label_str, {{"pose_found", pose_found ? "true" : "false"}});

        if (!pose_found) continue;

        Pose pose = matrix_to_pose(tf_field_from_robot.tf);
        RobotDescription desc{FrameId::EMPTY,
                              label,
                              pose,
                              Size{length, length, 0.1},
                              {vector_to_position(front_keypoint_in_field),
                               vector_to_position(back_keypoint_in_field)},
                              Velocity2D{}};
        valid_measurements.push_back({confidence, std::move(desc)});
    }
    return valid_measurements;
}

std::vector<RobotDescription> RobotFrontBackSimpleFilter::assign_frame_ids_to_measurements(
    std::vector<MeasurementWithConfidence> &valid_measurements,
    const std::vector<FrameId> &frame_ids) {
    std::vector<RobotDescription> result;
    const size_t N = std::min(valid_measurements.size(), frame_ids.size());
    if (N == 0) return result;

    std::vector<bool> meas_assigned(valid_measurements.size(), false);
    std::vector<bool> frame_id_assigned(frame_ids.size(), false);
    constexpr double no_previous = std::numeric_limits<double>::infinity();

    for (size_t n = 0; n < N; n++) {
        double best_dist = no_previous;
        size_t best_meas = 0;
        size_t best_fid = 0;
        bool best_has_previous = false;

        for (size_t m = 0; m < valid_measurements.size(); m++) {
            if (meas_assigned[m]) continue;
            const Position &pos = valid_measurements[m].second.pose.position;
            for (size_t f = 0; f < frame_ids.size(); f++) {
                if (frame_id_assigned[f]) continue;
                FrameId fid = frame_ids[f];
                auto it = last_position_per_frame_id_.find(fid);
                double d = no_previous;
                bool has_previous = false;
                if (it != last_position_per_frame_id_.end()) {
                    d = position_distance(pos, it->second);
                    has_previous = true;
                }
                if (!best_has_previous && has_previous) {
                    best_dist = d;
                    best_meas = m;
                    best_fid = f;
                    best_has_previous = true;
                } else if (best_has_previous == has_previous && d < best_dist) {
                    best_dist = d;
                    best_meas = m;
                    best_fid = f;
                    best_has_previous = has_previous;
                }
            }
        }

        if (best_dist == no_previous && !best_has_previous) {
            for (size_t m = 0; m < valid_measurements.size(); m++) {
                if (!meas_assigned[m]) {
                    best_meas = m;
                    break;
                }
            }
            for (size_t f = 0; f < frame_ids.size(); f++) {
                if (!frame_id_assigned[f]) {
                    best_fid = f;
                    break;
                }
            }
        }

        meas_assigned[best_meas] = true;
        frame_id_assigned[best_fid] = true;
        FrameId assigned_fid = frame_ids[best_fid];
        valid_measurements[best_meas].second.frame_id = assigned_fid;
        last_position_per_frame_id_[assigned_fid] =
            valid_measurements[best_meas].second.pose.position;
        result.push_back(std::move(valid_measurements[best_meas].second));
    }
    return result;
}

std::vector<RobotDescription> RobotFrontBackSimpleFilter::update_filter(
    std::vector<RobotDescription> inputs, [[maybe_unused]] CommandFeedback command_feedback) {
    return inputs;
}

void RobotFrontBackSimpleFilter::estimate_velocities(std::vector<RobotDescription> &descriptions,
                                                     double timestamp,
                                                     const CommandFeedback &command_feedback) {
    for (auto &desc : descriptions) {
        if (desc.frame_id == FrameId::EMPTY) continue;

        Pose2D current = pose_to_pose2d(desc.pose);

        // Check if command feedback is available for this robot (i.e. it's "ours")
        auto cmd_it = command_feedback.commands.find(desc.frame_id);
        auto state_it = velocity_state_per_frame_id_.find(desc.frame_id);

        if (cmd_it != command_feedback.commands.end()) {
            // Our robot: use commanded velocity rotated from body frame to field frame.
            // The command represents what we told the motors to do, which is a reliable
            // velocity estimate that doesn't suffer from measurement noise.
            const VelocityCommand &cmd = cmd_it->second;
            double cos_yaw = std::cos(current.yaw);
            double sin_yaw = std::sin(current.yaw);

            desc.velocity.vx = cmd.linear_x * cos_yaw - cmd.linear_y * sin_yaw;
            desc.velocity.vy = cmd.linear_x * sin_yaw + cmd.linear_y * cos_yaw;
            desc.velocity.omega = cmd.angular_z;
        } else if (state_it != velocity_state_per_frame_id_.end()) {
            // Opponent robot: differentiate position over time with EMA smoothing.
            const RobotVelocityState &prev = state_it->second;
            double dt = timestamp - prev.timestamp;

            if (dt > 0.001 && dt < 1.0) {
                double raw_vx = (current.x - prev.pose.x) / dt;
                double raw_vy = (current.y - prev.pose.y) / dt;
                // Wrap-safe yaw difference
                double delta_yaw = std::atan2(std::sin(current.yaw - prev.pose.yaw),
                                              std::cos(current.yaw - prev.pose.yaw));
                double raw_omega = delta_yaw / dt;

                double alpha = velocity_ema_alpha_;
                desc.velocity.vx = alpha * raw_vx + (1.0 - alpha) * prev.smoothed_velocity.vx;
                desc.velocity.vy = alpha * raw_vy + (1.0 - alpha) * prev.smoothed_velocity.vy;
                desc.velocity.omega =
                    alpha * raw_omega + (1.0 - alpha) * prev.smoothed_velocity.omega;
            } else {
                // dt too small or too large -- carry forward previous estimate
                desc.velocity = prev.smoothed_velocity;
            }
        }
        // else: first observation for this FrameId, velocity stays at default (0, 0, 0)

        // Store state for next frame
        velocity_state_per_frame_id_[desc.frame_id] = {timestamp, current, desc.velocity};
    }
}

std::vector<FrameId> RobotFrontBackSimpleFilter::get_frame_ids_for_label(Label label) const {
    if (label_to_frame_ids_.count(label) != 0 && !label_to_frame_ids_.at(label).empty())
        return label_to_frame_ids_.at(label);
    return {get_default_frame_id_for_label(label)};
}

FrameId RobotFrontBackSimpleFilter::get_default_frame_id_for_label(const Label label) const {
    if (label_to_frame_ids_.count(label) == 0) return default_frame_id_;
    const std::vector<FrameId> &v = label_to_frame_ids_.at(label);
    if (v.empty()) return default_frame_id_;
    return v[0];
}

}  // namespace auto_battlebot
