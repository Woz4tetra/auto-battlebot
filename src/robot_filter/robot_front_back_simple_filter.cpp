#include "robot_filter/robot_front_back_simple_filter.hpp"

#include <algorithm>
#include <cmath>

#include <magic_enum.hpp>

#include "data_structures/robot.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"
#include "enums/frame_id.hpp"
#include "robot_filter/label_group_utils.hpp"
#include "transform_utils.hpp"

namespace {
double position_distance(const auto_battlebot::Position &a, const auto_battlebot::Position &b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool blob_is_overwritten_by_keypoint(
    const auto_battlebot::RobotDescription &blob_measurement,
    const std::vector<auto_battlebot::RobotDescription> &keypoint_measurements,
    double min_overwrite_radius_meters, double overwrite_size_scale) {
    return std::any_of(
        keypoint_measurements.begin(), keypoint_measurements.end(),
        [&blob_measurement, min_overwrite_radius_meters,
         overwrite_size_scale](const auto_battlebot::RobotDescription &keypoint_measurement) {
            // Keypoint detections are more semantically reliable, so suppress nearby blob detections.
            const double overwrite_radius = std::max(
                min_overwrite_radius_meters,
                overwrite_size_scale * (blob_measurement.size.x + keypoint_measurement.size.x));
            const double dist = position_distance(blob_measurement.pose.position,
                                                  keypoint_measurement.pose.position);
            return dist <= overwrite_radius;
        });
}
}  // namespace

namespace auto_battlebot {
RobotFrontBackSimpleFilter::RobotFrontBackSimpleFilter(
    RobotFrontBackSimpleFilterConfiguration &config)
    : label_to_frame_ids_(config.label_to_frame_ids),
      default_frame_id_(config.default_frame_id),
      velocity_ema_alpha_(config.velocity_ema_alpha),
      max_jump_distance_(config.max_jump_distance),
      max_consecutive_jump_rejects_(config.max_consecutive_jump_rejects),
      blob_overwrite_min_distance_meters_(config.blob_overwrite_min_distance_meters),
      blob_overwrite_size_scale_(config.blob_overwrite_size_scale),
      robot_keypoint_tracker_(config.robot_keypoint_tracker_config),
      frame_id_assigner_(config.max_jump_distance, config.max_consecutive_jump_rejects),
      temporal_motion_filter_(config.velocity_ema_alpha) {
    diagnostics_logger_ = DiagnosticsLogger::get_logger("robot_front_back_simple_filter");

    FrontBackKeypointConverterConfig converter_config;
    converter_config.front_keypoints = config.front_keypoints;
    converter_config.back_keypoints = config.back_keypoints;
    keypoint_converter_ = std::make_unique<FrontBackKeypointConverter>(converter_config);
}

bool RobotFrontBackSimpleFilter::initialize(const std::vector<RobotConfig> &robots) {
    robot_configs_.clear();
    robot_keypoint_tracker_.set_robot_configs(robot_configs_);
    frame_id_assigner_.reset();
    temporal_motion_filter_.reset();
    robot_keypoint_tracker_.reset();
    for (const auto &robot : robots) {
        robot_configs_[robot.label] = robot;
    }
    robot_keypoint_tracker_.set_robot_configs(robot_configs_);
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
                                                            KeypointsStamped robot_blob_keypoints,
                                                            CommandFeedback command_feedback) {
    RobotDescriptionsStamped result;
    result.header.frame_id = FrameId::FIELD;
    result.header.stamp = keypoints.header.stamp;
    const Eigen::Matrix4d tf_fieldcenter_from_camera = field.tf_camera_from_fieldcenter.tf.inverse();

    std::vector<RobotDescription> filter_measurements =
        convert_keypoints_to_measurements(keypoints, field, camera_info, tf_fieldcenter_from_camera);
    const std::vector<RobotDescription> keypoint_measurements = filter_measurements;
    diagnostics_logger_->debug({{"num_keypoint_measurements", (int)filter_measurements.size()}});

    if (!robot_blob_keypoints.keypoints.empty() && field.child_frame_id != FrameId::EMPTY) {
        auto blob_measurements = robot_keypoint_tracker_.detect(
            robot_blob_keypoints, field, camera_info, keypoints.header.stamp);

        for (auto &measurement : blob_measurements) {
            const Eigen::Vector3d blob_center_camera(measurement.pose.position.x,
                                                     measurement.pose.position.y,
                                                     measurement.pose.position.z);
            const Eigen::Vector3d blob_center_field =
                transform_point(tf_fieldcenter_from_camera, blob_center_camera);
            measurement.pose.position = vector_to_position(blob_center_field);
        }

        blob_measurements.erase(
            std::remove_if(
                blob_measurements.begin(), blob_measurements.end(),
                [this, &keypoint_measurements](const RobotDescription &blob_measurement) {
                    return blob_is_overwritten_by_keypoint(
                        blob_measurement, keypoint_measurements,
                        blob_overwrite_min_distance_meters_, blob_overwrite_size_scale_);
                }),
            blob_measurements.end());

        diagnostics_logger_->debug({{"num_blob_measurements", (int)blob_measurements.size()}});

        std::map<Label, std::vector<MeasurementWithConfidence>> grouped_blob_measurements;
        for (auto &measurement : blob_measurements) {
            grouped_blob_measurements[measurement.label].push_back({0.0, std::move(measurement)});
        }

        for (auto &[label, measurements] : grouped_blob_measurements) {
            std::vector<FrameId> all_fids = get_frame_ids_for_label(label);
            std::vector<FrameId> available_fids;
            for (const auto &fid : all_fids) {
                bool already_used =
                    std::any_of(filter_measurements.begin(), filter_measurements.end(),
                                [fid](const RobotDescription &m) { return m.frame_id == fid; });
                if (!already_used) available_fids.push_back(fid);
            }
            if (available_fids.empty()) continue;
            auto assigned = frame_id_assigner_.assign(measurements, available_fids, diagnostics_logger_);
            for (auto &a : assigned) {
                // Orientation from blob rectangles is intentionally non-semantic. Keep identity.
                a.pose.rotation = Rotation{1.0, 0.0, 0.0, 0.0};
                filter_measurements.push_back(std::move(a));
            }
        }
    }

    result.descriptions = temporal_motion_filter_.update_with_prediction(
        filter_measurements, command_feedback, result.header.stamp, frame_id_assigner_);
    temporal_motion_filter_.estimate_velocities(result.descriptions, result.header.stamp,
                                                command_feedback);

    return result;
}

std::vector<RobotDescription> RobotFrontBackSimpleFilter::convert_keypoints_to_measurements(
    const KeypointsStamped &keypoints, const FieldDescription &field, const CameraInfo &camera_info,
    const Eigen::Matrix4d &tf_fieldcenter_from_camera) {
    std::vector<RobotDescription> filter_measurements;
    auto front_back_mapping = keypoint_converter_->convert(keypoints, field, camera_info);

    // Clear stale last_position for single-FrameId labels that have no detections this frame,
    // so when they reappear we assign without bias from an old position.
    for (const auto &[label, frame_ids] : label_to_frame_ids_) {
        if (frame_ids.size() != 1) continue;
        if (front_back_mapping.count(label) == 0) frame_id_assigner_.clear_last_position(frame_ids[0]);
    }

    for (const auto &[label, assignments_with_conf] : front_back_mapping) {
        std::vector<MeasurementWithConfidence> valid_measurements =
            build_valid_measurements(tf_fieldcenter_from_camera, label, assignments_with_conf);
        std::vector<FrameId> frame_ids = get_frame_ids_for_label(label);

        if (valid_measurements.empty()) {
            if (frame_ids.size() == 1) frame_id_assigner_.clear_last_position(frame_ids[0]);
            continue;
        }

        std::sort(valid_measurements.begin(), valid_measurements.end(),
                  [](const MeasurementWithConfidence &a, const MeasurementWithConfidence &b) {
                      return a.first > b.first;
                  });

        std::vector<RobotDescription> assigned =
            frame_id_assigner_.assign(valid_measurements, frame_ids, diagnostics_logger_);
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
    const Group group = group_for_label(label, robot_configs_);

    for (const auto &[assignment, confidence] : assignments_with_conf) {
        Eigen::Vector3d front_keypoint_in_field =
            transform_point(tf_fieldcenter_from_camera, assignment.front);
        Eigen::Vector3d back_keypoint_in_field =
            transform_point(tf_fieldcenter_from_camera, assignment.back);
        double length = (front_keypoint_in_field - back_keypoint_in_field).norm();

        Transform tf_field_from_robot;
        bool pose_found = keypoint_converter_->get_pose_from_points(
            front_keypoint_in_field, back_keypoint_in_field, tf_field_from_robot);
        diagnostics_logger_->debug(label_str, {{"pose_found", pose_found ? "true" : "false"}});

        if (!pose_found) continue;

        Pose pose = matrix_to_pose(tf_field_from_robot.tf);
        RobotDescription desc{FrameId::EMPTY,
                              label,
                              group,
                              pose,
                              Size{length, length, 0.1},
                              {vector_to_position(front_keypoint_in_field),
                               vector_to_position(back_keypoint_in_field)},
                              Velocity2D{},
                              false};
        valid_measurements.push_back({confidence, std::move(desc)});
    }
    return valid_measurements;
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
