#include "robot_filter/robot_front_back_filter.hpp"

#include <algorithm>
#include <cmath>
#include <magic_enum.hpp>

#include "data_structures/robot.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"
#include "enums/frame_id.hpp"
#include "robot_filter/label_group_utils.hpp"
#include "transform_utils.hpp"

namespace {
bool is_within_field_bounds(const auto_battlebot::Position &position,
                            const auto_battlebot::FieldDescription &field, double margin_meters) {
    const double width = field.size.size.x;
    const double height = field.size.size.y;
    if (width <= 0.0 || height <= 0.0) return true;
    const double half_x = width * 0.5 + margin_meters;
    const double half_y = height * 0.5 + margin_meters;
    return std::abs(position.x) <= half_x && std::abs(position.y) <= half_y;
}

void erase_out_of_field(std::vector<auto_battlebot::RobotDescription> &items,
                        const auto_battlebot::FieldDescription &field, double margin_meters) {
    items.erase(
        std::remove_if(items.begin(), items.end(),
                       [&field, margin_meters](const auto_battlebot::RobotDescription &item) {
                           return !is_within_field_bounds(item.pose.position, field, margin_meters);
                       }),
        items.end());
}

auto_battlebot::Group infer_group_from_frame_ids(
    const std::vector<auto_battlebot::FrameId> &frame_ids) {
    for (const auto frame_id : frame_ids) {
        switch (frame_id) {
            case auto_battlebot::FrameId::OUR_ROBOT_1:
            case auto_battlebot::FrameId::OUR_ROBOT_2:
                return auto_battlebot::Group::OURS;
            case auto_battlebot::FrameId::NEUTRAL_ROBOT_1:
            case auto_battlebot::FrameId::NEUTRAL_ROBOT_2:
                return auto_battlebot::Group::NEUTRAL;
            case auto_battlebot::FrameId::THEIR_ROBOT_1:
            case auto_battlebot::FrameId::THEIR_ROBOT_2:
            case auto_battlebot::FrameId::THEIR_ROBOT_3:
                return auto_battlebot::Group::THEIRS;
            default:
                break;
        }
    }
    return auto_battlebot::Group::THEIRS;
}

}  // namespace

namespace auto_battlebot {
RobotFrontBackFilter::RobotFrontBackFilter(const RobotFrontBackFilterConfiguration &config)
    : diagnostics_logger_(DiagnosticsLogger::get_logger("robot_front_back_filter")),
      label_to_frame_ids_(config.label_to_frame_ids),
      default_frame_id_(config.default_frame_id),
      max_jump_distance_(config.max_jump_distance),
      max_consecutive_jump_rejects_(config.max_consecutive_jump_rejects),
      size_overrides_(config.robot_size_meters_per_label),
      footprint_center_offsets_(config.footprint_center_offset_per_label),
      field_bounds_margin_meters_(config.field_bounds_margin_meters),
      frame_id_assigner_(config.max_jump_distance, config.max_consecutive_jump_rejects),
      motion_estimator_(make_motion_estimator(*config.motion_estimator)) {
    FrontBackKeypointConverterConfig converter_config;
    converter_config.front_keypoints = config.front_keypoints;
    converter_config.back_keypoints = config.back_keypoints;
    converter_config.keypoint_heights = config.keypoint_heights;
    keypoint_converter_ = std::make_unique<FrontBackKeypointConverter>(converter_config);
}

bool RobotFrontBackFilter::initialize(int opponent_count) {
    if (opponent_count < 1 || opponent_count > 3) {
        spdlog::error("Invalid opponent count: {}", opponent_count);
        return false;
    }

    static const std::vector<FrameId> opponent_frame_ids = {
        FrameId::THEIR_ROBOT_1, FrameId::THEIR_ROBOT_2, FrameId::THEIR_ROBOT_3};
    std::vector<FrameId> opponent_mapping;
    for (int i = 0; i < opponent_count; ++i) {
        opponent_mapping.push_back(opponent_frame_ids[static_cast<size_t>(i)]);
    }
    label_to_frame_ids_[Label::OPPONENT] = opponent_mapping;

    robot_configs_.clear();
    frame_id_assigner_.reset();
    motion_estimator_->reset();
    for (const auto &[label, frame_ids] : label_to_frame_ids_) {
        robot_configs_[label] = RobotConfig{label, infer_group_from_frame_ids(frame_ids)};
    }
    robot_configs_[Label::OPPONENT] = RobotConfig{Label::OPPONENT, Group::THEIRS};
    return true;
}

void RobotFrontBackFilter::predict(double now, CommandFeedback command_feedback) {
    motion_estimator_->predict(now, command_feedback);
    if (auto coasted = motion_estimator_->coast(now)) {
        state_.header.frame_id = FrameId::FIELD;
        state_.header.stamp = now + motion_estimator_->render_lead_s();
        state_.descriptions = std::move(*coasted);
        apply_known_geometry(state_.descriptions);
    }
}

void RobotFrontBackFilter::correct(ModelResultStamped keypoints, FieldDescription field,
                                   CameraInfo camera_info) {
    RobotDescriptionsStamped result;
    result.header.frame_id = FrameId::FIELD;
    result.header.stamp = keypoints.header.stamp;
    const Eigen::Matrix4d tf_fieldcenter_from_camera =
        field.tf_camera_from_fieldcenter.tf.inverse();

    auto all_measurements = convert_keypoints_to_measurements(keypoints, field, camera_info,
                                                              tf_fieldcenter_from_camera);
    erase_out_of_field(all_measurements, field, field_bounds_margin_meters_);
    const int num_keypoint_measurements = static_cast<int>(all_measurements.size());

    MotionEstimatorContext estimator_context;
    estimator_context.tf_fieldcenter_from_camera = tf_fieldcenter_from_camera;
    estimator_context.camera_info = camera_info;
    estimator_context.field_bounds_margin_meters = field_bounds_margin_meters_;
    result.descriptions =
        motion_estimator_->update(std::move(all_measurements), result.header.stamp,
                                  frame_id_assigner_, field, estimator_context);

    diagnostics_logger_->debug(
        {{"num_input_keypoints", static_cast<int>(keypoints.keypoints.size())},
         {"field_frame_valid", field.child_frame_id == FrameId::EMPTY ? "false" : "true"},
         {"num_keypoint_measurements", num_keypoint_measurements},
         {"num_measurements_after_temporal", static_cast<int>(result.descriptions.size())}});

    state_ = std::move(result);
    apply_known_geometry(state_.descriptions);
}

void RobotFrontBackFilter::apply_known_geometry(std::vector<RobotDescription> &descriptions) const {
    for (auto &description : descriptions) {
        const auto offset = footprint_center_offsets_.find(description.label);
        if (offset != footprint_center_offsets_.end()) {
            description.footprint_center_offset = offset->second;
        }
    }

    DiagnosticsData detected;
    for (auto &description : descriptions) {
        const auto found = size_overrides_.find(description.label);
        if (found == size_overrides_.end()) continue;
        const std::string label(magic_enum::enum_name(description.label));
        detected[label + "_x"] = description.size.x;
        detected[label + "_y"] = description.size.y;
        detected[label + "_z"] = description.size.z;
        description.size = found->second;
    }
    if (!detected.empty()) diagnostics_logger_->debug("detected_size", detected);
}

std::vector<RobotDescription> RobotFrontBackFilter::convert_keypoints_to_measurements(
    const ModelResultStamped &model_results, const FieldDescription &field,
    const CameraInfo &camera_info, const Eigen::Matrix4d &tf_fieldcenter_from_camera) {
    std::vector<RobotDescription> filter_measurements;
    auto front_back_mapping = keypoint_converter_->convert(model_results, field, camera_info);
    diagnostics_logger_->debug(
        {{"num_labels_with_front_back_assignments", static_cast<int>(front_back_mapping.size())}});

    const auto clear_absent_single_slot_positions = [&]() {
        int cleared = 0;
        for (const auto &[label, frame_ids] : label_to_frame_ids_) {
            if (frame_ids.size() != 1) continue;
            if (front_back_mapping.count(label) == 0) {
                frame_id_assigner_.clear_last_position(frame_ids[0]);
                ++cleared;
            }
        }
        return cleared;
    };
    const int cleared_single_label_positions = clear_absent_single_slot_positions();
    diagnostics_logger_->debug(
        {{"num_cleared_single_label_last_positions", cleared_single_label_positions}});

    for (const auto &[label, assignments_with_conf] : front_back_mapping) {
        std::vector<MeasurementWithConfidence> valid_measurements =
            build_valid_measurements(tf_fieldcenter_from_camera, label, assignments_with_conf);
        std::vector<FrameId> frame_ids = get_frame_ids_for_label(label);
        const std::string label_context =
            std::string("keypoint_assignment_") + std::string(magic_enum::enum_name(label));

        if (valid_measurements.empty()) {
            if (frame_ids.size() == 1) frame_id_assigner_.clear_last_position(frame_ids[0]);
            diagnostics_logger_->debug(
                label_context,
                {{"num_front_back_assignments", static_cast<int>(assignments_with_conf.size())},
                 {"num_valid_measurements", static_cast<int>(valid_measurements.size())},
                 {"num_frame_ids", static_cast<int>(frame_ids.size())},
                 {"num_assigned", 0},
                 {"cleared_last_position_due_to_empty", frame_ids.size() == 1 ? "true" : "false"}});
            continue;
        }

        std::sort(valid_measurements.begin(), valid_measurements.end(),
                  [](const MeasurementWithConfidence &a, const MeasurementWithConfidence &b) {
                      return a.confidence > b.confidence;
                  });

        std::vector<RobotDescription> assigned =
            frame_id_assigner_.assign(valid_measurements, frame_ids, diagnostics_logger_);
        diagnostics_logger_->debug(
            label_context,
            {{"num_front_back_assignments", static_cast<int>(assignments_with_conf.size())},
             {"num_valid_measurements", static_cast<int>(valid_measurements.size())},
             {"num_frame_ids", static_cast<int>(frame_ids.size())},
             {"num_assigned", static_cast<int>(assigned.size())},
             {"cleared_last_position_due_to_empty", "false"}});
        for (auto &desc : assigned) {
            desc.group = group_for_frame_id(desc.frame_id, desc.group);
            filter_measurements.push_back(std::move(desc));
        }
    }
    diagnostics_logger_->debug(
        {{"num_keypoint_measurements_assigned", static_cast<int>(filter_measurements.size())}});
    return filter_measurements;
}

std::vector<MeasurementWithConfidence> RobotFrontBackFilter::build_valid_measurements(
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
            front_keypoint_in_field, back_keypoint_in_field, assignment.reference_offset_m,
            tf_field_from_robot);
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
                              PoseUncertainty{}};
        valid_measurements.push_back({confidence, std::move(desc)});
    }
    return valid_measurements;
}

std::vector<FrameId> RobotFrontBackFilter::get_frame_ids_for_label(Label label) const {
    if (label_to_frame_ids_.count(label) != 0 && !label_to_frame_ids_.at(label).empty())
        return label_to_frame_ids_.at(label);
    return {get_default_frame_id_for_label(label)};
}

FrameId RobotFrontBackFilter::get_default_frame_id_for_label(const Label label) const {
    if (label_to_frame_ids_.count(label) == 0) return default_frame_id_;
    const std::vector<FrameId> &v = label_to_frame_ids_.at(label);
    if (v.empty()) return default_frame_id_;
    return v[0];
}

}  // namespace auto_battlebot
