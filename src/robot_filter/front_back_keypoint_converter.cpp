#include "robot_filter/front_back_keypoint_converter.hpp"

#include <spdlog/spdlog.h>

#include <cmath>
#include <magic_enum.hpp>

#include "diagnostics_logger/diagnostics_logger.hpp"
#include "transform_utils.hpp"

namespace auto_battlebot {
FrontBackKeypointConverter::FrontBackKeypointConverter(
    const FrontBackKeypointConverterConfig &config)
    : config_(config) {
    diagnostics_logger_ = DiagnosticsLogger::get_logger("front_back_keypoint_converter");
}

std::map<Label, std::vector<std::pair<FrontBackAssignment, double>>>
FrontBackKeypointConverter::convert(const ModelResultStamped &model_results,
                                    const FieldDescription &field, const CameraInfo &camera_info) {
    Eigen::Vector3d plane_center = Eigen::Vector3d::Zero();
    Eigen::Vector3d plane_normal = Eigen::Vector3d::UnitZ();
    if (!transform_to_plane_center_normal(field.tf_camera_from_fieldcenter, plane_center,
                                          plane_normal)) {
        spdlog::error("Failed to construct projection plane from field transform");
        return {};
    }

    using LabelAndDetectionIndex = std::pair<Label, int>;
    std::map<LabelAndDetectionIndex, FrontBackAssignment> group_assignments;
    std::map<LabelAndDetectionIndex, double> group_confidence;
    std::map<LabelAndDetectionIndex, bool> group_has_front;
    std::map<LabelAndDetectionIndex, bool> group_has_back;

    for (const Keypoint &keypoint : model_results.keypoints) {
        Eigen::Vector3d projected_keypoint;
        // Prefer the height KeypointHeightGate measured from depth; the configured constant is
        // the fallback for model_results it could not measure.
        const double height = keypoint.height_above_plane_m.value_or(
            config_.keypoint_heights.height_for(keypoint.label));
        if (!project_keypoint_onto_plane(keypoint, plane_center, plane_normal, camera_info, height,
                                         projected_keypoint)) {
            continue;
        }
        std::string keypoint_label_str =
            std::string(magic_enum::enum_name(keypoint.keypoint_label));
        diagnostics_logger_->debug(keypoint_label_str, {{"x", projected_keypoint[0]},
                                                        {"y", projected_keypoint[1]},
                                                        {"z", projected_keypoint[2]}});

        LabelAndDetectionIndex key(keypoint.label, keypoint.detection_index);
        if (group_assignments.find(key) == group_assignments.end())
            group_assignments[key] = FrontBackAssignment{};
        group_assignments[key].reference_offset_m = keypoint.reference_offset_m;
        if (group_confidence.find(key) == group_confidence.end())
            group_confidence[key] = keypoint.confidence;

        auto front_assign_it = std::find(config_.front_keypoints.begin(),
                                         config_.front_keypoints.end(), keypoint.keypoint_label);
        if (front_assign_it != config_.front_keypoints.end()) {
            group_assignments[key].front = projected_keypoint;
            group_has_front[key] = true;
            diagnostics_logger_->debug(keypoint_label_str, {{"type", "front"}});
            continue;
        }
        auto back_assign_it = std::find(config_.back_keypoints.begin(),
                                        config_.back_keypoints.end(), keypoint.keypoint_label);
        if (back_assign_it != config_.back_keypoints.end()) {
            group_assignments[key].back = projected_keypoint;
            group_has_back[key] = true;
            diagnostics_logger_->debug(keypoint_label_str, {{"type", "back"}});
            continue;
        }
        spdlog::error("Failed to assign keypoint with label: {}",
                      std::string(magic_enum::enum_name(keypoint.keypoint_label)));
    }

    // Build result: map Label -> vector of (assignment, confidence), only include groups with valid
    // pose (front and back set)
    std::map<Label, std::vector<std::pair<FrontBackAssignment, double>>> result;
    for (const auto &[group_key, assignment] : group_assignments) {
        if (!group_has_front[group_key] || !group_has_back[group_key]) {
            continue;
        }
        const Label label = group_key.first;
        double conf = group_confidence[group_key];
        result[label].push_back({assignment, conf});
    }
    return result;
}

bool FrontBackKeypointConverter::get_pose_from_points(const Eigen::Vector3d &front_point,
                                                      const Eigen::Vector3d &back_point,
                                                      double reference_offset_m,
                                                      Transform &out_transform) {
    constexpr double EPSILON = 1e-6;
    Eigen::Vector3d origin_vec(1.0, 0.0, 0.0);
    Eigen::Vector3d offset_vector = front_point - back_point;
    double magnitude = offset_vector.norm();
    if (magnitude <= EPSILON) {
        return false;
    }
    Eigen::Vector3d direction = offset_vector / magnitude;
    Eigen::Vector3d center = 0.5 * (front_point + back_point) + reference_offset_m * direction;
    // Compute rotation matrix that aligns origin_vec to direction
#ifndef __clang__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#endif
    Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(origin_vec, direction);
#ifndef __clang__
#pragma GCC diagnostic pop
#endif
    Eigen::Matrix3d rot = q.toRotationMatrix();
    Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();
    tf.block<3, 3>(0, 0) = rot;
    tf.block<3, 1>(0, 3) = center;
    out_transform.tf = tf;
    return true;
}

}  // namespace auto_battlebot
