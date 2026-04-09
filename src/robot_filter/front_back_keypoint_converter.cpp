#include "robot_filter/front_back_keypoint_converter.hpp"

#include <spdlog/spdlog.h>

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
FrontBackKeypointConverter::convert(const KeypointsStamped &keypoints,
                                    const FieldDescription &field, const CameraInfo &camera_info) {
    Eigen::Vector3d plane_center = Eigen::Vector3d::Zero();
    Eigen::Vector3d plane_normal = Eigen::Vector3d::UnitZ();
    if (!transform_to_plane_center_normal(field.tf_camera_from_fieldcenter, plane_center,
                                          plane_normal)) {
        spdlog::error("Failed to construct projection plane from field transform");
        return {};
    }

    // Group keypoints by (label, detection_index), each group becomes one FrontBackAssignment with
    // confidence
    using GroupKey = std::pair<Label, int>;
    std::map<GroupKey, FrontBackAssignment> group_assignments;
    std::map<GroupKey, double> group_confidence;

    for (const Keypoint &keypoint : keypoints.keypoints) {
        Eigen::Vector3d projected_keypoint =
            project_keypoint_onto_plane(keypoint, plane_center, plane_normal, camera_info);
        std::string keypoint_label_str =
            std::string(magic_enum::enum_name(keypoint.keypoint_label));
        diagnostics_logger_->debug(keypoint_label_str, {{"x", projected_keypoint[0]},
                                                        {"y", projected_keypoint[1]},
                                                        {"z", projected_keypoint[2]}});

        GroupKey key(keypoint.label, keypoint.detection_index);
        if (group_assignments.find(key) == group_assignments.end())
            group_assignments[key] = FrontBackAssignment{};
        if (group_confidence.find(key) == group_confidence.end())
            group_confidence[key] = keypoint.confidence;

        auto front_assign_it = std::find(config_.front_keypoints.begin(),
                                         config_.front_keypoints.end(), keypoint.keypoint_label);
        if (front_assign_it != config_.front_keypoints.end()) {
            group_assignments[key].front = projected_keypoint;
            diagnostics_logger_->debug(keypoint_label_str, {{"type", "front"}});
            continue;
        }
        auto back_assign_it = std::find(config_.back_keypoints.begin(),
                                        config_.back_keypoints.end(), keypoint.keypoint_label);
        if (back_assign_it != config_.back_keypoints.end()) {
            group_assignments[key].back = projected_keypoint;
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
        const Label label = group_key.first;
        double conf = group_confidence[group_key];
        result[label].push_back({assignment, conf});
    }
    return result;
}

Eigen::Vector3d FrontBackKeypointConverter::project_keypoint_onto_plane(
    const Keypoint &keypoint, const Eigen::Vector3d &plane_center,
    const Eigen::Vector3d &plane_normal, const CameraInfo &camera_info) {
    Eigen::Vector3d ray;
    if (!pixel_to_camera_ray(camera_info, keypoint.x, keypoint.y, ray)) return Eigen::Vector3d::Zero();
    Eigen::Vector3d point_on_plane;
    if (!intersect_camera_ray_with_plane(ray, plane_center, plane_normal, point_on_plane)) {
        return Eigen::Vector3d::Zero();
    }
    return point_on_plane;
}

Eigen::Vector3d FrontBackKeypointConverter::transform_point(const Eigen::Matrix4d &tf,
                                                            const Eigen::Vector3d &point) {
    Eigen::Vector4d point_homogenous(point[0], point[1], point[2], 1);
    Eigen::Vector3d transformed_point = (tf * point_homogenous).head<3>();
    return transformed_point;
}

bool FrontBackKeypointConverter::get_pose_from_points(const Eigen::Vector3d &front_point,
                                                      const Eigen::Vector3d &back_point,
                                                      Transform &out_transform) {
    constexpr double EPSILON = 1e-6;
    Eigen::Vector3d origin_vec(1.0, 0.0, 0.0);
    // Pose +X points toward front: direction from back to front
    Eigen::Vector3d offset_vector = front_point - back_point;
    double magnitude = offset_vector.norm();
    if (magnitude <= EPSILON) {
        return false;
    }
    Eigen::Vector3d direction = offset_vector / magnitude;
    // Center point between front and back
    Eigen::Vector3d center = 0.5 * (front_point + back_point);
    // Compute rotation matrix that aligns origin_vec to direction
#ifndef __clang__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#endif
    // Use Eigen's Quaternion for rotation between two vectors
    Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(origin_vec, direction);
#ifndef __clang__
#pragma GCC diagnostic pop
#endif
    Eigen::Matrix3d rot = q.toRotationMatrix();
    // Build 4x4 transform matrix
    Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();
    tf.block<3, 3>(0, 0) = rot;
    tf.block<3, 1>(0, 3) = center;
    out_transform.tf = tf;
    return true;
}

}  // namespace auto_battlebot
