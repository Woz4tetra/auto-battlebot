#include "robot_filter/robot_keypoint_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <tuple>

#include "robot_filter/label_group_utils.hpp"
#include "transform_utils.hpp"

namespace auto_battlebot {
RobotKeypointTracker::RobotKeypointTracker(const RobotKeypointTrackerConfig &config)
    : config_(config) {}

void RobotKeypointTracker::set_robot_configs(
    const std::unordered_map<Label, RobotConfig> &robot_configs) {
    robot_configs_ = robot_configs;
}

std::vector<RobotDescription> RobotKeypointTracker::detect(
    const KeypointsStamped &robot_blob_keypoints, const FieldDescription &field,
    const CameraInfo &camera_info) {
    auto detections = detect_with_confidence(robot_blob_keypoints, field, camera_info);
    std::vector<RobotDescription> descriptions;
    descriptions.reserve(detections.size());
    for (auto &detection : detections) {
        descriptions.push_back(std::move(detection.description));
    }
    return descriptions;
}

std::vector<RobotKeypointDetection> RobotKeypointTracker::detect_with_confidence(
    const KeypointsStamped &robot_blob_keypoints, const FieldDescription &field,
    const CameraInfo &camera_info) {
    auto candidates = extract_candidates(robot_blob_keypoints, field, camera_info);
    return to_detections_with_confidence(candidates);
}

std::vector<RobotKeypointCandidate> RobotKeypointTracker::extract_candidates(
    const KeypointsStamped &robot_blob_keypoints, const FieldDescription &field,
    const CameraInfo &camera_info) const {
    std::vector<RobotKeypointCandidate> candidates;
    if (robot_blob_keypoints.keypoints.empty() || field.child_frame_id == FrameId::EMPTY) {
        return candidates;
    }

    Eigen::Vector3d plane_center = Eigen::Vector3d::Zero();
    Eigen::Vector3d plane_normal = Eigen::Vector3d::UnitZ();
    if (!transform_to_plane_center_normal(field.tf_camera_from_fieldcenter, plane_center,
                                          plane_normal)) {
        return candidates;
    }

    using GroupKey = std::pair<Label, int>;
    std::map<GroupKey, std::vector<Keypoint>> grouped;
    for (const auto &kp : robot_blob_keypoints.keypoints) {
        grouped[{kp.label, kp.detection_index}].push_back(kp);
    }

    for (const auto &[group_key, group] : grouped) {
        if (group.size() < 2) continue;

        std::vector<Keypoint> sorted = group;
        std::sort(sorted.begin(), sorted.end(), [](const Keypoint &lhs, const Keypoint &rhs) {
            return lhs.confidence > rhs.confidence;
        });
        const Keypoint &kp_a = sorted[0];
        const Keypoint &kp_b = sorted[1];

        Eigen::Vector3d point_a;
        Eigen::Vector3d point_b;
        if (!project_keypoint_onto_plane(kp_a, plane_center, plane_normal, camera_info, point_a) ||
            !project_keypoint_onto_plane(kp_b, plane_center, plane_normal, camera_info, point_b)) {
            continue;
        }

        const Eigen::Vector3d center = 0.5 * (point_a + point_b);
        const double length = (point_a - point_b).norm();
        const double confidence = 0.5 * (kp_a.confidence + kp_b.confidence);
        if (length < config_.min_length_meters || length > config_.max_length_meters) continue;
        if (confidence < config_.min_confidence) continue;

        RobotKeypointCandidate candidate;
        candidate.label = group_key.first;
        candidate.group = group_for_label(group_key.first, robot_configs_);
        candidate.center = center;
        candidate.size_meters = length;
        candidate.confidence = confidence;
        candidates.push_back(candidate);
    }

    std::sort(candidates.begin(), candidates.end(),
              [](const RobotKeypointCandidate &lhs, const RobotKeypointCandidate &rhs) {
                  if (lhs.confidence != rhs.confidence) return lhs.confidence > rhs.confidence;
                  if (lhs.label != rhs.label) return lhs.label < rhs.label;
                  if (lhs.center.x() != rhs.center.x()) return lhs.center.x() < rhs.center.x();
                  return lhs.center.y() < rhs.center.y();
              });
    if (static_cast<int>(candidates.size()) > config_.max_candidates) {
        candidates.resize(static_cast<size_t>(config_.max_candidates));
    }
    return candidates;
}

std::vector<RobotKeypointDetection> RobotKeypointTracker::to_detections_with_confidence(
    const std::vector<RobotKeypointCandidate> &candidates) {
    std::vector<RobotKeypointDetection> detections;
    detections.reserve(candidates.size());
    for (const auto &candidate : candidates) {
        Position pos{candidate.center.x(), candidate.center.y(), candidate.center.z()};
        Pose pose{pos, Rotation{1.0, 0.0, 0.0, 0.0}};
        RobotDescription description{FrameId::EMPTY,
                                     candidate.label,
                                     candidate.group,
                                     pose,
                                     Size{candidate.size_meters, candidate.size_meters, 0.1},
                                     {},
                                     Velocity2D{},
                                     false};
        detections.push_back({std::move(description), candidate.confidence});
    }
    return detections;
}

}  // namespace auto_battlebot
