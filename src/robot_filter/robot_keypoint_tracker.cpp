#include "robot_filter/robot_keypoint_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <tuple>

#include "transform_utils.hpp"

namespace auto_battlebot {
namespace {
Group infer_group(Label label) {
    if (label == Label::HOUSE_BOT) return Group::NEUTRAL;
    if (label == Label::OPPONENT) return Group::THEIRS;
    return Group::OURS;
}
}  // namespace

RobotKeypointTracker::RobotKeypointTracker(const RobotKeypointTrackerConfig &config)
    : config_(config) {}

void RobotKeypointTracker::reset() { tracked_.clear(); }

std::vector<RobotDescription> RobotKeypointTracker::detect(const KeypointsStamped &robot_blob_keypoints,
                                                           const FieldDescription &field,
                                                           const CameraInfo &camera_info,
                                                           double timestamp) {
    auto candidates = extract_candidates(robot_blob_keypoints, field, camera_info);
    auto persistent = update_tracking(candidates, timestamp);
    return to_descriptions(persistent);
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
        std::sort(sorted.begin(), sorted.end(),
                  [](const Keypoint &lhs, const Keypoint &rhs) { return lhs.confidence > rhs.confidence; });
        const Keypoint &kp_a = sorted[0];
        const Keypoint &kp_b = sorted[1];

        Eigen::Vector3d ray_a;
        Eigen::Vector3d ray_b;
        if (!pixel_to_camera_ray(camera_info, kp_a.x, kp_a.y, ray_a) ||
            !pixel_to_camera_ray(camera_info, kp_b.x, kp_b.y, ray_b)) {
            continue;
        }

        Eigen::Vector3d point_a;
        Eigen::Vector3d point_b;
        if (!intersect_camera_ray_with_plane(ray_a, plane_center, plane_normal, point_a) ||
            !intersect_camera_ray_with_plane(ray_b, plane_center, plane_normal, point_b)) {
            continue;
        }

        const Eigen::Vector3d center = 0.5 * (point_a + point_b);
        const double length = (point_a - point_b).norm();
        const double confidence = 0.5 * (kp_a.confidence + kp_b.confidence);
        if (length < config_.min_length_meters || length > config_.max_length_meters) continue;
        if (confidence < config_.min_confidence) continue;

        RobotKeypointCandidate candidate;
        candidate.label = group_key.first;
        candidate.group = infer_group(group_key.first);
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

std::vector<RobotKeypointCandidate> RobotKeypointTracker::update_tracking(
    const std::vector<RobotKeypointCandidate> &candidates, double timestamp) {
    std::vector<bool> matched(candidates.size(), false);

    for (auto &track : tracked_) {
        double best_distance = config_.match_distance_meters;
        int best_idx = -1;
        for (size_t i = 0; i < candidates.size(); ++i) {
            if (matched[i]) continue;
            if (candidates[i].label != track.candidate.label) continue;
            const double distance = (candidates[i].center - track.candidate.center).norm();
            if (distance < best_distance) {
                best_distance = distance;
                best_idx = static_cast<int>(i);
            }
        }

        if (best_idx >= 0) {
            track.candidate = candidates[static_cast<size_t>(best_idx)];
            track.consecutive_frames++;
            track.last_seen_timestamp = timestamp;
            matched[static_cast<size_t>(best_idx)] = true;
        }
    }

    for (size_t i = 0; i < candidates.size(); ++i) {
        if (matched[i]) continue;
        tracked_.push_back({candidates[i], 1, timestamp});
    }

    tracked_.erase(std::remove_if(tracked_.begin(), tracked_.end(),
                                  [this, timestamp](const TrackedCandidate &track) {
                                      return (timestamp - track.last_seen_timestamp) >
                                             config_.tracked_timeout_seconds;
                                  }),
                   tracked_.end());

    std::vector<RobotKeypointCandidate> persistent;
    for (const auto &track : tracked_) {
        if (track.consecutive_frames >= config_.persistence_frames_required) {
            persistent.push_back(track.candidate);
        }
    }
    return persistent;
}

std::vector<RobotDescription> RobotKeypointTracker::to_descriptions(
    const std::vector<RobotKeypointCandidate> &candidates) {
    std::vector<RobotDescription> descriptions;
    descriptions.reserve(candidates.size());
    for (const auto &candidate : candidates) {
        Position pos{candidate.center.x(), candidate.center.y(), candidate.center.z()};
        Pose pose{pos, Rotation{1.0, 0.0, 0.0, 0.0}};
        descriptions.push_back({FrameId::EMPTY,
                                candidate.label,
                                candidate.group,
                                pose,
                                Size{candidate.size_meters, candidate.size_meters, 0.1},
                                {},
                                Velocity2D{},
                                false});
    }
    return descriptions;
}
}  // namespace auto_battlebot

