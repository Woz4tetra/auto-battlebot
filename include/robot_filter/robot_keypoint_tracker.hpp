#pragma once

#include <Eigen/Dense>
#include <unordered_map>
#include <vector>

#include "data_structures/camera.hpp"
#include "data_structures/field.hpp"
#include "data_structures/keypoint.hpp"
#include "data_structures/robot.hpp"

namespace auto_battlebot {
struct RobotKeypointTrackerConfig {
    double min_length_meters = 0.05;
    double max_length_meters = 1.0;
    double min_confidence = 0.3;
    int max_candidates = 6;
};

struct RobotKeypointCandidate {
    Label label = Label::EMPTY;
    Group group = Group::THEIRS;
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    double size_meters = 0.0;
    double confidence = 0.0;
};

struct RobotKeypointDetection {
    RobotDescription description;
    double confidence = 0.0;
};

class RobotKeypointTracker {
   public:
    explicit RobotKeypointTracker(const RobotKeypointTrackerConfig &config);

    void set_robot_configs(const std::unordered_map<Label, RobotConfig> &robot_configs);
    std::vector<RobotKeypointDetection> detect_with_confidence(
        const KeypointsStamped &robot_blob_keypoints, const FieldDescription &field,
        const CameraInfo &camera_info);
    std::vector<RobotDescription> detect(const KeypointsStamped &robot_blob_keypoints,
                                         const FieldDescription &field,
                                         const CameraInfo &camera_info);

   private:
    RobotKeypointTrackerConfig config_;
    std::unordered_map<Label, RobotConfig> robot_configs_;

    std::vector<RobotKeypointCandidate> extract_candidates(
        const KeypointsStamped &robot_blob_keypoints, const FieldDescription &field,
        const CameraInfo &camera_info) const;
    static std::vector<RobotKeypointDetection> to_detections_with_confidence(
        const std::vector<RobotKeypointCandidate> &candidates);
};
}  // namespace auto_battlebot
