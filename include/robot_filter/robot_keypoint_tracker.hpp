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
    int persistence_frames_required = 1;
    double match_distance_meters = 0.4;
    double tracked_timeout_seconds = 0.5;
    int max_candidates = 6;
};

struct RobotKeypointCandidate {
    Label label = Label::EMPTY;
    Group group = Group::THEIRS;
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    double size_meters = 0.0;
    double confidence = 0.0;
};

class RobotKeypointTracker {
   public:
    explicit RobotKeypointTracker(const RobotKeypointTrackerConfig &config);

    void reset();
    void set_robot_configs(const std::unordered_map<Label, RobotConfig> &robot_configs);
    std::vector<RobotDescription> detect(const KeypointsStamped &robot_blob_keypoints,
                                         const FieldDescription &field,
                                         const CameraInfo &camera_info, double timestamp);

   private:
    struct TrackedCandidate {
        RobotKeypointCandidate candidate;
        int consecutive_frames = 0;
        double last_seen_timestamp = 0.0;
    };

    RobotKeypointTrackerConfig config_;
    std::unordered_map<Label, RobotConfig> robot_configs_;
    std::vector<TrackedCandidate> tracked_;

    std::vector<RobotKeypointCandidate> extract_candidates(
        const KeypointsStamped &robot_blob_keypoints, const FieldDescription &field,
        const CameraInfo &camera_info) const;
    std::vector<RobotKeypointCandidate> update_tracking(
        const std::vector<RobotKeypointCandidate> &candidates, double timestamp);
    static std::vector<RobotDescription> to_descriptions(
        const std::vector<RobotKeypointCandidate> &candidates);
};
}  // namespace auto_battlebot
