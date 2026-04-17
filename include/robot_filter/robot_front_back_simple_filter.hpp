#pragma once

#include <Eigen/Dense>
#include <map>
#include <unordered_map>
#include <vector>

#include "data_structures/command_feedback.hpp"
#include "diagnostics_logger/diagnostics_module_logger.hpp"
#include "robot_filter/config.hpp"
#include "robot_filter/frame_id_assigner.hpp"
#include "robot_filter/front_back_keypoint_converter.hpp"
#include "robot_filter/robot_filter_interface.hpp"
#include "robot_filter/robot_keypoint_tracker.hpp"
#include "robot_filter/robot_temporal_motion_filter.hpp"

namespace auto_battlebot {
class RobotFrontBackSimpleFilter : public RobotFilterInterface {
   public:
    RobotFrontBackSimpleFilter(RobotFrontBackSimpleFilterConfiguration &config);

    bool initialize(const std::vector<RobotConfig> &robots) override;
    RobotDescriptionsStamped update(KeypointsStamped keypoints, FieldDescription field,
                                    CameraInfo camera_info, KeypointsStamped robot_blob_keypoints,
                                    CommandFeedback command_feedback) override;
    bool set_opponent_count(int count) override;

   private:
    std::unordered_map<Label, RobotConfig> robot_configs_;
    std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;
    std::unique_ptr<FrontBackKeypointConverter> keypoint_converter_;
    std::map<Label, std::vector<FrameId>> label_to_frame_ids_;
    FrameId default_frame_id_;
    /** Exponential moving average smoothing factor for opponent velocity estimation (0..1, higher =
     * more responsive). */
    double velocity_ema_alpha_;
    /** Reject measurements that jump further than this (meters) from last known position. */
    double max_jump_distance_;
    /** After this many consecutive rejected frames, accept the measurement (tracking reset). */
    int max_consecutive_jump_rejects_;
    /** Minimum radius (meters) for suppressing blobs near keypoint detections. */
    double blob_overwrite_min_distance_meters_;
    /** Radius scale for suppression based on blob/keypoint size estimates. */
    double blob_overwrite_size_scale_;
    RobotKeypointTracker robot_keypoint_tracker_;
    FrameIdAssigner frame_id_assigner_;
    RobotTemporalMotionFilter temporal_motion_filter_;

    std::vector<RobotDescription> convert_keypoints_to_measurements(
        const KeypointsStamped &keypoints, const FieldDescription &field, const CameraInfo &camera_info,
        const Eigen::Matrix4d &tf_fieldcenter_from_camera);

    using MeasurementWithConfidence = std::pair<double, RobotDescription>;
    std::vector<MeasurementWithConfidence> build_valid_measurements(
        const Eigen::Matrix4d &tf_fieldcenter_from_camera, Label label,
        const std::vector<std::pair<FrontBackAssignment, double>> &assignments_with_conf);

    /** FrameIds to assign for this label (from config or single default). */
    std::vector<FrameId> get_frame_ids_for_label(Label label) const;
    /** Used when label has no mapping or single default FrameId. */
    FrameId get_default_frame_id_for_label(const Label label) const;
};

}  // namespace auto_battlebot
