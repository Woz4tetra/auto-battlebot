#pragma once

#include <map>
#include <unordered_map>
#include <vector>

#include "data_structures/command_feedback.hpp"
#include "data_structures/pose.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"
#include "diagnostics_logger/diagnostics_module_logger.hpp"
#include "robot_filter/config.hpp"
#include "robot_filter/front_back_keypoint_converter.hpp"
#include "robot_filter/robot_filter_interface.hpp"
#include "robot_filter/robot_keypoint_tracker.hpp"
#include "transform_utils.hpp"

namespace auto_battlebot {
/** Per-robot state used to estimate velocity across frames. */
struct RobotVelocityState {
    double timestamp = 0.0;
    Pose2D pose;
    Velocity2D smoothed_velocity;
};

class RobotFrontBackSimpleFilter : public RobotFilterInterface {
   public:
    RobotFrontBackSimpleFilter(RobotFrontBackSimpleFilterConfiguration &config);

    bool initialize(const std::vector<RobotConfig> &robots) override;
    RobotDescriptionsStamped update(KeypointsStamped keypoints, FieldDescription field,
                                    CameraInfo camera_info,
                                    KeypointsStamped robot_blob_keypoints,
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
    RobotKeypointTracker robot_keypoint_tracker_;
    /** Last known position per FrameId for distance-based assignment when multiple of same label.
     */
    std::map<FrameId, Position> last_position_per_frame_id_;
    /** Velocity estimation state per FrameId. */
    std::map<FrameId, RobotVelocityState> velocity_state_per_frame_id_;
    /** Consecutive jump-rejection count per FrameId. */
    std::map<FrameId, int> jump_reject_count_per_frame_id_;
    /** Last emitted description per FrameId to persist estimates across dropped detections. */
    std::map<FrameId, RobotDescription> last_description_per_frame_id_;

    std::vector<RobotDescription> convert_keypoints_to_measurements(KeypointsStamped keypoints,
                                                                    FieldDescription field,
                                                                    CameraInfo camera_info);
    std::vector<RobotDescription> update_filter(std::vector<RobotDescription> inputs,
                                                CommandFeedback command_feedback, double timestamp);

    /**
     * Estimate velocity for each robot description.
     * Our robots (present in command_feedback): use commanded velocity rotated to field frame.
     * Opponent robots: differentiate position over time with EMA smoothing.
     */
    void estimate_velocities(std::vector<RobotDescription> &descriptions, double timestamp,
                             const CommandFeedback &command_feedback);

    using MeasurementWithConfidence = std::pair<double, RobotDescription>;
    std::vector<MeasurementWithConfidence> build_valid_measurements(
        const Eigen::Matrix4d &tf_fieldcenter_from_camera, Label label,
        const std::vector<std::pair<FrontBackAssignment, double>> &assignments_with_conf);
    std::vector<RobotDescription> assign_frame_ids_to_measurements(
        std::vector<MeasurementWithConfidence> &valid_measurements,
        const std::vector<FrameId> &frame_ids);

    /** FrameIds to assign for this label (from config or single default). */
    std::vector<FrameId> get_frame_ids_for_label(Label label) const;
    /** Used when label has no mapping or single default FrameId. */
    FrameId get_default_frame_id_for_label(const Label label) const;
};

}  // namespace auto_battlebot
