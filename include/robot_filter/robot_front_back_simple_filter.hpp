#pragma once

#include <map>
#include <opencv2/opencv.hpp>
#include <unordered_map>
#include <vector>

#include "data_structures/command_feedback.hpp"
#include "data_structures/pose.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"
#include "diagnostics_logger/diagnostics_module_logger.hpp"
#include "robot_filter/config.hpp"
#include "robot_filter/elevation_detector.hpp"
#include "robot_filter/front_back_keypoint_converter.hpp"
#include "robot_filter/robot_filter_interface.hpp"
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
                                    const CameraData &camera_data,
                                    CommandFeedback command_feedback) override;
    bool set_opponent_count(int count) override;

   private:
    std::unordered_map<Label, RobotConfig> robot_configs_;
    std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;
    FrontBackKeypointConverter keypoint_converter_;
    std::map<Label, std::vector<FrameId>> label_to_frame_ids_;
    FrameId default_frame_id_;
    double velocity_ema_alpha_;
    std::map<FrameId, Position> last_position_per_frame_id_;
    std::map<FrameId, RobotVelocityState> velocity_state_per_frame_id_;

    ElevationDetector elevation_detector_;

    std::vector<RobotDescription> convert_keypoints_to_measurements(KeypointsStamped keypoints,
                                                                    FieldDescription field,
                                                                    const CameraInfo &camera_info);
    std::vector<RobotDescription> update_filter(std::vector<RobotDescription> inputs,
                                                CommandFeedback command_feedback);

    void estimate_velocities(std::vector<RobotDescription> &descriptions, double timestamp,
                             const CommandFeedback &command_feedback);

    using MeasurementWithConfidence = std::pair<double, RobotDescription>;
    std::vector<MeasurementWithConfidence> build_valid_measurements(
        const Eigen::Matrix4d &tf_fieldcenter_from_camera, Label label,
        const std::vector<std::pair<FrontBackAssignment, double>> &assignments_with_conf);
    std::vector<RobotDescription> assign_frame_ids_to_measurements(
        std::vector<MeasurementWithConfidence> &valid_measurements,
        const std::vector<FrameId> &frame_ids);

    std::vector<FrameId> get_frame_ids_for_label(Label label) const;
    FrameId get_default_frame_id_for_label(const Label label) const;
};

}  // namespace auto_battlebot
