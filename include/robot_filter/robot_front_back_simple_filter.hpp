#pragma once

#include <map>
#include <unordered_map>
#include <vector>
#include "robot_filter/config.hpp"
#include "robot_filter/robot_filter_interface.hpp"
#include "robot_filter/front_back_keypoint_converter.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"
#include "diagnostics_logger/diagnostics_module_logger.hpp"
#include "transform_utils.hpp"
#include "data_structures/command_feedback.hpp"
#include "data_structures/pose.hpp"

namespace auto_battlebot
{
    class RobotFrontBackSimpleFilter : public RobotFilterInterface
    {
    public:
        RobotFrontBackSimpleFilter(RobotFrontBackSimpleFilterConfiguration &config);

        bool initialize(const std::vector<RobotConfig> &robots) override;
        RobotDescriptionsStamped update(KeypointsStamped keypoints, FieldDescription field, CameraInfo camera_info, CommandFeedback command_feedback) override;

    private:
        std::unordered_map<Label, RobotConfig> robot_configs_;
        std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;
        std::unique_ptr<FrontBackKeypointConverter> keypoint_converter_;
        std::map<Label, std::vector<FrameId>> label_to_frame_ids_;
        FrameId default_frame_id_;
        /** Last known position per FrameId for distance-based assignment when multiple of same label. */
        std::map<FrameId, Position> last_position_per_frame_id_;

        std::vector<RobotDescription> convert_keypoints_to_measurements(KeypointsStamped keypoints, FieldDescription field, CameraInfo camera_info);
        std::vector<RobotDescription> update_filter(std::vector<RobotDescription> inputs, CommandFeedback command_feedback);

        using MeasurementWithConfidence = std::pair<double, RobotDescription>;
        std::vector<MeasurementWithConfidence> build_valid_measurements(
            const Eigen::Matrix4d &tf_fieldcenter_from_camera,
            Label label,
            const std::vector<std::pair<FrontBackAssignment, double>> &assignments_with_conf);
        std::vector<RobotDescription> assign_frame_ids_to_measurements(
            std::vector<MeasurementWithConfidence> &valid_measurements,
            const std::vector<FrameId> &frame_ids);

        /** FrameIds to assign for this label (from config or single default). */
        std::vector<FrameId> get_frame_ids_for_label(Label label) const;
        /** Used when label has no mapping or single default FrameId. */
        FrameId get_default_frame_id_for_label(const Label label) const;
    };

} // namespace auto_battlebot
