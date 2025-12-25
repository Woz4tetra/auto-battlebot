#pragma once

#include <unordered_map>
#include "robot_filter/config.hpp"
#include "robot_filter/robot_filter_interface.hpp"
#include "robot_filter/front_back_keypoint_converter.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"
#include "diagnostics_logger/diagnostics_module_logger.hpp"
#include "transform_utils.hpp"
#include "data_structures/command_feedback.hpp"

namespace auto_battlebot
{
    class RobotFrontBackKalmanFilter : public RobotFilterInterface
    {
    public:
        RobotFrontBackKalmanFilter(RobotFrontBackKalmanFilterConfiguration &config);

        bool initialize(const std::vector<RobotConfig> &robots) override;
        RobotDescriptionsStamped update(KeypointsStamped keypoints, FieldDescription field, CameraInfo camera_info, CommandFeedback command_feedback) override;

    private:
        std::unordered_map<Label, RobotConfig> robot_configs_;
        RobotFrontBackKalmanFilterConfiguration config_;
        std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;
        std::unique_ptr<FrontBackKeypointConverter> keypoint_converter_;

        std::vector<RobotDescription> convert_keypoints_to_measurements(KeypointsStamped keypoints, FieldDescription field, CameraInfo camera_info);
        std::vector<RobotDescription> update_filter(std::vector<RobotDescription> inputs, CommandFeedback command_feedback);
    };

} // namespace auto_battlebot
