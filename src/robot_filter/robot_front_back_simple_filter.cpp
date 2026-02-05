#include "robot_filter/robot_front_back_simple_filter.hpp"

namespace auto_battlebot
{
    RobotFrontBackSimpleFilter::RobotFrontBackSimpleFilter(RobotFrontBackSimpleFilterConfiguration &config)
        : label_to_frame_id_(config.label_to_frame_id),
          default_frame_id_(config.default_frame_id)
    {
        diagnostics_logger_ = DiagnosticsLogger::get_logger("robot_front_back_simple_filter");

        FrontBackKeypointConverterConfig converter_config;
        converter_config.front_keypoints = config.front_keypoints;
        converter_config.back_keypoints = config.back_keypoints;
        keypoint_converter_ = std::make_unique<FrontBackKeypointConverter>(converter_config);
    }

    bool RobotFrontBackSimpleFilter::initialize(const std::vector<RobotConfig> &robots)
    {
        robot_configs_.clear();
        for (const auto &robot : robots)
        {
            robot_configs_[robot.label] = robot;
        }
        return true;
    }

    RobotDescriptionsStamped RobotFrontBackSimpleFilter::update(KeypointsStamped keypoints, FieldDescription field, CameraInfo camera_info, CommandFeedback command_feedback)
    {
        RobotDescriptionsStamped result;
        result.header.frame_id = FrameId::FIELD;
        result.header.stamp = keypoints.header.stamp;

        std::vector<RobotDescription> filter_measurements = convert_keypoints_to_measurements(keypoints, field, camera_info);
        diagnostics_logger_->debug({{"num_measurements", (int)filter_measurements.size()}});

        result.descriptions = update_filter(filter_measurements, command_feedback);

        return result;
    }

    std::vector<RobotDescription> RobotFrontBackSimpleFilter::convert_keypoints_to_measurements(KeypointsStamped keypoints, FieldDescription field, CameraInfo camera_info)
    {
        Eigen::Matrix4d tf_fieldcenter_from_camera = field.tf_camera_from_fieldcenter.tf.inverse();

        std::vector<RobotDescription> filter_measurements;
        std::map<Label, FrontBackAssignment> front_back_mapping = keypoint_converter_->convert(keypoints, field, camera_info);
        for (const auto &[label, assignment] : front_back_mapping)
        {
            Eigen::Vector3d front_keypoint_in_field = FrontBackKeypointConverter::transform_point(tf_fieldcenter_from_camera, assignment.front);
            Eigen::Vector3d back_keypoint_in_field = FrontBackKeypointConverter::transform_point(tf_fieldcenter_from_camera, assignment.back);
            double length = (front_keypoint_in_field - back_keypoint_in_field).norm();

            Transform tf_field_from_robot;
            bool pose_found = keypoint_converter_->get_pose_from_points(front_keypoint_in_field, back_keypoint_in_field, tf_field_from_robot);

            std::string label_str = std::string(magic_enum::enum_name(label));
            diagnostics_logger_->debug(label_str, {{"pose_found", pose_found ? "true" : "false"}});

            if (!pose_found)
                continue;

            filter_measurements.push_back(
                RobotDescription{
                    get_frame_id_from_label(label),
                    label,
                    matrix_to_pose(tf_field_from_robot.tf),
                    Size{length, length, 0.1},
                    {vector_to_position(front_keypoint_in_field), vector_to_position(back_keypoint_in_field)}});
        }
        return filter_measurements;
    }

    std::vector<RobotDescription> RobotFrontBackSimpleFilter::update_filter(std::vector<RobotDescription> inputs, [[maybe_unused]] CommandFeedback command_feedback)
    {
        return inputs;
    }

    FrameId RobotFrontBackSimpleFilter::get_frame_id_from_label(const Label label) const
    {
        if (label_to_frame_id_.count(label) == 0)
        {
            return default_frame_id_;
        }
        return label_to_frame_id_.at(label);
    }

} // namespace auto_battlebot
