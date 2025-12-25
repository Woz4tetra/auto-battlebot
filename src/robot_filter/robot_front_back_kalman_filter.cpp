#include "robot_filter/robot_front_back_kalman_filter.hpp"
#include "transform_utils.hpp"

namespace auto_battlebot
{
    RobotFrontBackKalmanFilter::RobotFrontBackKalmanFilter(RobotFrontBackKalmanFilterConfiguration &config)
        : config_(config)
    {
        diagnostics_logger_ = DiagnosticsLogger::get_logger("robot_front_back_kalman_filter");

        FrontBackKeypointConverterConfig converter_config;
        converter_config.front_keypoints = config.front_keypoints;
        converter_config.back_keypoints = config.back_keypoints;
        keypoint_converter_ = std::make_unique<FrontBackKeypointConverter>(converter_config);
    }

    bool RobotFrontBackKalmanFilter::initialize(const std::vector<RobotConfig> &robots)
    {
        robot_configs_.clear();
        for (const auto &robot : robots)
        {
            robot_configs_[robot.label] = robot;
        }
        return true;
    }

    RobotDescriptionsStamped RobotFrontBackKalmanFilter::update(KeypointsStamped keypoints, FieldDescription field, CameraInfo camera_info)
    {
        RobotDescriptionsStamped result;
        result.header.frame_id = FrameId::FIELD;
        result.header.stamp = keypoints.header.stamp;

        Eigen::Matrix4d tf_fieldcenter_from_camera = field.tf_camera_from_fieldcenter.tf.inverse();

        std::map<Label, FrontBackAssignment> front_back_mapping = keypoint_converter_->convert(keypoints, field, camera_info);
        for (const auto &[label, assignment] : front_back_mapping)
        {
            FrameId robot_frame_id = config_.label_to_frame_ids[label][0];

            Eigen::Vector3d front_keypoint_in_field = FrontBackKeypointConverter::transform_point(tf_fieldcenter_from_camera, assignment.front);
            Eigen::Vector3d back_keypoint_in_field = FrontBackKeypointConverter::transform_point(tf_fieldcenter_from_camera, assignment.back);
            double length = (front_keypoint_in_field - back_keypoint_in_field).norm();

            Transform tf_field_from_robot;
            bool pose_found = keypoint_converter_->get_pose_from_points(front_keypoint_in_field, back_keypoint_in_field, tf_field_from_robot);

            std::string label_str = std::string(magic_enum::enum_name(label));
            diagnostics_logger_->debug(label_str, {{"pose_found", pose_found ? "true" : "false"}});

            if (!pose_found)
                continue;

            result.descriptions.push_back(
                RobotDescription{
                    robot_frame_id,
                    label,
                    matrix_to_pose(tf_field_from_robot.tf),
                    Size{length, length, 0.1},
                    {vector_to_position(front_keypoint_in_field), vector_to_position(back_keypoint_in_field)}});
        }
        diagnostics_logger_->debug({{"num_robots", (int)result.descriptions.size()}});

        return result;
    }

} // namespace auto_battlebot
