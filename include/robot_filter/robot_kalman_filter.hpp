#pragma once

#include <unordered_map>
#include "robot_filter/config.hpp"
#include "robot_filter/robot_filter_interface.hpp"
#include "transform_utils.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"
#include "diagnostics_logger/diagnostics_module_logger.hpp"

namespace auto_battlebot
{
    struct FrontBackAssignment
    {
        Eigen::Vector3d front = Eigen::Vector3d::Zero();
        Eigen::Vector3d back = Eigen::Vector3d::Zero();
    };

    class RobotKalmanFilter : public RobotFilterInterface
    {
    public:
        RobotKalmanFilter(RobotKalmanFilterConfiguration &config);

        bool initialize(const std::vector<RobotConfig> &robots) override;
        RobotDescriptionsStamped update(KeypointsStamped keypoints, FieldDescription field, CameraInfo camera_info) override;

    private:
        std::unordered_map<Label, RobotConfig> robot_configs_;
        RobotKalmanFilterConfiguration config_;
        std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;

        Eigen::Vector3d plane_from_3_points(Eigen::Vector3d point1, Eigen::Vector3d point2, Eigen::Vector3d point3);
        Eigen::Vector3d transform_to_plane_normal(Transform transform);
        Eigen::Vector3d find_ray_plane_intersection(Eigen::Vector3d ray, Eigen::Vector3d plane_center, Eigen::Vector3d plane_normal);
        Eigen::Vector3d project_keypoint_onto_plane(Keypoint keypoint, Eigen::Vector3d plane_center, Eigen::Vector3d plane_normal, CameraInfo camera_info);
        std::map<Label, FrontBackAssignment> compute_front_back_mapping(KeypointsStamped keypoints, FieldDescription field, CameraInfo camera_info);
        bool get_pose_from_points(const Eigen::Vector3d &front_point, const Eigen::Vector3d &back_point, Transform &out_transform);
    };

} // namespace auto_battlebot
