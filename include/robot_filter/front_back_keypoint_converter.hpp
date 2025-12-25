#pragma once

#include <map>
#include <vector>
#include <Eigen/Dense>
#include "data_structures.hpp"
#include "enums.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"
#include "diagnostics_logger/diagnostics_module_logger.hpp"

namespace auto_battlebot
{
    struct FrontBackAssignment
    {
        Eigen::Vector3d front = Eigen::Vector3d::Zero();
        Eigen::Vector3d back = Eigen::Vector3d::Zero();
    };

    struct FrontBackKeypointConverterConfig
    {
        std::vector<KeypointLabel> front_keypoints;
        std::vector<KeypointLabel> back_keypoints;
    };

    /**
     * @brief Converts 2D keypoints to 3D front/back assignments by projecting onto a plane
     */
    class FrontBackKeypointConverter
    {
    public:
        FrontBackKeypointConverter(const FrontBackKeypointConverterConfig &config);

        /**
         * @brief Convert keypoints to front/back 3D positions in camera frame
         * @param keypoints Input 2D keypoints
         * @param field Field description with plane transform
         * @param camera_info Camera intrinsics
         * @return Map of label to front/back 3D positions in camera frame
         */
        std::map<Label, FrontBackAssignment> convert(
            const KeypointsStamped &keypoints,
            const FieldDescription &field,
            const CameraInfo &camera_info);

        /**
         * @brief Compute pose from front and back 3D points
         * @param front_point Front keypoint position
         * @param back_point Back keypoint position
         * @param out_transform Output transform (pose)
         * @return True if pose was successfully computed
         */
        bool get_pose_from_points(
            const Eigen::Vector3d &front_point,
            const Eigen::Vector3d &back_point,
            Transform &out_transform);

        /**
         * @brief Transform a point using a 4x4 transformation matrix
         */
        static Eigen::Vector3d transform_point(const Eigen::Matrix4d &tf, const Eigen::Vector3d &point);

    private:
        FrontBackKeypointConverterConfig config_;
        std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;

        Eigen::Vector3d plane_from_3_points(
            const Eigen::Vector3d &point1,
            const Eigen::Vector3d &point2,
            const Eigen::Vector3d &point3);

        Eigen::Vector3d transform_to_plane_normal(const Transform &transform);

        Eigen::Vector3d find_ray_plane_intersection(
            const Eigen::Vector3d &ray,
            const Eigen::Vector3d &plane_center,
            const Eigen::Vector3d &plane_normal);

        Eigen::Vector3d project_keypoint_onto_plane(
            const Keypoint &keypoint,
            const Eigen::Vector3d &plane_center,
            const Eigen::Vector3d &plane_normal,
            const CameraInfo &camera_info);
    };

} // namespace auto_battlebot
