#pragma once

#include <Eigen/Dense>
#include <map>
#include <memory>
#include <vector>

#include "data_structures/camera.hpp"
#include "data_structures/field.hpp"
#include "data_structures/keypoint.hpp"
#include "data_structures/transform.hpp"
#include "diagnostics_logger/diagnostics_module_logger.hpp"

namespace auto_battlebot {
struct FrontBackAssignment {
    Eigen::Vector3d front = Eigen::Vector3d::Zero();
    Eigen::Vector3d back = Eigen::Vector3d::Zero();
};

struct FrontBackKeypointConverterConfig {
    std::vector<KeypointLabel> front_keypoints;
    std::vector<KeypointLabel> back_keypoints;
};

/**
 * @brief Converts 2D keypoints to 3D front/back assignments by projecting onto a plane
 */
class FrontBackKeypointConverter {
   public:
    FrontBackKeypointConverter(const FrontBackKeypointConverterConfig &config);

    /**
     * @brief Convert keypoints to front/back 3D positions in camera frame
     * @param keypoints Input 2D keypoints (grouped by label and detection_index)
     * @param field Field description with plane transform
     * @param camera_info Camera intrinsics
     * @return Map of label to list of (front/back assignment, confidence) per instance
     */
    std::map<Label, std::vector<std::pair<FrontBackAssignment, double>>> convert(
        const KeypointsStamped &keypoints, const FieldDescription &field,
        const CameraInfo &camera_info);

    /**
     * @brief Compute pose from front and back 3D points
     * @param front_point Front keypoint position
     * @param back_point Back keypoint position
     * @param out_transform Output transform (pose)
     * @return True if pose was successfully computed
     */
    bool get_pose_from_points(const Eigen::Vector3d &front_point, const Eigen::Vector3d &back_point,
                              Transform &out_transform);

    /**
     * @brief Transform a point using a 4x4 transformation matrix
     */
    static Eigen::Vector3d transform_point(const Eigen::Matrix4d &tf, const Eigen::Vector3d &point);

   private:
    FrontBackKeypointConverterConfig config_;
    std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;

    Eigen::Vector3d project_keypoint_onto_plane(const Keypoint &keypoint,
                                                const Eigen::Vector3d &plane_center,
                                                const Eigen::Vector3d &plane_normal,
                                                const CameraInfo &camera_info);
};

}  // namespace auto_battlebot
