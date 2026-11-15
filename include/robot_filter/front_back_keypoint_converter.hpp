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
#include "robot_filter/keypoint_heights.hpp"

namespace auto_battlebot {
struct FrontBackAssignment {
    Eigen::Vector3d front = Eigen::Vector3d::Zero();
    Eigen::Vector3d back = Eigen::Vector3d::Zero();
    double reference_offset_m = 0.0;
};

struct FrontBackKeypointConverterConfig {
    std::vector<KeypointLabel> front_keypoints;
    std::vector<KeypointLabel> back_keypoints;
    KeypointHeights keypoint_heights;
};

class FrontBackKeypointConverter {
   public:
    explicit FrontBackKeypointConverter(const FrontBackKeypointConverterConfig &config);

    std::map<Label, std::vector<std::pair<FrontBackAssignment, double>>> convert(
        const ModelResultStamped &model_results, const FieldDescription &field,
        const CameraInfo &camera_info);

    /**
     * @brief Compute pose from front and back 3D points
     * @param front_point Front keypoint position
     * @param back_point Back keypoint position
     * @param reference_offset_m The robot's reference point, this far ahead of the keypoints'
     *        midpoint along the back-to-front direction (Keypoint::reference_offset_m)
     * @param out_transform Output transform (pose)
     * @return True if pose was successfully computed
     */
    bool get_pose_from_points(const Eigen::Vector3d &front_point, const Eigen::Vector3d &back_point,
                              double reference_offset_m, Transform &out_transform);

   private:
    FrontBackKeypointConverterConfig config_;
    std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;
};

}  // namespace auto_battlebot
