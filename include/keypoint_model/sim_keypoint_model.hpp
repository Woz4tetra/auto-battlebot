#pragma once

#include <Eigen/Dense>
#include <memory>
#include <vector>

#include "keypoint_model/config.hpp"
#include "keypoint_model/keypoint_model_interface.hpp"
#include "simulation/sim_connection.hpp"

namespace auto_battlebot {

class SimKeypointModel : public KeypointModelInterface {
   public:
    explicit SimKeypointModel(const SimKeypointModelConfiguration &config);

    bool initialize() override { return true; }
    ModelResultStamped update(RgbImage image, const CameraInfo &camera_info) override;

   private:
    SimKeypointModelConfiguration config_;
    std::shared_ptr<SimConnection> connection_;
};

ModelResultStamped sim_measurements_to_keypoints(const SimMeasurementFrame &frame,
                                                 const std::vector<Label> &labels,
                                                 double keypoint_offset_m,
                                                 const Eigen::Matrix4d &camera_from_field,
                                                 const CameraInfo &camera_info,
                                                 const Header &image_header);

}  // namespace auto_battlebot
