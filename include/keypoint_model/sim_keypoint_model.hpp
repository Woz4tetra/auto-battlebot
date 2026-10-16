#pragma once

#include <Eigen/Dense>
#include <memory>
#include <vector>

#include "keypoint_model/config.hpp"
#include "keypoint_model/keypoint_model_interface.hpp"
#include "simulation/sim_connection.hpp"

namespace auto_battlebot {

/**
 * Keypoint model for the lockstep sim. Builds front and back keypoints from the poses the sim's
 * sensing model delivered (simulation/kinematic_sim_server.py, docs/plans/sensing_model_plan.md),
 * so the real RobotFrontBackFilter and KalmanMotionEstimator run on simulated perception: its
 * latency, dropouts, noise and heading flips. A flipped heading arrives as swapped front and back
 * keypoints, the way a real flip does.
 *
 * Keypoints are projected at floor height. The sim's depth image is the bare floor, so the height
 * gate abstains on these box-less detections and stamps each keypoint with the height it measured
 * there, which is zero. Projecting at z = 0 keeps the converter's round trip exact.
 *
 * The result carries the frame's capture stamp. The tick that delivers it is later by the sensing
 * model's latency, which is what the filter sees on the robot.
 */
class SimKeypointModel : public KeypointModelInterface {
   public:
    explicit SimKeypointModel(const SimKeypointModelConfiguration &config);

    bool initialize() override { return true; }
    ModelResultStamped update(RgbImage image, const CameraInfo &camera_info) override;

   private:
    SimKeypointModelConfiguration config_;
    std::shared_ptr<SimConnection> connection_;
};

/**
 * The conversion behind SimKeypointModel::update, separate so it can be tested without a socket.
 * Slot i of `frame` is labelled `labels[i]`; slots past the end of `labels` are skipped. A robot
 * whose front or back keypoint lands behind the camera or outside the image is dropped, as a real
 * detector would miss it. On a frame that was not delivered the result is empty and stamped with
 * `image_header`.
 */
ModelResultStamped sim_measurements_to_keypoints(const SimMeasurementFrame &frame,
                                                 const std::vector<Label> &labels,
                                                 double keypoint_offset_m,
                                                 const Eigen::Matrix4d &camera_from_field,
                                                 const CameraInfo &camera_info,
                                                 const Header &image_header);

}  // namespace auto_battlebot
