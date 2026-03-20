#pragma once

#include <map>
#include <memory>
#include <vector>

#include "robot_filter/robot_filter_interface.hpp"
#include "simulation/sim_connection.hpp"

namespace auto_battlebot {

/**
 * Robot filter that bypasses all perception and uses ground truth poses
 * from the simulation. Reads the latest GT poses cached in SimConnection.
 */
class GroundTruthRobotFilter : public RobotFilterInterface {
   public:
    GroundTruthRobotFilter() = default;

    bool initialize(const std::vector<RobotConfig> &robots) override;
    RobotDescriptionsStamped update(KeypointsStamped keypoints, FieldDescription field,
                                    CameraInfo camera_info, MaskStamped robot_mask,
                                    CommandFeedback command_feedback) override;

   private:
    std::vector<RobotConfig> robot_configs_;
    std::vector<FrameId> our_frame_ids_;
    std::vector<FrameId> opponent_frame_ids_;
    std::shared_ptr<SimConnection> connection_;
};

}  // namespace auto_battlebot
