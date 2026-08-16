#pragma once

#include <map>
#include <memory>
#include <vector>

#include "robot_filter/robot_filter_interface.hpp"
#include "simulation/sim_connection.hpp"
#include "time/clock_interface.hpp"

namespace auto_battlebot {

/**
 * Robot filter that bypasses all perception and uses ground truth poses
 * from the simulation. Reads the latest GT poses cached in SimConnection.
 */
class GroundTruthRobotFilter : public RobotFilterInterface {
   public:
    explicit GroundTruthRobotFilter(std::shared_ptr<ClockInterface> clock)
        : clock_(std::move(clock)) {}

    bool initialize(int opponent_count) override;

    /** Reads the latest ground-truth poses from SimConnection. Perception inputs are ignored. */
    void correct(KeypointsStamped keypoints, FieldDescription field, CameraInfo camera_info,
                 KeypointsStamped robot_blob_keypoints) override;

    RobotDescriptionsStamped state() const override { return state_; }

   private:
    std::vector<FrameId> our_frame_ids_;
    std::vector<FrameId> opponent_frame_ids_;
    std::shared_ptr<SimConnection> connection_;
    std::shared_ptr<ClockInterface> clock_;
    /** Ground truth needs no propagation, so this only changes on correct(). */
    RobotDescriptionsStamped state_;
};

}  // namespace auto_battlebot
