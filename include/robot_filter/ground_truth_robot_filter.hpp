#pragma once

#include <map>
#include <memory>
#include <optional>
#include <vector>

#include "robot_filter/robot_filter_interface.hpp"
#include "simulation/sim_connection.hpp"
#include "time/clock_interface.hpp"

namespace auto_battlebot {

class GroundTruthRobotFilter : public RobotFilterInterface {
   public:
    explicit GroundTruthRobotFilter(std::shared_ptr<ClockInterface> clock, int neutral_count = 0)
        : clock_(std::move(clock)) {
        trailing_neutral_slot_count_ = neutral_count;
    }

    bool initialize(int opponent_count) override;

    void correct(ModelResultStamped keypoints, FieldDescription field, CameraInfo camera_info,
                 ModelResultStamped robot_blob_keypoints) override;

    RobotDescriptionsStamped state() const override { return state_; }

   private:
    std::optional<RobotDescription> describe(FrameId frame_id, Label label, Group group,
                                             const Pose2D &gt_pose, double now, double dt);

    Velocity2D velocity_from_delta(FrameId frame_id, const Pose2D &pose, double now);

    static bool interval_usable(double interval);

    static double normalize_angle(double angle);

    std::vector<FrameId> our_frame_ids_;
    std::vector<FrameId> opponent_frame_ids_;
    std::vector<FrameId> neutral_frame_ids_;
    int trailing_neutral_slot_count_ = 0;
    std::shared_ptr<SimConnection> connection_;
    std::shared_ptr<ClockInterface> clock_;
    RobotDescriptionsStamped state_;
    std::map<FrameId, Pose2D> prev_poses_;
    std::map<FrameId, Pose2D> measured_poses_;
    std::map<FrameId, double> measured_stamps_;
    std::map<FrameId, Velocity2D> prev_velocities_;
    double prev_stamp_ = 0.0;
};

}  // namespace auto_battlebot
