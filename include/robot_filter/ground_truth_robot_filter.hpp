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
    /**
     * Build one description from a ground-truth slot. A non-finite pose means the sim did not
     * observe this robot on this frame, which yields a held pose flagged stale rather than a
     * missing entry, so the ground-truth ordering stays fixed.
     */
    RobotDescription describe(FrameId frame_id, Label label, Group group, const Pose2D &gt_pose,
                              double dt, bool dt_usable);

    /**
     * Field-frame velocity from this robot's pose delta since the last correct(), and record the
     * pose for next time. Returns zero on the first frame or an unusable dt.
     */
    Velocity2D velocity_from_delta(FrameId frame_id, const Pose2D &pose, double dt, bool dt_usable);

    static double normalize_angle(double angle);

    std::vector<FrameId> our_frame_ids_;
    std::vector<FrameId> opponent_frame_ids_;
    std::shared_ptr<SimConnection> connection_;
    std::shared_ptr<ClockInterface> clock_;
    /** Ground truth needs no propagation, so this only changes on correct(). */
    RobotDescriptionsStamped state_;
    std::map<FrameId, Pose2D> prev_poses_;
    std::map<FrameId, Velocity2D> prev_velocities_;
    double prev_stamp_ = 0.0;
};

}  // namespace auto_battlebot
