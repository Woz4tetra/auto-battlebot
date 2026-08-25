#pragma once

#include <map>
#include <memory>
#include <optional>
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
    explicit GroundTruthRobotFilter(std::shared_ptr<ClockInterface> clock, int neutral_count = 0)
        : clock_(std::move(clock)) {
        neutral_count_ = neutral_count;
    }

    bool initialize(int opponent_count) override;

    /** Reads the latest ground-truth poses from SimConnection. Perception inputs are ignored. */
    void correct(KeypointsStamped keypoints, FieldDescription field, CameraInfo camera_info,
                 KeypointsStamped robot_blob_keypoints) override;

    RobotDescriptionsStamped state() const override { return state_; }

   private:
    /**
     * Build one description from a ground-truth slot. A non-finite pose means the sim did not
     * observe this robot on this frame, which yields a held pose flagged stale rather than a
     * missing entry. Returns nothing when the slot has never been observed: there is no pose to
     * hold, and inventing one would hand consumers a fabricated position.
     */
    std::optional<RobotDescription> describe(FrameId frame_id, Label label, Group group,
                                             const Pose2D &gt_pose, double now, double dt);

    /**
     * Field-frame velocity from this robot's pose delta since its last *measurement*, and record
     * the measurement for next time. The interval is measured-to-measured rather than
     * frame-to-frame: across a dropout the pose delta spans the whole gap, so dividing it by one
     * frame interval would scale the velocity by the number of frames missed. Returns zero on the
     * first measurement or an unusable interval.
     */
    Velocity2D velocity_from_delta(FrameId frame_id, const Pose2D &pose, double now);

    /** True when an elapsed time is a plausible measurement-to-measurement interval. */
    static bool interval_usable(double interval);

    static double normalize_angle(double angle);

    std::vector<FrameId> our_frame_ids_;
    std::vector<FrameId> opponent_frame_ids_;
    /** Trailing ground-truth slots mapped to NEUTRAL_ROBOT_* instead of THEIR_ROBOT_*. */
    std::vector<FrameId> neutral_frame_ids_;
    int neutral_count_ = 0;
    std::shared_ptr<SimConnection> connection_;
    std::shared_ptr<ClockInterface> clock_;
    /** Ground truth needs no propagation, so this only changes on correct(). */
    RobotDescriptionsStamped state_;
    /** Best current estimate per track: the measurement, or the coast propagated from it. */
    std::map<FrameId, Pose2D> prev_poses_;
    /** Last actual measurement and its stamp. Never written by the coast branch, so a returning
     * frame differences against real data instead of against the propagated pose it just made. */
    std::map<FrameId, Pose2D> measured_poses_;
    std::map<FrameId, double> measured_stamps_;
    std::map<FrameId, Velocity2D> prev_velocities_;
    double prev_stamp_ = 0.0;
};

}  // namespace auto_battlebot
