#pragma once

#include <map>
#include <vector>

#include "data_structures/command_feedback.hpp"
#include "data_structures/pose.hpp"
#include "data_structures/robot.hpp"
#include "robot_filter/frame_id_assigner.hpp"

namespace auto_battlebot {

class RobotTemporalMotionFilter {
   public:
    RobotTemporalMotionFilter() = default;

    /** Clears all tracked robot state. Call before starting a new match. */
    void reset();

    /**
     * Merges new measurements into tracked state and returns one RobotDescription per tracked
     * robot. Robots with no new measurement this frame are predicted forward using their last
     * commanded velocity and clamped to field bounds. Predicted robots are flagged is_stale=true.
     *
     * An our-robot (Group::OURS) track that has gone unmeasured for longer than
     * our_robot_hold_window_s (seconds) is dropped from the output and forgotten rather than held
     * indefinitely -- the stale-identity decay. A window <= 0 disables the decay (hold forever).
     */
    std::vector<RobotDescription> update_with_prediction(
        std::vector<RobotDescription> inputs, const CommandFeedback &command_feedback,
        double timestamp, FrameIdAssigner &frame_id_assigner, const FieldDescription &field,
        double field_bounds_margin_meters, double our_robot_hold_window_s);

   private:
    // Last timestamp each tracked robot appeared in the output. Used only to compute the dt for the
    // commanded-velocity dead-reckoning of unmeasured robots in update_with_prediction.
    std::map<FrameId, double> last_timestamp_per_frame_id_;
    // Last timestamp each tracked robot had a real measurement (not a prediction). Drives the
    // our-robot stale-identity decay in update_with_prediction.
    std::map<FrameId, double> last_measured_timestamp_per_frame_id_;
    std::map<FrameId, RobotDescription> last_description_per_frame_id_;
};
}  // namespace auto_battlebot
