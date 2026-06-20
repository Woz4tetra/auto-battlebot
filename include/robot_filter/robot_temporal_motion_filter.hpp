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
     */
    std::vector<RobotDescription> update_with_prediction(std::vector<RobotDescription> inputs,
                                                         const CommandFeedback &command_feedback,
                                                         double timestamp,
                                                         FrameIdAssigner &frame_id_assigner,
                                                         const FieldDescription &field,
                                                         double field_bounds_margin_meters);

   private:
    // Last timestamp each tracked robot appeared in the output. Used only to compute the dt for the
    // commanded-velocity dead-reckoning of unmeasured robots in update_with_prediction.
    std::map<FrameId, double> last_timestamp_per_frame_id_;
    std::map<FrameId, RobotDescription> last_description_per_frame_id_;
};
}  // namespace auto_battlebot
