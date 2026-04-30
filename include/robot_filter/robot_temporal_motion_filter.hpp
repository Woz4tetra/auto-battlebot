#pragma once

#include <map>
#include <vector>

#include "data_structures/command_feedback.hpp"
#include "data_structures/pose.hpp"
#include "data_structures/robot.hpp"
#include "robot_filter/frame_id_assigner.hpp"

namespace auto_battlebot {
/** Per-robot state used to estimate velocity across frames. */
struct RobotVelocityState {
    double timestamp = 0.0;
    Pose2D pose;
    Velocity2D smoothed_velocity;
};

class RobotTemporalMotionFilter {
   public:
    explicit RobotTemporalMotionFilter(double velocity_ema_alpha);

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

    /**
     * Fills velocity on each description in-place. Our robots use commanded velocity; opponents
     * use EMA-smoothed finite differences from consecutive poses.
     */
    void estimate_velocities(std::vector<RobotDescription> &descriptions, double timestamp,
                             const CommandFeedback &command_feedback);

   private:
    double velocity_ema_alpha_;
    std::map<FrameId, RobotVelocityState> velocity_state_per_frame_id_;
    std::map<FrameId, RobotDescription> last_description_per_frame_id_;
};
}  // namespace auto_battlebot
