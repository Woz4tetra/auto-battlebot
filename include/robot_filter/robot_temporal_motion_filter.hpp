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

    void reset();

    std::vector<RobotDescription> update_with_prediction(std::vector<RobotDescription> inputs,
                                                         const CommandFeedback &command_feedback,
                                                         double timestamp,
                                                         FrameIdAssigner &frame_id_assigner);

    void estimate_velocities(std::vector<RobotDescription> &descriptions, double timestamp,
                             const CommandFeedback &command_feedback);

   private:
    double velocity_ema_alpha_;
    std::map<FrameId, RobotVelocityState> velocity_state_per_frame_id_;
    std::map<FrameId, RobotDescription> last_description_per_frame_id_;
};
}  // namespace auto_battlebot
