#pragma once

#include <map>
#include <vector>

#include "data_structures/command_feedback.hpp"
#include "data_structures/pose.hpp"
#include "data_structures/robot.hpp"
#include "plant/plant_params.hpp"
#include "robot_filter/frame_id_assigner.hpp"

namespace auto_battlebot {

class RobotTemporalMotionFilter {
   public:
    explicit RobotTemporalMotionFilter(const JigPlantParams &plant) : plant_(plant) {}

    void reset();

    std::vector<RobotDescription> update_with_prediction(
        std::vector<RobotDescription> inputs, const CommandFeedback &command_feedback,
        double timestamp, FrameIdAssigner &frame_id_assigner, const FieldDescription &field,
        double field_bounds_margin_meters, double our_robot_hold_window_s);

   private:
    std::map<FrameId, double> last_output_timestamp_per_frame_id_;
    std::map<FrameId, double> last_measured_timestamp_s_per_frame_id_;
    std::map<FrameId, RobotDescription> last_description_per_frame_id_;
    JigPlantParams plant_;
};
}  // namespace auto_battlebot
