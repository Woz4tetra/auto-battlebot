#pragma once

#include <map>
#include <vector>

#include "data_structures/pose.hpp"
#include "data_structures/robot.hpp"
#include "diagnostics_logger/diagnostics_module_logger.hpp"

namespace auto_battlebot {
class FrameIdAssigner {
   public:
    using MeasurementWithConfidence = std::pair<double, RobotDescription>;

    FrameIdAssigner(double max_jump_distance, int max_consecutive_jump_rejects);

    void reset();
    void clear_last_position(FrameId frame_id);
    void set_last_position(FrameId frame_id, const Position &position);
    const std::map<FrameId, Position> &last_positions() const;

    std::vector<RobotDescription> assign(
        std::vector<MeasurementWithConfidence> &valid_measurements,
        const std::vector<FrameId> &frame_ids,
        std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger);

   private:
    double max_jump_distance_;
    int max_consecutive_jump_rejects_;
    std::map<FrameId, Position> last_position_per_frame_id_;
    std::map<FrameId, int> jump_reject_count_per_frame_id_;
};
}  // namespace auto_battlebot
