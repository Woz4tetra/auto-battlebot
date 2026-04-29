#pragma once

#include <map>
#include <vector>

#include "data_structures/measurement.hpp"
#include "data_structures/pose.hpp"
#include "data_structures/robot.hpp"
#include "diagnostics_logger/diagnostics_module_logger.hpp"

namespace auto_battlebot {
class FrameIdAssigner {
   public:
    FrameIdAssigner(double max_jump_distance, int max_consecutive_jump_rejects);

    void reset();
    void clear_last_position(FrameId frame_id);
    void set_last_position(FrameId frame_id, const Position &position);
    const std::map<FrameId, Position> &last_positions() const;

    /**
     * Greedily pair measurements with FrameIds, preferring (measurement, frame_id) pairs whose
     * distance to the FrameId's last known position is smallest. Pairs whose distance exceeds
     * `max_jump_distance_` are rejected for up to `max_consecutive_jump_rejects_` consecutive
     * frames per FrameId.
     *
     * If `per_measurement_allowed_frame_ids` is non-empty, it must have one entry per
     * measurement; only (m, f) pairs where `frame_ids[f]` appears in
     * `per_measurement_allowed_frame_ids[m]` are considered. This lets callers pool
     * measurements with heterogeneous label-to-FrameId eligibility into a single global
     * assignment instead of running multiple per-label calls (which couples the result to the
     * outer loop's iteration order).
     */
    std::vector<RobotDescription> assign(
        std::vector<MeasurementWithConfidence> &valid_measurements,
        const std::vector<FrameId> &frame_ids,
        std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger,
        const std::vector<std::vector<FrameId>> &per_measurement_allowed_frame_ids = {});

   private:
    double max_jump_distance_;
    int max_consecutive_jump_rejects_;
    std::map<FrameId, Position> last_position_per_frame_id_;
    std::map<FrameId, int> jump_reject_count_per_frame_id_;
};
}  // namespace auto_battlebot
