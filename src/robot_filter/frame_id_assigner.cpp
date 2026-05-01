#include "robot_filter/frame_id_assigner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <magic_enum.hpp>

namespace auto_battlebot {
namespace {
double position_distance(const Position &a, const Position &b) {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}
}  // namespace

FrameIdAssigner::FrameIdAssigner(double max_jump_distance, int max_consecutive_jump_rejects)
    : max_jump_distance_(max_jump_distance),
      max_consecutive_jump_rejects_(max_consecutive_jump_rejects) {}

void FrameIdAssigner::reset() {
    last_position_per_frame_id_.clear();
    jump_reject_count_per_frame_id_.clear();
}

void FrameIdAssigner::clear_last_position(FrameId frame_id) {
    last_position_per_frame_id_.erase(frame_id);
}

void FrameIdAssigner::set_last_position(FrameId frame_id, const Position &position) {
    last_position_per_frame_id_[frame_id] = position;
}

const std::map<FrameId, Position> &FrameIdAssigner::last_positions() const {
    return last_position_per_frame_id_;
}

std::vector<RobotDescription> FrameIdAssigner::assign(
    std::vector<MeasurementWithConfidence> &valid_measurements,
    const std::vector<FrameId> &frame_ids,
    std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger,
    const std::vector<std::vector<FrameId>> &per_measurement_allowed_frame_ids) {
    std::vector<RobotDescription> result;
    const size_t N = std::min(valid_measurements.size(), frame_ids.size());
    if (N == 0) return result;

    const bool has_constraints = !per_measurement_allowed_frame_ids.empty();
    auto is_allowed = [&](size_t m, size_t f) {
        if (!has_constraints) return true;
        const auto &allowed = per_measurement_allowed_frame_ids[m];
        return std::find(allowed.begin(), allowed.end(), frame_ids[f]) != allowed.end();
    };

    std::vector<bool> meas_assigned(valid_measurements.size(), false);
    std::vector<bool> frame_id_assigned(frame_ids.size(), false);
    constexpr double no_previous = std::numeric_limits<double>::infinity();

    for (size_t n = 0; n < N; n++) {
        double best_dist = no_previous;
        size_t best_meas = 0;
        size_t best_fid = 0;
        bool best_has_previous = false;
        bool any_pair_found = false;

        for (size_t m = 0; m < valid_measurements.size(); m++) {
            if (meas_assigned[m]) continue;
            const Position &pos = valid_measurements[m].description.pose.position;
            for (size_t f = 0; f < frame_ids.size(); f++) {
                if (frame_id_assigned[f]) continue;
                if (!is_allowed(m, f)) continue;
                any_pair_found = true;
                const FrameId fid = frame_ids[f];
                auto it = last_position_per_frame_id_.find(fid);
                double d = no_previous;
                bool has_previous = false;
                if (it != last_position_per_frame_id_.end()) {
                    d = position_distance(pos, it->second);
                    has_previous = true;
                }
                if (!best_has_previous && has_previous) {
                    best_dist = d;
                    best_meas = m;
                    best_fid = f;
                    best_has_previous = true;
                } else if (best_has_previous == has_previous && d < best_dist) {
                    best_dist = d;
                    best_meas = m;
                    best_fid = f;
                    best_has_previous = has_previous;
                }
            }
        }

        if (!any_pair_found) break;

        if (best_dist == no_previous && !best_has_previous) {
            // No FrameId has a previous position; pick the first allowed unassigned pair.
            bool fallback_found = false;
            for (size_t m = 0; m < valid_measurements.size() && !fallback_found; m++) {
                if (meas_assigned[m]) continue;
                for (size_t f = 0; f < frame_ids.size(); f++) {
                    if (frame_id_assigned[f]) continue;
                    if (!is_allowed(m, f)) continue;
                    best_meas = m;
                    best_fid = f;
                    fallback_found = true;
                    break;
                }
            }
            if (!fallback_found) break;
        }

        meas_assigned[best_meas] = true;
        frame_id_assigned[best_fid] = true;
        const FrameId assigned_fid = frame_ids[best_fid];

        if (best_has_previous && best_dist > max_jump_distance_) {
            int &reject_count = jump_reject_count_per_frame_id_[assigned_fid];
            reject_count++;

            if (reject_count <= max_consecutive_jump_rejects_) {
                diagnostics_logger->debug(
                    "jump_reject",
                    {{"frame_id", std::string(magic_enum::enum_name(assigned_fid))},
                     {"distance", best_dist},
                     {"threshold", max_jump_distance_},
                     {"consecutive", reject_count}},
                    "Measurement rejected: jump too large");
                continue;
            }

            diagnostics_logger->debug(
                "jump_reject",
                {{"frame_id", std::string(magic_enum::enum_name(assigned_fid))},
                 {"distance", best_dist},
                 {"consecutive", reject_count}},
                "Accepting after max consecutive rejects");
        }

        jump_reject_count_per_frame_id_[assigned_fid] = 0;
        valid_measurements[best_meas].description.frame_id = assigned_fid;
        last_position_per_frame_id_[assigned_fid] =
            valid_measurements[best_meas].description.pose.position;
        result.push_back(std::move(valid_measurements[best_meas].description));
    }
    return result;
}
}  // namespace auto_battlebot
