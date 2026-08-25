#include "hazards/hazard_assembler.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "enums/frame_id.hpp"
#include "hazards/hazard_geometry.hpp"
#include "transform_utils.hpp"

namespace auto_battlebot {
HazardAssembler::HazardAssembler(HazardAssemblerConfig config,
                                 std::vector<StaticHazardConfig> static_hazards,
                                 std::shared_ptr<ClockInterface> clock)
    : config_(config),
      static_hazards_(std::move(static_hazards)),
      clock_(std::move(clock)),
      logger_(DiagnosticsLogger::get_logger("hazards")),
      last_our_half_diagonal_m_(config.our_half_diagonal_fallback_m) {}

double HazardAssembler::our_half_diagonal(const RobotDescriptionsStamped &robots) const {
    for (const auto &robot : robots.descriptions) {
        if (robot.frame_id != FrameId::OUR_ROBOT_1) continue;
        const double radius = half_diagonal(robot.size);
        if (radius > 1e-6) {
            last_our_half_diagonal_m_ = radius;
            return radius;
        }
    }
    return last_our_half_diagonal_m_;
}

void HazardAssembler::assemble(const RobotDescriptionsStamped &robots, FieldDescription &field) {
    const double now_s = clock_->now();
    const double ours = our_half_diagonal(robots);

    std::vector<FieldHazard> hazards;
    hazards.reserve(static_hazards_.size() + held_tracks_.size());

    for (const auto &config : static_hazards_) {
        FieldHazard hazard;
        hazard.center.x = config.center_x;
        hazard.center.y = config.center_y;
        hazard.radius = config.radius + ours + config_.static_margin_m;
        hazard.source = HazardSource::STATIC;
        hazards.push_back(hazard);
    }

    // Live neutral tracks refresh their held entry; stale ones keep the last live geometry until
    // the hold window expires.
    std::vector<HeldTrack> refreshed;
    for (const auto &robot : robots.descriptions) {
        if (robot.group != Group::NEUTRAL) continue;
        if (robot.is_stale) continue;
        HeldTrack track;
        track.hazard.center = pose_to_pose2d(robot.pose);
        track.hazard.radius = half_diagonal(robot.size) + ours + config_.tracked_margin_m;
        track.hazard.velocity = robot.velocity;
        track.hazard.source = HazardSource::TRACKED;
        track.last_live_s = now_s;
        refreshed.push_back(track);
    }

    if (refreshed.empty()) {
        for (const auto &held : held_tracks_) {
            if (now_s - held.last_live_s <= config_.tracked_hold_s) {
                refreshed.push_back(held);
            }
        }
    }
    held_tracks_ = std::move(refreshed);
    for (const auto &held : held_tracks_) {
        hazards.push_back(held.hazard);
    }

    int static_count = 0;
    int tracked_count = 0;
    for (const auto &hazard : hazards) {
        if (hazard.source == HazardSource::STATIC) {
            ++static_count;
        } else {
            ++tracked_count;
        }
    }

    // Its own channel, every cycle, so a replay can tell "the controller did not know" from
    // "the controller knew and drove in anyway".
    DiagnosticsData values{
        {"count", static_cast<int>(hazards.size())},
        {"static_count", static_count},
        {"tracked_count", tracked_count},
        {"our_half_diagonal_m", ours},
    };
    for (size_t i = 0; i < hazards.size(); ++i) {
        const std::string prefix = std::to_string(i) + "/";
        values[prefix + "x"] = hazards[i].center.x;
        values[prefix + "y"] = hazards[i].center.y;
        values[prefix + "radius"] = hazards[i].radius;
        values[prefix + "source"] =
            std::string(hazards[i].source == HazardSource::STATIC ? "STATIC" : "TRACKED");
    }
    logger_->debug("hazards", values);

    field.hazards = std::move(hazards);
}
}  // namespace auto_battlebot
