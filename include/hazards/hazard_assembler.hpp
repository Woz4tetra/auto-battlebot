#pragma once

#include <memory>
#include <vector>

#include "data_structures/field.hpp"
#include "data_structures/robot.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"
#include "hazards/hazard_config.hpp"
#include "time/clock_interface.hpp"

namespace auto_battlebot {

struct HazardAssemblerConfig {
    /** Extra clearance (m) added to every static hazard on top of our robot's half-diagonal. */
    double static_margin_m = 0.10;
    /** Extra clearance (m) for hazards derived from live neutral tracks. Smaller than the static
     * margin on purpose: the house bot is a hazard we must not hit, but treating it as a large
     * keep-out disc makes big parts of the field unreachable when it parks near the middle. */
    double tracked_margin_m = 0.05;
    /** How long a stale neutral track keeps producing a hazard. Forgetting a hazard is worse than
     * holding a slightly wrong one, so the disc outlives the track by this much. */
    double tracked_hold_s = 0.75;
    /** Fallback half-diagonal (m) for our robot when no our-robot track is present this cycle.
     * Without one, a dropout would briefly shrink every keep-out disc. */
    double our_half_diagonal_fallback_m = 0.11;
};

/** Builds the per-cycle hazard list that FieldDescription carries.
 *
 * Two sources, deliberately different in how they decay:
 *   - static config geometry, which never expires (a detector outage must not delete the hole);
 *   - live tracks at Group::NEUTRAL, which are held for `tracked_hold_s` past going stale.
 *
 * Inflation happens here and only here: the emitted radius is the raw hazard radius plus our
 * robot's half-diagonal plus the source's margin, so no consumer re-derives clearance.
 */
class HazardAssembler {
   public:
    HazardAssembler(HazardAssemblerConfig config, std::vector<StaticHazardConfig> static_hazards,
                    std::shared_ptr<ClockInterface> clock);

    /** Assemble this cycle's hazards and stamp them into `field`. */
    void assemble(const RobotDescriptionsStamped &robots, FieldDescription &field);

   private:
    double our_half_diagonal(const RobotDescriptionsStamped &robots) const;

    HazardAssemblerConfig config_;
    std::vector<StaticHazardConfig> static_hazards_;
    std::shared_ptr<ClockInterface> clock_;
    std::shared_ptr<DiagnosticsModuleLogger> logger_;

    /** Last live sighting of each neutral track, so a stale one can be held rather than dropped. */
    struct HeldTrack {
        FieldHazard hazard;
        double last_live_s = 0.0;
    };
    std::vector<HeldTrack> held_tracks_;
    mutable double last_our_half_diagonal_m_ = 0.0;
};

}  // namespace auto_battlebot
