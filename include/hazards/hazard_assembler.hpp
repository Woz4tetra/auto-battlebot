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
    double static_margin_m = 0.10;
    double tracked_margin_m = 0.05;
    double tracked_hold_s = 0.75;
    double hard_margin_m = 0.02;
    double our_half_diagonal_fallback_m = 0.11;
};

class HazardAssembler {
   public:
    HazardAssembler(HazardAssemblerConfig config, std::vector<StaticHazardConfig> static_hazards,
                    std::shared_ptr<ClockInterface> clock);

    void assemble(const RobotDescriptionsStamped &robots, FieldDescription &field);

   private:
    double our_half_diagonal(const RobotDescriptionsStamped &robots) const;

    HazardAssemblerConfig config_;
    std::vector<StaticHazardConfig> static_hazards_;
    std::shared_ptr<ClockInterface> clock_;
    std::shared_ptr<DiagnosticsModuleLogger> logger_;

    struct LastNeutralSighting {
        FieldHazard hazard;
        double last_live_s = 0.0;
    };
    std::vector<LastNeutralSighting> last_neutral_sightings_;
    mutable double last_our_half_diagonal_m_ = 0.0;
};

}  // namespace auto_battlebot
