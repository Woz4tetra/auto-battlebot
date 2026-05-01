#pragma once

#include <memory>

#include "data_structures/velocity.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"
#include "diagnostics_logger/diagnostics_module_logger.hpp"

namespace auto_battlebot {

/**
 * Converts a normalized body-frame VelocityCommand into normalized trainer channel values for a
 * differential-drive robot, applying:
 *   1. Velocity saturation (angular priority, linear fills remaining headroom)
 *   2. Per-wheel lifted deadzone (applied in wheel space so steering at speed and spinning in
 *      place do not artificially inflate small inputs)
 *   3. Channel reversal to compensate for physical motor wiring
 *
 * All values are normalized to [-1, 1]. The caller is responsible for scaling to the actual
 * trainer range.
 */
class DifferentialDriveProcessor {
   public:
    struct Config {
        /** Combined output budget: |linear| + |angular| <= limit.
         *  Angular takes priority; linear fills remaining headroom.
         *  0 = disabled (each axis clamped independently to [-1, 1]). */
        double velocity_saturation_limit = 1.0;
        /** Input magnitude (%) below which output is forced to zero. */
        double zero_deadzone_percent = 0.0;
        /** Minimum non-zero output magnitude (%) after zero deadzone is exceeded. */
        double lifted_deadzone_percent = 0.0;
        bool reverse_linear = false;
        bool reverse_angular = false;
    };

    struct Result {
        double linear;   // final normalized linear channel value  [-1, 1]
        double angular;  // final normalized angular channel value [-1, 1]
    };

    DifferentialDriveProcessor(const Config& config,
                               std::shared_ptr<DiagnosticsModuleLogger> logger);

    Result process(VelocityCommand command) const;

   private:
    Config config_;
    std::shared_ptr<DiagnosticsModuleLogger> logger_;
};

}  // namespace auto_battlebot
