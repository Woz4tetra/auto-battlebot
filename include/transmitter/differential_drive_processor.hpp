#pragma once

#include <memory>

#include "data_structures/velocity.hpp"
#include "diagnostics_logger/diagnostics_module_logger.hpp"
#include "transmitter/drive_processor_interface.hpp"

namespace auto_battlebot {

/**
 * Emits linear and angular channels for a robot whose radio-side mixer combines them into
 * per-wheel motor outputs. Applies:
 *   1. Velocity saturation (angular priority, linear fills remaining headroom)
 *   2. Per-wheel lifted deadzone (applied in wheel space so steering at speed and spinning in
 *      place do not artificially inflate small inputs)
 *   3. Channel reversal to compensate for physical motor wiring
 */
class DifferentialDriveProcessor : public DriveProcessorInterface {
   public:
    DifferentialDriveProcessor(const Config& config,
                               std::shared_ptr<DiagnosticsModuleLogger> logger);

    /** channel_a is the linear command, channel_b is the angular command. */
    Channels process(VelocityCommand command) const override;

    /** The channels are already body-frame axes, so this is a pass-through. */
    BodyVelocity to_body_velocity(Channels channels) const override;

   private:
    Config config_;
    std::shared_ptr<DiagnosticsModuleLogger> logger_;
};

}  // namespace auto_battlebot
