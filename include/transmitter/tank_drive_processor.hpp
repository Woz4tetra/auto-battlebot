#pragma once

#include <memory>

#include "data_structures/velocity.hpp"
#include "diagnostics_logger/diagnostics_module_logger.hpp"
#include "transmitter/drive_processor_interface.hpp"

namespace auto_battlebot {

/**
 * Emits left and right motor channels for a robot that expects per-wheel commands, with no
 * radio-side mixer between the channels and the motors. Applies:
 *   1. Velocity saturation (angular priority, linear fills remaining headroom)
 *   2. Channel reversal to compensate for physical motor wiring
 *   3. Per-wheel lifted deadzone
 *
 * The stages match DifferentialDriveProcessor up to the mixing: this one stops at wheel space
 * instead of converting back to body axes.
 */
class TankDriveProcessor : public DriveProcessorInterface {
   public:
    TankDriveProcessor(const Config& config, std::shared_ptr<DiagnosticsModuleLogger> logger);

    /** channel_a is the left wheel command, channel_b is the right wheel command. */
    Channels process(VelocityCommand command) const override;

    /** Inverse of the differential mixing: linear is the wheel average, angular is half the
     *  difference. */
    BodyVelocity to_body_velocity(Channels channels) const override;

   private:
    Config config_;
    std::shared_ptr<DiagnosticsModuleLogger> logger_;
};

}  // namespace auto_battlebot
