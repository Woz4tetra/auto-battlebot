#pragma once

#include <memory>
#include <string>

#include "data_structures/velocity.hpp"
#include "diagnostics_logger/diagnostics_module_logger.hpp"
#include "transmitter/drive_mixing.hpp"

namespace auto_battlebot {

class DriveProcessorInterface {
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

    /** The two normalized channel values. channel_a is linear (differential) or the left wheel
     *  (tank); channel_b is angular (differential) or the right wheel (tank). */
    struct Channels {
        double channel_a;
        double channel_b;
    };

    virtual ~DriveProcessorInterface() = default;

    virtual Channels process(VelocityCommand command) const = 0;

    /**
     * Undo the mixing in process() to recover the body-frame command a pair of channel values
     * represents. Used to report the values on the wire back to the filter, so the reversals are
     * undone too: the result is in the same body frame as the command process() took, which is
     * the frame the filter's plant prediction and the RL policy's observation expect.
     */
    virtual BodyVelocity to_body_velocity(Channels channels) const = 0;
};

std::unique_ptr<DriveProcessorInterface> make_drive_processor(
    const std::string& type, const DriveProcessorInterface::Config& config,
    std::shared_ptr<DiagnosticsModuleLogger> logger);

}  // namespace auto_battlebot
