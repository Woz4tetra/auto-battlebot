#pragma once

#include <memory>
#include <string>

#include "data_structures/velocity.hpp"
#include "diagnostics_logger/diagnostics_module_logger.hpp"
#include "transmitter/drive_mixing.hpp"

namespace auto_battlebot {

/**
 * Converts a normalized body-frame VelocityCommand into the two normalized channel values the
 * transmitter writes, and converts a pair of channel values back into a body-frame command.
 *
 * Implementations share the saturation and deadzone stages and differ in what the two channels
 * carry: DifferentialDriveProcessor sends linear and angular for the radio-side mixer to combine,
 * TankDriveProcessor sends the left and right wheel commands directly.
 *
 * All values are normalized to [-1, 1]. The caller scales them to the actual trainer range.
 */
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
     * represents. Used to report the values on the wire back to the filter. The reversals are not
     * undone: they compensate robot wiring downstream of the channels, so the result stays in
     * channel terms.
     */
    virtual BodyVelocity to_body_velocity(Channels channels) const = 0;
};

/** Build the drive processor named by `type` ("TankDriveProcessor" or
 *  "DifferentialDriveProcessor"). Throws std::invalid_argument on an unknown name. */
std::unique_ptr<DriveProcessorInterface> make_drive_processor(
    const std::string& type, const DriveProcessorInterface::Config& config,
    std::shared_ptr<DiagnosticsModuleLogger> logger);

}  // namespace auto_battlebot
