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
        double velocity_saturation_limit = 1.0;
        double zero_deadzone_percent = 0.0;
        double lifted_deadzone_percent = 0.0;
        bool reverse_linear = false;
        bool reverse_angular = false;
    };

    struct Channels {
        double first_output;
        double second_output;
    };

    virtual ~DriveProcessorInterface() = default;

    virtual Channels process(VelocityCommand command) const = 0;

    virtual BodyVelocity to_body_velocity(Channels channels) const = 0;
};

std::unique_ptr<DriveProcessorInterface> make_drive_processor(
    const std::string& type, const DriveProcessorInterface::Config& config,
    std::shared_ptr<DiagnosticsModuleLogger> logger);

}  // namespace auto_battlebot
