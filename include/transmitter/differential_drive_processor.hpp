#pragma once

#include <memory>

#include "data_structures/velocity.hpp"
#include "diagnostics_logger/diagnostics_module_logger.hpp"
#include "transmitter/drive_processor_interface.hpp"

namespace auto_battlebot {

class DifferentialDriveProcessor : public DriveProcessorInterface {
   public:
    DifferentialDriveProcessor(const Config& config,
                               std::shared_ptr<DiagnosticsModuleLogger> logger);

    Channels process(VelocityCommand command) const override;

    BodyVelocity to_body_velocity(Channels channels) const override;

   private:
    Config config_;
    std::shared_ptr<DiagnosticsModuleLogger> logger_;
};

}  // namespace auto_battlebot
