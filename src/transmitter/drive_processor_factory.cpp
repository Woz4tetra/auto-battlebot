#include <spdlog/spdlog.h>

#include <stdexcept>
#include <utility>

#include "transmitter/differential_drive_processor.hpp"
#include "transmitter/drive_processor_interface.hpp"
#include "transmitter/tank_drive_processor.hpp"

namespace auto_battlebot {

std::unique_ptr<DriveProcessorInterface> make_drive_processor(
    const std::string& type, const DriveProcessorInterface::Config& config,
    std::shared_ptr<DiagnosticsModuleLogger> logger) {
    spdlog::info("Selected {} for drive processor", type);
    if (type == "TankDriveProcessor") {
        return std::make_unique<TankDriveProcessor>(config, std::move(logger));
    }
    if (type == "DifferentialDriveProcessor") {
        return std::make_unique<DifferentialDriveProcessor>(config, std::move(logger));
    }
    throw std::invalid_argument("Failed to load drive processor of type " + type);
}

}  // namespace auto_battlebot
