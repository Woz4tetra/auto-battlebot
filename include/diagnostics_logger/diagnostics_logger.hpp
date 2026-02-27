#pragma once

#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "diagnostics_logger/diagnostics_backend_interface.hpp"
#include "diagnostics_logger/diagnostics_module_logger.hpp"

namespace auto_battlebot {
/**
 * @brief A singleton class that manages diagnostics loggers and forwards to backends.
 *
 * Example usage:
 *     // Initialize once at startup with one or more backends (e.g. UI, ROS)
 *     DiagnosticsLogger::initialize({ ui_backend, ros_backend });
 *
 *     // Get a logger for a module
 *     auto logger = DiagnosticsLogger::get_logger("module_name");
 *     logger->info({}, "This is an info message");
 *     logger->debug({{"sensor_name", "sensor_value"}});
 *
 *     // Call periodically from a single place
 *     DiagnosticsLogger::publish();
 *
 * If a single logger sets multiple messages before being published, they are concatenated
 * together in the final output.
 *
 * Nested dictionaries and arrays are flattened with keys joined by slashes.
 */
class DiagnosticsLogger {
   public:
    /**
     * @brief Initialize the diagnostics logger system with backends
     *
     * @param backends List of backends to receive diagnostic snapshots (e.g. UI, ROS)
     */
    static void initialize(std::vector<std::shared_ptr<DiagnosticsBackend>> backends);

    /**
     * @brief Check if the diagnostics logger has been initialized
     */
    static bool is_initialized();

    /**
     * @brief Get or create a logger for a specific module
     */
    static std::shared_ptr<DiagnosticsModuleLogger> get_logger(const std::string &name);

    /**
     * @brief Remove a logger
     */
    static void remove_logger(const std::string &name);

    /**
     * @brief Gather snapshots from all loggers, clear them, and pass to each backend
     */
    static void publish();

   protected:
    static std::map<std::string, std::shared_ptr<DiagnosticsModuleLogger>> loggers_;
    static std::vector<std::shared_ptr<DiagnosticsBackend>> backends_;
    static bool initialized_;
    static bool test_mode_;  // When true, skip calling backends (for unit tests)
};

}  // namespace auto_battlebot
