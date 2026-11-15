#pragma once

#include <map>
#include <memory>
#include <mutex>
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
 *     // Initialize once at startup with one or more backends (e.g. UI, Foxglove)
 *     DiagnosticsLogger::initialize({ ui_backend, foxglove_backend });
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
    static void initialize(std::vector<std::shared_ptr<DiagnosticsBackend>> backends);

    static bool is_initialized();

    static std::shared_ptr<DiagnosticsModuleLogger> get_logger(const std::string &name);

    static void remove_logger(const std::string &name);

    static void publish();

   protected:
    /** Guards loggers_: the UI thread can create a logger while the loop thread publishes. */
    static std::mutex loggers_mutex_;
    static std::map<std::string, std::shared_ptr<DiagnosticsModuleLogger>> loggers_;
    static std::vector<std::shared_ptr<DiagnosticsBackend>> backends_;
    static bool initialized_;
    static bool test_mode_;
};

}  // namespace auto_battlebot
