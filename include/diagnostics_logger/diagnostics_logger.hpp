#pragma once

#include <map>
#include <memory>
#include <string>
#include <stdexcept>
#include <miniros/publisher.h>
#include <std_msgs/Header.hxx>
#include <diagnostic_msgs/DiagnosticArray.hxx>
#include "diagnostics_logger/diagnostics_module_logger.hpp"

namespace auto_battlebot
{
    /**
     * @brief A singleton class that manages diagnostics loggers and publishes their statuses.
     *
     * Example usage:
     *     // Initialize once at startup
     *     DiagnosticsLogger::initialize("my_app", publisher);
     *
     *     // Get a logger for a module
     *     auto logger = DiagnosticsLogger::get_logger("module_name");
     *     logger->info({}, "This is an info message");
     *     logger->debug({{"sensor_name", "sensor_value"}});
     *     logger->error({{"error_code", 123}}, "Hardware has failed");
     *
     *     // Call periodically from a single place
     *     DiagnosticsLogger::publish();
     *
     * If a single logger sets multiple messages before being published, they are concatenated
     * together in the final output:
     *     logger->warning({}, "Low battery");
     *     logger->error({}, "Sensor disconnected");
     * Results in: "Low battery | Sensor disconnected"
     *
     * Nested dictionaries and arrays are flattened with keys joined by slashes:
     *     logger->error({{"motor", {{"temperatures", std::vector<int>{95, 94, 90}}, {"voltage", 12.5}}}});
     * Results in Foxglove:
     *     motor/temperature/0: 95
     *     motor/temperature/1: 94
     *     motor/temperature/2: 90
     *     motor/voltage: 12.5
     */
    class DiagnosticsLogger
    {
    public:
        /**
         * @brief Initialize the diagnostics logger system
         *
         * @param app_name Name of the application
         * @param publisher ROS publisher for DiagnosticArray messages
         */
        static void initialize(
            const std::string &app_name,
            std::shared_ptr<miniros::Publisher> publisher);

        /**
         * @brief Check if the diagnostics logger has been initialized
         */
        static bool is_initialized();

        /**
         * @brief Get or create a logger for a specific module
         *
         * @param name Name of the module
         * @return std::shared_ptr<DiagnosticsModuleLogger>
         */
        static std::shared_ptr<DiagnosticsModuleLogger> get_logger(const std::string &name);

        /**
         * @brief Remove a logger
         *
         * @param name Name of the module logger to remove
         */
        static void remove_logger(const std::string &name);

        /**
         * @brief Publish all accumulated diagnostics and clear loggers
         *
         * This should be called periodically from a single place at the top level
         * of the application.
         */
        static void publish();

    protected:
        static std::map<std::string, std::shared_ptr<DiagnosticsModuleLogger>> loggers_;
        static std::shared_ptr<miniros::Publisher> diagnostics_publisher_;
        static std::string app_name_;
    };

} // namespace auto_battlebot
