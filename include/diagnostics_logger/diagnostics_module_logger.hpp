#pragma once

#include <string>
#include <vector>
#include <set>
#include <algorithm>
#include <diagnostic_msgs/DiagnosticStatus.hxx>
#include "diagnostics_logger/diagnostics_utils.hpp"

namespace auto_battlebot
{
    /**
     * @brief A logger for a specific module that accumulates diagnostic information.
     *
     * This class should not be used directly; instead, use DiagnosticsLogger
     * to manage multiple module loggers.
     */
    class DiagnosticsModuleLogger
    {
    public:
        /**
         * @brief Construct a new DiagnosticsModuleLogger
         *
         * @param app_name Application name (used as hardware_id)
         * @param logger_name Name of this specific logger module
         */
        DiagnosticsModuleLogger(const std::string &app_name, const std::string &logger_name);

        /**
         * @brief Log diagnostic information with a specific level
         *
         * @param level Diagnostic level (OK, WARN, ERROR, STALE)
         * @param data Optional data dictionary
         * @param message Optional message string
         */
        void log(int8_t level, const DiagnosticsData &data = {}, const std::string &message = "");

        /**
         * @brief Log debug/info level diagnostic
         */
        void debug(const DiagnosticsData &data = {}, const std::string &message = "");

        /**
         * @brief Log info level diagnostic
         */
        void info(const DiagnosticsData &data = {}, const std::string &message = "");

        /**
         * @brief Log warning level diagnostic
         */
        void warning(const DiagnosticsData &data = {}, const std::string &message = "");

        /**
         * @brief Log error level diagnostic
         */
        void error(const DiagnosticsData &data = {}, const std::string &message = "");

        /**
         * @brief Clear all accumulated diagnostic information
         */
        void clear();

        /**
         * @brief Check if this logger has any status to report
         */
        bool has_status() const;

        /**
         * @brief Get the accumulated diagnostic status
         *
         * @return diagnostic_msgs::DiagnosticStatus
         */
        diagnostic_msgs::DiagnosticStatus get_status() const;

    private:
        std::string app_name_;
        std::string logger_name_;
        DiagnosticsData data_;
        int8_t level_;
        std::vector<std::string> messages_;
        std::set<std::string> logged_messages_; // To avoid duplicate messages
    };

} // namespace auto_battlebot
