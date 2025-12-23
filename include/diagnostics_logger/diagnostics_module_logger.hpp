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
         * @param logger_name Name of this specific logger module
         */
        DiagnosticsModuleLogger(const std::string &logger_name);

        /**
         * @brief Log diagnostic information with a specific level
         *
         * @param level Diagnostic level (OK, WARN, ERROR, STALE)
         * @param data Optional data dictionary
         * @param message Optional message string
         * @param subsection_name Name of this subsection under the logger module name
         */
        void log(int8_t level, const std::string &subsection_name, const DiagnosticsData &data);
        void log(int8_t level, const std::string &subsection_name, const DiagnosticsData &data, const std::string &message);
        void log(int8_t level, const std::string &subsection_name, const std::string &message);
        void log(int8_t level, const DiagnosticsData &data);
        void log(int8_t level, const std::string &message);

        /**
         * @brief Log debug/info level diagnostic
         */
        void debug(const DiagnosticsData &data);
        void debug(const std::string &message);
        void debug(const std::string &subsection_name, const DiagnosticsData &data);
        void debug(const std::string &subsection_name, const DiagnosticsData &data, const std::string &message);
        void debug(const std::string &subsection_name, const std::string &message);

        /**
         * @brief Log info level diagnostic
         */
        void info(const DiagnosticsData &data);
        void info(const std::string &message);
        void info(const std::string &subsection_name, const DiagnosticsData &data);
        void info(const std::string &subsection_name, const DiagnosticsData &data, const std::string &message);
        void info(const std::string &subsection_name, const std::string &message);

        /**
         * @brief Log warning level diagnostic
         */
        void warning(const DiagnosticsData &data);
        void warning(const std::string &message);
        void warning(const std::string &subsection_name, const DiagnosticsData &data);
        void warning(const std::string &subsection_name, const DiagnosticsData &data, const std::string &message);
        void warning(const std::string &subsection_name, const std::string &message);

        /**
         * @brief Log error level diagnostic
         */
        void error(const DiagnosticsData &data);
        void error(const std::string &message);
        void error(const std::string &subsection_name, const DiagnosticsData &data);
        void error(const std::string &subsection_name, const DiagnosticsData &data, const std::string &message);
        void error(const std::string &subsection_name, const std::string &message);

        /**
         * @brief Clear all accumulated diagnostic information
         */
        void clear();

        /**
         * @brief Check if this logger has any status to report
         */
        bool has_status() const;

        /**
         * @brief Get logger name
         *
         * @return std::string
         */
        std::string get_name() const;

        /**
         * @brief Get the accumulated diagnostic status
         *
         * @return std::vector<diagnostic_msgs::DiagnosticStatus>
         */
        std::vector<diagnostic_msgs::DiagnosticStatus> get_status() const;

    private:
        std::string logger_name_;
        std::map<std::string, DiagnosticsData> data_;
        int8_t level_;
        std::vector<std::string> messages_;
        std::set<std::string> logged_messages_; // To avoid duplicate messages
    };

} // namespace auto_battlebot
