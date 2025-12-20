#include "diagnostics_logger/diagnostics_module_logger.hpp"
#include <algorithm>

namespace auto_battlebot
{
    DiagnosticsModuleLogger::DiagnosticsModuleLogger(
        const std::string &app_name,
        const std::string &logger_name)
        : app_name_(app_name), logger_name_(logger_name), level_(diagnostic_msgs::DiagnosticStatus::OK)
    {
    }

    void DiagnosticsModuleLogger::log(
        int8_t level,
        const DiagnosticsData &data,
        const std::string &message)
    {
        // Update level to the highest severity
        level_ = std::max(level, level_);

        // Add message if not empty and not already logged
        if (!message.empty() && logged_messages_.find(message) == logged_messages_.end())
        {
            logged_messages_.insert(message);
            messages_.push_back(message);
        }

        // Merge data
        if (!data.empty())
        {
            for (const auto &[key, value] : data)
            {
                data_[key] = value;
            }
        }
    }

    void DiagnosticsModuleLogger::debug(const DiagnosticsData &data, const std::string &message)
    {
        log(diagnostic_msgs::DiagnosticStatus::OK, data, message);
    }

    void DiagnosticsModuleLogger::info(const DiagnosticsData &data, const std::string &message)
    {
        log(diagnostic_msgs::DiagnosticStatus::OK, data, message);
    }

    void DiagnosticsModuleLogger::warning(const DiagnosticsData &data, const std::string &message)
    {
        log(diagnostic_msgs::DiagnosticStatus::WARN, data, message);
    }

    void DiagnosticsModuleLogger::error(const DiagnosticsData &data, const std::string &message)
    {
        log(diagnostic_msgs::DiagnosticStatus::ERROR, data, message);
    }

    void DiagnosticsModuleLogger::clear()
    {
        level_ = diagnostic_msgs::DiagnosticStatus::OK;
        messages_.clear();
        data_.clear();
        logged_messages_.clear();
    }

    bool DiagnosticsModuleLogger::has_status() const
    {
        return !messages_.empty() || !data_.empty();
    }

    diagnostic_msgs::DiagnosticStatus DiagnosticsModuleLogger::get_status() const
    {
        // Join messages with " | " separator
        std::string combined_message;
        for (size_t i = 0; i < messages_.size(); ++i)
        {
            if (i > 0)
            {
                combined_message += " | ";
            }
            combined_message += messages_[i];
        }

        return dict_to_diagnostics(
            data_,
            level_,
            logger_name_,
            combined_message,
            app_name_);
    }

} // namespace auto_battlebot
