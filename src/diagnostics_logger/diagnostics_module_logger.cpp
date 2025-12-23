#include "diagnostics_logger/diagnostics_module_logger.hpp"

namespace auto_battlebot
{
    DiagnosticsModuleLogger::DiagnosticsModuleLogger(const std::string &logger_name) : logger_name_(logger_name),
                                                                                       level_(diagnostic_msgs::DiagnosticStatus::OK),
                                                                                       data_(std::map<std::string, DiagnosticsData>{})
    {
    }

    void DiagnosticsModuleLogger::log(
        int8_t level,
        const std::string &subsection_name,
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

        // Get or create subsection data map
        if (data_.find(subsection_name) == data_.end())
        {
            data_[subsection_name] = DiagnosticsData{};
        }

        // Combine diagnostic data with existing data in the subsection
        for (const auto &[key, value] : data)
        {
            data_[subsection_name][key] = value;
        }
    }

    void DiagnosticsModuleLogger::debug(const DiagnosticsData &data)
    {
        debug("", data, "");
    }

    void DiagnosticsModuleLogger::debug(const std::string &message)
    {
        debug("", {}, message);
    }

    void DiagnosticsModuleLogger::debug(const std::string &subsection_name, const DiagnosticsData &data)
    {
        debug(subsection_name, {}, "");
    }

    void DiagnosticsModuleLogger::debug(const std::string &subsection_name, const std::string &message)
    {
        debug(subsection_name, {}, message);
    }

    void DiagnosticsModuleLogger::debug(const std::string &subsection_name, const DiagnosticsData &data, const std::string &message)
    {
        log(diagnostic_msgs::DiagnosticStatus::OK, subsection_name, data, message);
    }

    void DiagnosticsModuleLogger::info(const DiagnosticsData &data)
    {
        info("", data, "");
    }

    void DiagnosticsModuleLogger::info(const std::string &message)
    {
        info("", {}, message);
    }

    void DiagnosticsModuleLogger::info(const std::string &subsection_name, const DiagnosticsData &data)
    {
        info(subsection_name, {}, "");
    }

    void DiagnosticsModuleLogger::info(const std::string &subsection_name, const std::string &message)
    {
        error(subsection_name, {}, message);
    }

    void DiagnosticsModuleLogger::info(const std::string &subsection_name, const DiagnosticsData &data, const std::string &message)
    {
        log(diagnostic_msgs::DiagnosticStatus::OK, subsection_name, data, message);
    }

    void DiagnosticsModuleLogger::warning(const DiagnosticsData &data)
    {
        warning("", data, "");
    }

    void DiagnosticsModuleLogger::warning(const std::string &message)
    {
        warning("", {}, message);
    }

    void DiagnosticsModuleLogger::warning(const std::string &subsection_name, const DiagnosticsData &data)
    {
        warning(subsection_name, {}, "");
    }
    void DiagnosticsModuleLogger::warning(const std::string &subsection_name, const std::string &message)
    {
        error(subsection_name, {}, message);
    }
    void DiagnosticsModuleLogger::warning(const std::string &subsection_name, const DiagnosticsData &data, const std::string &message)
    {
        log(diagnostic_msgs::DiagnosticStatus::WARN, subsection_name, data, message);
    }

    void DiagnosticsModuleLogger::error(const DiagnosticsData &data)
    {
        error("", data, "");
    }

    void DiagnosticsModuleLogger::error(const std::string &message)
    {
        error("", {}, message);
    }

    void DiagnosticsModuleLogger::error(const std::string &subsection_name, const DiagnosticsData &data)
    {
        error(subsection_name, {}, "");
    }
    void DiagnosticsModuleLogger::error(const std::string &subsection_name, const std::string &message)
    {
        error(subsection_name, {}, message);
    }
    void DiagnosticsModuleLogger::error(const std::string &subsection_name, const DiagnosticsData &data, const std::string &message)
    {
        log(diagnostic_msgs::DiagnosticStatus::ERROR, subsection_name, data, message);
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

    std::vector<diagnostic_msgs::DiagnosticStatus> DiagnosticsModuleLogger::get_status() const
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

        std::vector<diagnostic_msgs::DiagnosticStatus> diagnostics;
        for (const auto &[subsection_name, data_map] : data_)
        {

            diagnostics.push_back(dict_to_diagnostics(
                data_map,
                level_,
                subsection_name,
                combined_message,
                logger_name_));
        }
        return diagnostics;
    }

    std::string DiagnosticsModuleLogger::get_name() const
    {
        return logger_name_;
    }

} // namespace auto_battlebot
