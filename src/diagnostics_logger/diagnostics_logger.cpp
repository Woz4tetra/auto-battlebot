#include "diagnostics_logger/diagnostics_logger.hpp"
#include <std_msgs/Header.hxx>
#include <stdexcept>

namespace auto_battlebot
{
    // Static member definitions
    std::map<std::string, std::shared_ptr<DiagnosticsModuleLogger>> DiagnosticsLogger::loggers_;
    std::shared_ptr<miniros::Publisher> DiagnosticsLogger::diagnostics_publisher_;
    std::string DiagnosticsLogger::app_name_;

    void DiagnosticsLogger::initialize(
        const std::string &app_name,
        std::shared_ptr<miniros::Publisher> publisher)
    {
        if (diagnostics_publisher_ != nullptr)
        {
            throw std::runtime_error("DiagnosticsLogger already initialized");
        }
        app_name_ = app_name;
        diagnostics_publisher_ = publisher;
    }

    bool DiagnosticsLogger::is_initialized()
    {
        return diagnostics_publisher_ != nullptr;
    }

    std::shared_ptr<DiagnosticsModuleLogger> DiagnosticsLogger::get_logger(const std::string &name)
    {
        auto it = loggers_.find(name);
        if (it != loggers_.end())
        {
            return it->second;
        }

        auto logger = std::make_shared<DiagnosticsModuleLogger>(app_name_, name);
        loggers_[name] = logger;
        return logger;
    }

    void DiagnosticsLogger::remove_logger(const std::string &name)
    {
        auto it = loggers_.find(name);
        if (it != loggers_.end())
        {
            it->second->clear();
            loggers_.erase(it);
        }
    }

    void DiagnosticsLogger::publish()
    {
        if (diagnostics_publisher_ == nullptr)
        {
            throw std::runtime_error("DiagnosticsLogger not initialized");
        }

        std::vector<diagnostic_msgs::DiagnosticStatus> statuses;

        // Collect statuses from all loggers
        for (auto &[name, logger] : loggers_)
        {
            if (!logger->has_status())
            {
                continue;
            }
            statuses.push_back(logger->get_status());
            logger->clear();
        }

        // Only publish if we have statuses
        if (statuses.empty())
        {
            return;
        }

        // Create DiagnosticArray message
        diagnostic_msgs::DiagnosticArray diagnostics;

        // Set header with current time
        diagnostics.header.stamp = miniros::Time::now();
        diagnostics.header.frame_id = "";

        // Add all statuses
        diagnostics.status = statuses;

        // Publish
        diagnostics_publisher_->publish(diagnostics);
    }

} // namespace auto_battlebot
