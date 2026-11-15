#pragma once

#include <chrono>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "diagnostics_logger/diagnostics_backend_interface.hpp"
#include "diagnostics_logger/diagnostics_utils.hpp"

namespace auto_battlebot {
class DiagnosticsModuleLogger {
   public:
    using TimeSource = std::function<std::chrono::steady_clock::time_point()>;

    static constexpr std::chrono::seconds HELD_SECTION_EXPIRY{10};

    explicit DiagnosticsModuleLogger(const std::string &logger_name,
                                     TimeSource now = std::chrono::steady_clock::now);

    void log(int8_t level, const std::string &subsection_name, const DiagnosticsData &data);
    void log(int8_t level, const std::string &subsection_name, const DiagnosticsData &data,
             const std::string &message);
    void log(int8_t level, const std::string &subsection_name, const std::string &message);
    void log(int8_t level, const DiagnosticsData &data);
    void log(int8_t level, const std::string &message);

    void debug(const DiagnosticsData &data);
    void debug(const std::string &message);
    void debug(const std::string &subsection_name, const DiagnosticsData &data);
    void debug(const std::string &subsection_name, const DiagnosticsData &data,
               const std::string &message);
    void debug(const std::string &subsection_name, const std::string &message);

    void info(const DiagnosticsData &data);
    void info(const std::string &message);
    void info(const std::string &subsection_name, const DiagnosticsData &data);
    void info(const std::string &subsection_name, const DiagnosticsData &data,
              const std::string &message);
    void info(const std::string &subsection_name, const std::string &message);

    void warning(const DiagnosticsData &data);
    void warning(const std::string &message);
    void warning(const std::string &subsection_name, const DiagnosticsData &data);
    void warning(const std::string &subsection_name, const DiagnosticsData &data,
                 const std::string &message);
    void warning(const std::string &subsection_name, const std::string &message);

    void error(const DiagnosticsData &data);
    void error(const std::string &message);
    void error(const std::string &subsection_name, const DiagnosticsData &data);
    void error(const std::string &subsection_name, const DiagnosticsData &data,
               const std::string &message);
    void error(const std::string &subsection_name, const std::string &message);

    void clear();

    bool has_status() const;

    std::string get_name() const;

    std::vector<DiagnosticStatusSnapshot> get_snapshots() const;

   private:
    struct Section {
        DiagnosticsData data;
        std::vector<std::string> messages;
        int8_t level = DiagnosticLevel::OK;
        std::chrono::steady_clock::time_point last_logged;
        /** Logged in the current publish window. */
        bool fresh = false;
    };

    std::string logger_name_;
    TimeSource now_;
    mutable std::mutex mutex_;
    std::map<std::string, Section> sections_;
};

}  // namespace auto_battlebot
