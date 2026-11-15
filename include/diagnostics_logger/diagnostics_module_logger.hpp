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
/**
 * @brief A logger for a specific module that accumulates diagnostic information.
 *
 * This class should not be used directly; instead, use DiagnosticsLogger
 * to manage multiple module loggers.
 *
 * Subsections are sticky. Each publish window (the span between two clear() calls) starts a
 * subsection over on its first log, then merges later logs into it. A subsection not logged in
 * the current window keeps its last values and is reported as held, with its age, until
 * SECTION_EXPIRY passes without a log. So a module that logs one subsection per send and another
 * only on change publishes both in every message instead of alternating between them.
 */
class DiagnosticsModuleLogger {
   public:
    using TimeSource = std::function<std::chrono::steady_clock::time_point()>;

    /** A held subsection drops out of the snapshots once it goes this long without a log. */
    static constexpr std::chrono::seconds SECTION_EXPIRY{10};

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
    // Guards sections_. A module logger has two writers: the control thread logging, and the
    // Runner thread clearing during DiagnosticsLogger::publish().
    mutable std::mutex mutex_;
    std::map<std::string, Section> sections_;
};

}  // namespace auto_battlebot
