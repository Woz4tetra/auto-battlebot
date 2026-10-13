#include "diagnostics_logger/diagnostics_module_logger.hpp"

#include <algorithm>
#include <utility>

namespace auto_battlebot {
DiagnosticsModuleLogger::DiagnosticsModuleLogger(const std::string &logger_name, TimeSource now)
    : logger_name_(logger_name), now_(std::move(now)) {}

void DiagnosticsModuleLogger::log(int8_t level, const std::string &subsection_name,
                                  const DiagnosticsData &data) {
    log(level, subsection_name, data, "");
}
void DiagnosticsModuleLogger::log(int8_t level, const std::string &subsection_name,
                                  const std::string &message) {
    log(level, subsection_name, {}, message);
}
void DiagnosticsModuleLogger::log(int8_t level, const DiagnosticsData &data) {
    log(level, "", data, "");
}
void DiagnosticsModuleLogger::log(int8_t level, const std::string &message) {
    log(level, "", {}, message);
}
void DiagnosticsModuleLogger::log(int8_t level, const std::string &subsection_name,
                                  const DiagnosticsData &data, const std::string &message) {
    const auto now = now_();
    std::lock_guard<std::mutex> lock(mutex_);
    Section &section = sections_[subsection_name];
    // The first log of a window replaces whatever the section held from an earlier one.
    if (!section.fresh) {
        section = Section{};
        section.fresh = true;
    }
    section.level = std::max(level, section.level);
    section.last_logged = now;
    if (!message.empty()) section.messages.push_back(message);
    for (const auto &[key, value] : data) section.data[key] = value;
}

void DiagnosticsModuleLogger::debug(const DiagnosticsData &data) { debug("", data, ""); }
void DiagnosticsModuleLogger::debug(const std::string &message) { debug("", {}, message); }
void DiagnosticsModuleLogger::debug(const std::string &subsection_name,
                                    const DiagnosticsData &data) {
    debug(subsection_name, data, "");
}
void DiagnosticsModuleLogger::debug(const std::string &subsection_name,
                                    const std::string &message) {
    debug(subsection_name, {}, message);
}
void DiagnosticsModuleLogger::debug(const std::string &subsection_name, const DiagnosticsData &data,
                                    const std::string &message) {
    log(DiagnosticLevel::OK, subsection_name, data, message);
}

void DiagnosticsModuleLogger::info(const DiagnosticsData &data) { info("", data, ""); }
void DiagnosticsModuleLogger::info(const std::string &message) { info("", {}, message); }
void DiagnosticsModuleLogger::info(const std::string &subsection_name,
                                   const DiagnosticsData &data) {
    info(subsection_name, data, "");
}
void DiagnosticsModuleLogger::info(const std::string &subsection_name, const std::string &message) {
    info(subsection_name, {}, message);
}
void DiagnosticsModuleLogger::info(const std::string &subsection_name, const DiagnosticsData &data,
                                   const std::string &message) {
    log(DiagnosticLevel::OK, subsection_name, data, message);
}

void DiagnosticsModuleLogger::warning(const DiagnosticsData &data) { warning("", data, ""); }
void DiagnosticsModuleLogger::warning(const std::string &message) { warning("", {}, message); }
void DiagnosticsModuleLogger::warning(const std::string &subsection_name,
                                      const DiagnosticsData &data) {
    warning(subsection_name, data, "");
}
void DiagnosticsModuleLogger::warning(const std::string &subsection_name,
                                      const std::string &message) {
    warning(subsection_name, {}, message);
}
void DiagnosticsModuleLogger::warning(const std::string &subsection_name,
                                      const DiagnosticsData &data, const std::string &message) {
    log(DiagnosticLevel::WARN, subsection_name, data, message);
}

void DiagnosticsModuleLogger::error(const DiagnosticsData &data) { error("", data, ""); }
void DiagnosticsModuleLogger::error(const std::string &message) { error("", {}, message); }
void DiagnosticsModuleLogger::error(const std::string &subsection_name,
                                    const DiagnosticsData &data) {
    error(subsection_name, data, "");
}
void DiagnosticsModuleLogger::error(const std::string &subsection_name,
                                    const std::string &message) {
    error(subsection_name, {}, message);
}
void DiagnosticsModuleLogger::error(const std::string &subsection_name, const DiagnosticsData &data,
                                    const std::string &message) {
    log(DiagnosticLevel::ERROR, subsection_name, data, message);
}

void DiagnosticsModuleLogger::clear() {
    const auto now = now_();
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto it = sections_.begin(); it != sections_.end();) {
        if (now - it->second.last_logged >= SECTION_EXPIRY) {
            it = sections_.erase(it);
        } else {
            it->second.fresh = false;
            ++it;
        }
    }
}

bool DiagnosticsModuleLogger::has_status() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return std::any_of(sections_.begin(), sections_.end(),
                       [](const auto &entry) { return entry.second.fresh; });
}

std::vector<DiagnosticStatusSnapshot> DiagnosticsModuleLogger::get_snapshots() const {
    const auto now = now_();
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<DiagnosticStatusSnapshot> snapshots;
    for (const auto &[subsection_name, section] : sections_) {
        const auto age = now - section.last_logged;
        if (!section.fresh && age >= SECTION_EXPIRY) continue;
        std::string combined_message;
        for (size_t i = 0; i < section.messages.size(); ++i) {
            if (i > 0) combined_message += " | ";
            combined_message += section.messages[i];
        }
        DiagnosticStatusSnapshot snap;
        snap.name = logger_name_;
        snap.subsection = subsection_name;
        snap.level = section.level;
        snap.values = flatten_diagnostics_data(section.data);
        snap.message = combined_message;
        snap.age_ms = std::chrono::duration<double, std::milli>(age).count();
        snap.held = !section.fresh;
        snapshots.push_back(std::move(snap));
    }
    return snapshots;
}

std::string DiagnosticsModuleLogger::get_name() const { return logger_name_; }

}  // namespace auto_battlebot
