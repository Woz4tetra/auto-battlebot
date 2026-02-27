#include "diagnostics_logger/diagnostics_logger.hpp"

namespace auto_battlebot {
std::map<std::string, std::shared_ptr<DiagnosticsModuleLogger>> DiagnosticsLogger::loggers_;
std::vector<std::shared_ptr<DiagnosticsBackend>> DiagnosticsLogger::backends_;
bool DiagnosticsLogger::initialized_ = false;
bool DiagnosticsLogger::test_mode_ = false;

void DiagnosticsLogger::initialize(std::vector<std::shared_ptr<DiagnosticsBackend>> backends) {
    if (initialized_) {
        throw std::runtime_error("DiagnosticsLogger already initialized");
    }
    backends_ = std::move(backends);
    initialized_ = true;
}

bool DiagnosticsLogger::is_initialized() { return initialized_; }

std::shared_ptr<DiagnosticsModuleLogger> DiagnosticsLogger::get_logger(const std::string &name) {
    auto it = loggers_.find(name);
    if (it != loggers_.end()) {
        return it->second;
    }
    auto logger = std::make_shared<DiagnosticsModuleLogger>(name);
    loggers_[name] = logger;
    return logger;
}

void DiagnosticsLogger::remove_logger(const std::string &name) {
    auto it = loggers_.find(name);
    if (it != loggers_.end()) {
        it->second->clear();
        loggers_.erase(it);
    }
}

void DiagnosticsLogger::publish() {
    if (!initialized_) {
        throw std::runtime_error("DiagnosticsLogger not initialized");
    }

    std::vector<DiagnosticStatusSnapshot> all_snapshots;
    for (auto &[name, logger] : loggers_) {
        if (!logger->has_status()) {
            continue;
        }
        std::vector<DiagnosticStatusSnapshot> snapshots = logger->get_snapshots();
        for (auto &s : snapshots) {
            all_snapshots.push_back(std::move(s));
        }
        logger->clear();
    }

    if (!all_snapshots.empty() && !test_mode_) {
        for (const auto &backend : backends_) {
            if (backend) {
                backend->receive(all_snapshots);
            }
        }
    }
}

}  // namespace auto_battlebot
