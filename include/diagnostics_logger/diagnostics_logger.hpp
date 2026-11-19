#pragma once

#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "diagnostics_logger/diagnostics_backend_interface.hpp"
#include "diagnostics_logger/diagnostics_module_logger.hpp"

namespace auto_battlebot {
class DiagnosticsLogger {
   public:
    static void initialize(std::vector<std::shared_ptr<DiagnosticsBackend>> backends);

    static bool is_initialized();

    static std::shared_ptr<DiagnosticsModuleLogger> get_logger(const std::string &name);

    static void remove_logger(const std::string &name);

    static void publish();

   protected:
    static std::mutex loggers_mutex_;
    static std::map<std::string, std::shared_ptr<DiagnosticsModuleLogger>> loggers_;
    static std::vector<std::shared_ptr<DiagnosticsBackend>> backends_;
    static bool initialized_;
    static bool test_mode_;
};

}  // namespace auto_battlebot
