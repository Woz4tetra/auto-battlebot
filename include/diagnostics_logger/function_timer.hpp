#pragma once

#include <chrono>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <string>

#include "diagnostics_logger/diagnostics_module_logger.hpp"

namespace auto_battlebot {
class FunctionTimer {
   public:
    FunctionTimer(std::shared_ptr<DiagnosticsModuleLogger> logger,
                  const std::string &timer_name = "timer",
                  std::optional<double> warn_threshold_ms = std::nullopt);

    ~FunctionTimer();

    FunctionTimer(const FunctionTimer &) = delete;
    FunctionTimer &operator=(const FunctionTimer &) = delete;

    FunctionTimer(FunctionTimer &&) = default;
    FunctionTimer &operator=(FunctionTimer &&) = default;

    void stop();

    double elapsed_ms() const;

    double elapsed_s() const;

   private:
    std::shared_ptr<DiagnosticsModuleLogger> logger_;
    std::string timer_name_;
    std::optional<double> warn_threshold_ms_;
    std::chrono::steady_clock::time_point start_time_;
    bool stopped_;

    void log_elapsed_time();
};

}  // namespace auto_battlebot
