#pragma once

#include <chrono>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include "diagnostics_logger/diagnostics_module_logger.hpp"

namespace auto_battlebot {
class FunctionTimer {
   public:
    /**
     * @brief Construct a new FunctionTimer
     *
     * @param logger Diagnostics logger to log timing information to
     * @param timer_name Name of the timer. Useful if there are multiple for a single diagnostic
     * logger submodule
     * @param warn_threshold_ms Optional threshold in milliseconds. If execution time exceeds
     *                          this, log as warning instead of info
     */
    FunctionTimer(std::shared_ptr<DiagnosticsModuleLogger> logger,
                  const std::string &timer_name = "timer", double warn_threshold_ms = -1.0);

    ~FunctionTimer();

    FunctionTimer(const FunctionTimer &) = delete;
    FunctionTimer &operator=(const FunctionTimer &) = delete;

    FunctionTimer(FunctionTimer &&) = default;
    FunctionTimer &operator=(FunctionTimer &&) = default;

    /**
     * @brief Stop the timer and log the elapsed time
     *
     * Can be called manually to log before the timer goes out of scope.
     * Subsequent calls or destructor will not log again.
     */
    void stop();

    double elapsed_ms() const;

    double elapsed_s() const;

   private:
    std::shared_ptr<DiagnosticsModuleLogger> logger_;
    std::string timer_name_;
    double warn_threshold_ms_;
    std::chrono::steady_clock::time_point start_time_;
    bool stopped_;

    void log_elapsed_time();
};

}  // namespace auto_battlebot
