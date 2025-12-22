#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <sstream>
#include <iomanip>
#include <diagnostic_msgs/DiagnosticStatus.hxx>
#include "diagnostics_logger/diagnostics_module_logger.hpp"

namespace auto_battlebot
{
    /**
     * @brief RAII timer class that measures function execution time and logs to diagnostics.
     *
     * Example usage:
     *     auto logger = DiagnosticsLogger::get_logger("my_module");
     *
     *     // Basic usage - logs on destruction
     *     {
     *         FunctionTimer timer(logger, "process_frame");
     *         // ... do work ...
     *     } // Automatically logs elapsed time
     *
     *     // Manual stop and log
     *     FunctionTimer timer(logger, "computation");
     *     // ... do work ...
     *     timer.stop(); // Logs elapsed time
     *
     *     // With warning threshold
     *     FunctionTimer timer(logger, "critical_task", 50.0); // Warn if > 50ms
     *
     *     // Check elapsed time without stopping
     *     FunctionTimer timer(logger, "task");
     *     // ... do work ...
     *     double elapsed = timer.elapsed_ms();
     */
    class FunctionTimer
    {
    public:
        /**
         * @brief Construct a new FunctionTimer
         *
         * @param logger Diagnostics logger to log timing information to
         * @param function_name Name of the function being timed
         * @param warn_threshold_ms Optional threshold in milliseconds. If execution time exceeds
         *                          this, log as warning instead of info
         */
        FunctionTimer(
            std::shared_ptr<DiagnosticsModuleLogger> logger,
            const std::string &function_name,
            double warn_threshold_ms = -1.0);

        /**
         * @brief Destructor - automatically logs if not already stopped
         */
        ~FunctionTimer();

        // Disable copying
        FunctionTimer(const FunctionTimer &) = delete;
        FunctionTimer &operator=(const FunctionTimer &) = delete;

        // Allow moving
        FunctionTimer(FunctionTimer &&) = default;
        FunctionTimer &operator=(FunctionTimer &&) = default;

        /**
         * @brief Stop the timer and log the elapsed time
         *
         * Can be called manually to log before the timer goes out of scope.
         * Subsequent calls or destructor will not log again.
         */
        void stop();

        /**
         * @brief Get elapsed time in milliseconds without stopping the timer
         *
         * @return double Elapsed time in milliseconds
         */
        double elapsed_ms() const;

        /**
         * @brief Get elapsed time in seconds without stopping the timer
         *
         * @return double Elapsed time in seconds
         */
        double elapsed_s() const;

    private:
        std::shared_ptr<DiagnosticsModuleLogger> logger_;
        std::string function_name_;
        double warn_threshold_ms_;
        std::chrono::steady_clock::time_point start_time_;
        bool stopped_;

        /**
         * @brief Internal method to log the timing information
         */
        void log_elapsed_time();
    };

} // namespace auto_battlebot
