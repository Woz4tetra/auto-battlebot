#include "diagnostics_logger/function_timer.hpp"

namespace auto_battlebot
{
    FunctionTimer::FunctionTimer(
        std::shared_ptr<DiagnosticsModuleLogger> logger,
        const std::string &function_name,
        double warn_threshold_ms)
        : logger_(logger),
          timer_name_(function_name),
          warn_threshold_ms_(warn_threshold_ms),
          start_time_(std::chrono::steady_clock::now()),
          stopped_(false)
    {
    }

    FunctionTimer::~FunctionTimer()
    {
        if (!stopped_)
        {
            log_elapsed_time();
        }
    }

    void FunctionTimer::stop()
    {
        if (!stopped_)
        {
            log_elapsed_time();
            stopped_ = true;
        }
    }

    double FunctionTimer::elapsed_ms() const
    {
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - start_time_);
        return duration.count() / 1000.0;
    }

    double FunctionTimer::elapsed_s() const
    {
        return elapsed_ms() / 1000.0;
    }

    void FunctionTimer::log_elapsed_time()
    {
        double elapsed = elapsed_ms();

        DiagnosticsData data;
        data["elapsed_ms"] = elapsed;

        // Log as warning if threshold is set and exceeded, otherwise info
        if (warn_threshold_ms_ > 0 && elapsed > warn_threshold_ms_)
        {
            data["threshold_ms"] = warn_threshold_ms_;
            logger_->warning(timer_name_, data, "");
        }
        else
        {
            logger_->info(timer_name_, data, "");
        }
    }

} // namespace auto_battlebot
