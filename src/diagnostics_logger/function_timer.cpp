#include "diagnostics_logger/function_timer.hpp"

namespace auto_battlebot
{
    FunctionTimer::FunctionTimer(
        std::shared_ptr<DiagnosticsModuleLogger> logger,
        const std::string &function_name,
        double warn_threshold_ms)
        : logger_(logger),
          function_name_(function_name),
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

        DiagnosticsData nested_data;
        nested_data["elapsed_ms"] = elapsed;

        DiagnosticsData data;
        data[function_name_] = nested_data;

        // Log as warning if threshold is set and exceeded, otherwise info
        if (warn_threshold_ms_ > 0 && elapsed > warn_threshold_ms_)
        {
            DiagnosticsData threshold_data = nested_data;
            threshold_data["threshold_ms"] = warn_threshold_ms_;
            data[function_name_] = threshold_data;
            logger_->warning(data);
        }
        else
        {
            logger_->info(data);
        }
    }

} // namespace auto_battlebot
