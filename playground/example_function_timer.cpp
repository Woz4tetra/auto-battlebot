#include "diagnostics_logger/function_timer.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"
#include <miniros/publisher.h>
#include <thread>
#include <chrono>

using namespace auto_battlebot;

void example_fast_function()
{
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

void example_slow_function()
{
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
}

int main()
{
    // Initialize diagnostics system
    auto publisher = std::make_shared<miniros::Publisher>("diagnostics", 10);
    DiagnosticsLogger::initialize("function_timer_example", publisher);

    // Get a logger for this module
    auto logger = DiagnosticsLogger::get_logger("timing_example");

    // Example 1: Basic usage - automatic timing
    {
        FunctionTimer timer(logger, "fast_function");
        example_fast_function();
    } // Logs when timer goes out of scope

    // Example 2: With warning threshold
    {
        FunctionTimer timer(logger, "slow_function", 50.0); // Warn if > 50ms
        example_slow_function();
    } // Logs as warning because it exceeds threshold

    // Example 3: Manual stop
    {
        FunctionTimer timer(logger, "manual_stop");
        example_fast_function();
        timer.stop(); // Manually log the time
        // Additional code here won't be timed
    }

    // Example 4: Check elapsed time without stopping
    {
        FunctionTimer timer(logger, "check_progress");
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        double elapsed = timer.elapsed_ms();
        // Use elapsed time for decisions...
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    } // Still logs total time on destruction

    // Publish all diagnostics
    DiagnosticsLogger::publish();

    return 0;
}
