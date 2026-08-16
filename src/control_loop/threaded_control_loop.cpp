#include "control_loop/threaded_control_loop.hpp"

#include <utility>

namespace auto_battlebot {

ThreadedControlLoop::ThreadedControlLoop(std::shared_ptr<ControlLoop> loop, double rate_hz,
                                         double watchdog_timeout_ms)
    : ControlLoopInterface(std::move(loop)),
      period_(std::chrono::microseconds(
          static_cast<int64_t>(rate_hz > 0.0 ? 1000000.0 / rate_hz : 4000.0))),
      watchdog_timeout_(
          std::chrono::microseconds(static_cast<int64_t>(watchdog_timeout_ms * 1000))),
      diagnostics_logger_(DiagnosticsLogger::get_logger("control_loop")) {}

ThreadedControlLoop::~ThreadedControlLoop() { stop(); }

void ThreadedControlLoop::start() {
    if (running_.exchange(true)) return;
    thread_ = std::thread(&ThreadedControlLoop::thread_main, this);
}

void ThreadedControlLoop::stop() {
    if (!running_.exchange(false)) return;
    if (thread_.joinable()) thread_.join();
}

bool ThreadedControlLoop::is_healthy() const {
    if (!running_.load()) return true;  // not started yet, or deliberately stopped
    if (cycles_.load() == 0) return true;
    const auto age = std::chrono::steady_clock::now() - loop_->last_cycle_time();
    return age <= watchdog_timeout_;
}

void ThreadedControlLoop::thread_main() {
    auto next = std::chrono::steady_clock::now();
    int64_t report_cycles = 0;
    int64_t report_misses = 0;
    double busy_us_sum = 0.0;
    double busy_us_max = 0.0;
    auto last_report = std::chrono::steady_clock::now();

    while (running_.load()) {
        const auto cycle_start = std::chrono::steady_clock::now();
        loop_->pump_input();
        loop_->run_cycle();
        const auto cycle_end = std::chrono::steady_clock::now();

        const double busy_us =
            std::chrono::duration<double, std::micro>(cycle_end - cycle_start).count();
        busy_us_sum += busy_us;
        if (busy_us > busy_us_max) busy_us_max = busy_us;
        cycles_.fetch_add(1);
        report_cycles++;

        next += period_;
        if (cycle_end > next) {
            // Overran the period. Count it and resync rather than accumulating debt, which would
            // otherwise make the loop sprint to catch up after any one slow cycle.
            deadline_misses_.fetch_add(1);
            report_misses++;
            next = cycle_end;
        } else {
            std::this_thread::sleep_until(next);
        }

        if (cycle_end - last_report >= std::chrono::seconds(1)) {
            const double elapsed_s = std::chrono::duration<double>(cycle_end - last_report).count();
            diagnostics_logger_->debug(
                "control_loop",
                {{"rate_hz", report_cycles / elapsed_s},
                 {"deadline_misses", static_cast<int>(report_misses)},
                 {"cycle_us_avg", report_cycles > 0 ? busy_us_sum / report_cycles : 0.0},
                 {"cycle_us_max", busy_us_max}});
            last_report = cycle_end;
            report_cycles = 0;
            report_misses = 0;
            busy_us_sum = 0.0;
            busy_us_max = 0.0;
        }
    }
}

}  // namespace auto_battlebot
