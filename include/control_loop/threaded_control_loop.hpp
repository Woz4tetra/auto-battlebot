#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include "control_loop/control_loop_interface.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"

namespace auto_battlebot {

/**
 * Runs control cycles on their own thread, paced against the wall clock at rate_hz.
 *
 * The command tracks the filter's evolving estimate instead of freezing between perception frames.
 * The thread owns the transmitter, so the Runner reaches it only through ControlLoop's request
 * setters.
 */
class ThreadedControlLoop : public ControlLoopInterface {
   public:
    ThreadedControlLoop(std::shared_ptr<ControlLoop> loop, double rate_hz,
                        double watchdog_timeout_ms);
    ~ThreadedControlLoop() override;

    void stop() override;

    /** The control thread reads input itself; the Runner's call must not touch the transmitter. */
    void pump_input() override {}

    bool is_healthy() const override;

   protected:
    void start_driver() override;

   private:
    void thread_main();

    std::chrono::microseconds period_;
    std::chrono::microseconds watchdog_timeout_;
    std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;
    std::thread thread_;
    std::atomic<bool> running_{false};
    std::atomic<int64_t> deadline_misses_{0};
    std::atomic<int64_t> cycles_{0};
};

}  // namespace auto_battlebot
