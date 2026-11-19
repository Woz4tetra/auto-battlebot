#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include "control_loop/control_loop_interface.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"

namespace auto_battlebot {

class ThreadedControlLoop : public ControlLoopInterface {
   public:
    ThreadedControlLoop(std::shared_ptr<ControlLoop> loop, double rate_hz,
                        double watchdog_timeout_ms);
    ~ThreadedControlLoop() override;

    void stop() override;

    void pump_input() override {}

    bool is_healthy() const override;

   protected:
    void start_driver() override;

   private:
    void thread_main();
    void join_thread();

    std::chrono::microseconds period_;
    std::chrono::microseconds watchdog_timeout_;
    std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;
    std::thread thread_;
    std::atomic<bool> running_{false};
    std::atomic<int64_t> deadline_misses_{0};
    std::atomic<int64_t> cycles_{0};
};

}  // namespace auto_battlebot
