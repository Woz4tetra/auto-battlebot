#pragma once

#include <memory>

#include "control_loop/control_loop.hpp"

namespace auto_battlebot {

/**
 * Drives ControlLoop::run_cycle(). Two implementations, differing only in what schedules a cycle.
 *
 * SteppedControlLoop runs cycles synchronously from the perception tick, which keeps playback
 * reproducible and, at rate_hz = 0, reproduces the pre-Phase-2 pipeline exactly.
 * ThreadedControlLoop runs them on its own thread at rate_hz, which is the point of the change.
 */
class ControlLoopInterface {
   public:
    virtual ~ControlLoopInterface() = default;

    virtual void start() = 0;
    virtual void stop() = 0;

    /**
     * Read transmitter input on the caller's thread.
     *
     * Stepped drivers do the read here so the ordering matches the pre-Phase-2 tick. Threaded
     * drivers own the transmitter on their own thread and ignore this.
     */
    virtual void pump_input() {}

    /**
     * Run cycles until logical time reaches `until`. Stepped drivers only; threaded drivers pace
     * themselves and ignore this.
     */
    virtual void advance_to([[maybe_unused]] double until) {}

    /** False when the driver has missed its deadline by more than the configured watchdog. */
    virtual bool is_healthy() const { return true; }

    ControlLoop &loop() { return *loop_; }
    const ControlLoop &loop() const { return *loop_; }

   protected:
    explicit ControlLoopInterface(std::shared_ptr<ControlLoop> loop) : loop_(std::move(loop)) {}
    std::shared_ptr<ControlLoop> loop_;
};

}  // namespace auto_battlebot
