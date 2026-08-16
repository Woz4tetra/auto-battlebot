#pragma once

#include <memory>

#include "control_loop/control_loop.hpp"

namespace auto_battlebot {

/**
 * Drives ControlLoop::run_cycle(). Two implementations, differing only in what schedules a cycle.
 *
 * SteppedControlLoop runs cycles synchronously from the perception tick, which keeps playback
 * reproducible. ThreadedControlLoop runs them on its own thread at rate_hz, so the command tracks
 * the filter's evolving estimate instead of freezing between perception frames.
 */
class ControlLoopInterface {
   public:
    virtual ~ControlLoopInterface() = default;

    /**
     * Initializes the transmitter, then starts the driver. Not virtual: the ordering is the point,
     * since a driver that begins cycling before the transmitter is up would send() into it.
     */
    bool start() {
        const bool ok = loop_->initialize();
        start_driver();
        return ok;
    }

    virtual void stop() = 0;

    /**
     * Read transmitter input on the caller's thread.
     *
     * Stepped drivers do the read here, so a single-threaded run reads the transmitter before
     * the camera grab. Threaded drivers own the transmitter on their own thread and ignore this.
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

    /** Begin cycling. Spawns the thread for threaded drivers, does nothing for stepped ones. */
    virtual void start_driver() = 0;

    std::shared_ptr<ControlLoop> loop_;
};

}  // namespace auto_battlebot
