#pragma once

#include <memory>
#include <utility>

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

    // The rest of the loop's surface, forwarded so callers deal with one object rather than
    // having to know which half of the split each call lives on. Every one of these is safe to
    // call from another thread while a driver is cycling.
    void submit_measurement(ControlMeasurement measurement) {
        loop_->submit_measurement(std::move(measurement));
    }
    bool take_init_button_press() { return loop_->take_init_button_press(); }
    ControlOutput latest_output() const { return loop_->latest_output(); }
    void request_filter_reinit(int opponent_count) { loop_->request_filter_reinit(opponent_count); }
    void set_autonomy_enabled(bool enabled) { loop_->set_autonomy_enabled(enabled); }
    TransmitterStatus transmitter_status() const { return loop_->transmitter_status(); }
    NavigationVisualization last_visualization() const { return loop_->last_visualization(); }

   protected:
    explicit ControlLoopInterface(std::shared_ptr<ControlLoop> loop) : loop_(std::move(loop)) {}

    /** Begin cycling. Spawns the thread for threaded drivers, does nothing for stepped ones. */
    virtual void start_driver() = 0;

    std::shared_ptr<ControlLoop> loop_;
};

}  // namespace auto_battlebot
