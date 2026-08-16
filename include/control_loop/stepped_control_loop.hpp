#pragma once

#include <memory>

#include "control_loop/control_loop_interface.hpp"

namespace auto_battlebot {

/**
 * Runs control cycles synchronously from the perception tick against the logical clock.
 *
 * No thread, so playback and sim stay reproducible: the same recording produces the same cycle
 * sequence regardless of wall-clock speed or machine load.
 *
 * `rate_hz = 0` means exactly one cycle per advance_to() call, which reproduces the pre-Phase-2
 * pipeline: one filter/target/navigation/transmit pass per perception frame. This is deliberately
 * not the same as `rate_hz = 30`, where floor(dt / period) runs zero cycles on a frame that
 * arrives slightly early and would silently skip commands under frame jitter.
 */
class SteppedControlLoop : public ControlLoopInterface {
   public:
    SteppedControlLoop(std::shared_ptr<ControlLoop> loop, double rate_hz);

    void stop() override {}

    /** Stepped drivers read input inline so the ordering matches the pre-Phase-2 tick. */
    void pump_input() override { loop_->pump_input(); }

    void advance_to(double until) override;

   protected:
    void start_driver() override {}

   private:
    double period_ = 0.0;  // 0 = one cycle per call
    double last_advance_ = 0.0;
    bool started_ = false;
};

}  // namespace auto_battlebot
