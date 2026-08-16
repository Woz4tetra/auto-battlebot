#include "control_loop/stepped_control_loop.hpp"

#include <cmath>
#include <utility>

namespace auto_battlebot {

SteppedControlLoop::SteppedControlLoop(std::shared_ptr<ControlLoop> loop, double rate_hz)
    : ControlLoopInterface(std::move(loop)), period_(rate_hz > 0.0 ? 1.0 / rate_hz : 0.0) {}

void SteppedControlLoop::advance_to(double until) {
    if (period_ <= 0.0) {
        // Pre-Phase-2 behavior: one cycle per perception frame, independent of frame timing.
        loop_->run_cycle();
        return;
    }

    if (!started_) {
        started_ = true;
        last_advance_ = until;
        loop_->run_cycle();
        return;
    }

    const double dt = until - last_advance_;
    if (dt <= 0.0) return;  // non-advancing clock: nothing to catch up on

    // Cap the catch-up so a long stall (a stopped debugger, a slow frame) cannot spend an
    // unbounded time replaying cycles that all see the same measurement anyway.
    constexpr int kMaxCyclesPerAdvance = 64;
    int cycles = static_cast<int>(std::floor(dt / period_));
    if (cycles <= 0) return;
    if (cycles > kMaxCyclesPerAdvance) cycles = kMaxCyclesPerAdvance;

    for (int i = 0; i < cycles; i++) loop_->run_cycle();
    last_advance_ += cycles * period_;
}

}  // namespace auto_battlebot
