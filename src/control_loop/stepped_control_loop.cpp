#include "control_loop/stepped_control_loop.hpp"

#include <cmath>
#include <utility>

namespace auto_battlebot {

SteppedControlLoop::SteppedControlLoop(std::shared_ptr<ControlLoop> loop, double rate_hz,
                                       int max_catchup_cycles)
    : ControlLoopInterface(std::move(loop)),
      period_s_(rate_hz > 0.0 ? std::optional<double>(1.0 / rate_hz) : std::nullopt),
      max_catchup_cycles_(max_catchup_cycles) {}

void SteppedControlLoop::advance_to(double until) {
    if (!period_s_) {
        loop_->run_cycle();
        return;
    }

    if (!started_) {
        started_ = true;
        last_advance_ = until;
        loop_->run_cycle_at(until);
        return;
    }

    const double dt = until - last_advance_;
    if (dt <= 0.0) return;

    int cycles = static_cast<int>(std::floor(dt / *period_s_));
    if (cycles <= 0) return;
    if (cycles > max_catchup_cycles_) cycles = max_catchup_cycles_;

    for (int i = 1; i <= cycles; i++) loop_->run_cycle_at(last_advance_ + i * *period_s_);
    last_advance_ += cycles * *period_s_;
}

}  // namespace auto_battlebot
