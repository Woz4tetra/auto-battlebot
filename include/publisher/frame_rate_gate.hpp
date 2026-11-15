#pragma once

#include <chrono>

namespace auto_battlebot {

class FrameRateGate {
   public:
    using Clock = std::chrono::steady_clock;

    explicit FrameRateGate(double rate_hz)
        : period_(rate_hz > 0.0 ? std::chrono::duration_cast<Clock::duration>(
                                      std::chrono::duration<double>(1.0 / rate_hz))
                                : Clock::duration::zero()),
          early_(period_ / 2) {}

    bool admit(Clock::time_point now) {
        if (period_ == Clock::duration::zero()) return true;
        if (started_ && now < next_due_ - early_) return false;
        next_due_ = (!started_ || now - next_due_ >= period_) ? now + period_ : next_due_ + period_;
        started_ = true;
        return true;
    }

   private:
    Clock::duration period_;
    Clock::duration early_;
    Clock::time_point next_due_{};
    bool started_ = false;
};

}  // namespace auto_battlebot
