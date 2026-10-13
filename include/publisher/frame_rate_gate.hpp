#pragma once

#include <chrono>

namespace auto_battlebot {

/**
 * Lets frames through at up to `rate_hz` on a fixed cadence.
 *
 * Each frame let through moves the next due time one period past the previous due time, not
 * past now. A rule of "a full period since the last frame sent" cut a 31 Hz camera to about 20 fps
 * under a 30 Hz cap: any frame that arrived a little under 33.3 ms after the last one sent waited
 * for the next, leaving a 64 ms gap. On a fixed cadence that source gets 30 fps.
 *
 * A frame up to half a period early still goes through, against that due time, so the cadence
 * holds when arrivals jitter: a camera alternating 27 and 37 ms gaps gets 30.0 fps this way and
 * 28.7 without it. The average never exceeds the rate, since each frame let through moves the due
 * time a full period. A gap longer than a period restarts the cadence at the frame that ends it,
 * so a stall is never followed by a burst.
 */
class FrameRateGate {
   public:
    using Clock = std::chrono::steady_clock;

    /** rate_hz <= 0 lets every frame through. */
    explicit FrameRateGate(double rate_hz)
        : period_(rate_hz > 0.0 ? std::chrono::duration_cast<Clock::duration>(
                                      std::chrono::duration<double>(1.0 / rate_hz))
                                : Clock::duration::zero()),
          early_(period_ / 2) {}

    /** Whether the frame arriving at `now` goes through. */
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
