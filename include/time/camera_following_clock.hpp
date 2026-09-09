#pragma once

#include <atomic>
#include <chrono>

#include "time/clock_interface.hpp"

namespace auto_battlebot {

/**
 * Logical time anchored to the camera and advanced by the wall clock between frames.
 *
 * now() is the newest frame stamp plus the real time elapsed since the Runner handed it over, so
 * `now() - frame_stamp` is this machine's actual processing age of that frame, whatever timeline
 * the stamps live on. That is the quantity the filter's forward propagation needs, and it is the
 * only one of the three clocks that reports it correctly off a recording.
 *
 * SystemClock gets it right only when the camera stamps are wall-clock, which SVO playback stamps
 * are not: unrebased they sit on the recording's clock (a gap of days), and rebased they carry a
 * fixed offset captured during the startup transient that measured ~790 ms of the wrong sign on a
 * 70 s replay. Either way every propagation short-circuits and the emitted estimate freezes at the
 * shutter. ManualClock reports zero elapsed time, so it never coasts between frames at all.
 *
 * set() is called from the perception thread and now() from the control thread, so the anchor is
 * a single atomic offset rather than a stamp/instant pair that could be read torn.
 */
class CameraFollowingClock : public ClockInterface {
   public:
    double now() const override {
        // Before the first frame there is nothing to anchor to. The control loop returns early
        // until a measurement has defined the field, so nothing reads this value.
        if (!anchored_.load(std::memory_order_acquire)) return 0.0;
        return steady_seconds() + offset_.load(std::memory_order_relaxed);
    }

    void set(double seconds) override {
        offset_.store(seconds - steady_seconds(), std::memory_order_relaxed);
        anchored_.store(true, std::memory_order_release);
    }

   private:
    static double steady_seconds() {
        return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
            .count();
    }

    std::atomic<double> offset_{0.0};
    std::atomic<bool> anchored_{false};
};

}  // namespace auto_battlebot
