#pragma once

#include <atomic>
#include <chrono>

#include "time/clock_interface.hpp"

namespace auto_battlebot {

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
