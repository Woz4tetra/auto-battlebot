#pragma once

#include <atomic>

#include "time/clock_interface.hpp"

namespace auto_battlebot {

class SteppedClock : public ClockInterface {
   public:
    double now() const override { return seconds_.load(std::memory_order_acquire); }

    void set(double seconds) override {
        if (!stepping_.load(std::memory_order_acquire)) {
            seconds_.store(seconds, std::memory_order_release);
        }
    }

    void step_to(double seconds) override {
        stepping_.store(true, std::memory_order_release);
        seconds_.store(seconds, std::memory_order_release);
    }

   private:
    std::atomic<double> seconds_{0.0};
    std::atomic<bool> stepping_{false};
};

}  // namespace auto_battlebot
