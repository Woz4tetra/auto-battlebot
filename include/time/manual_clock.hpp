#pragma once

#include "time/clock_interface.hpp"

namespace auto_battlebot {
class ManualClock : public ClockInterface {
   public:
    double now() const override { return time_seconds_; }
    void set(double seconds) override { time_seconds_ = seconds; }

   private:
    double time_seconds_ = 0.0;
};

}  // namespace auto_battlebot
