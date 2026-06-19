#pragma once

#include "time/clock_interface.hpp"

namespace auto_battlebot {
/**
 * @brief Logical time set externally, typically from the camera frame timestamp.
 *
 * Used for the headless sim, deterministic SVO/MCAP playback, and unit tests. now() returns
 * whatever was last set(), so the control loop's time advances exactly with the frames it
 * processes, independent of wall-clock speed.
 */
class ManualClock : public ClockInterface {
   public:
    double now() const override { return time_seconds_; }
    void set(double seconds) override { time_seconds_ = seconds; }

   private:
    double time_seconds_ = 0.0;
};

}  // namespace auto_battlebot
