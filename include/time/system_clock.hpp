#pragma once

#include "time/clock_interface.hpp"
#include "time_utils.hpp"

namespace auto_battlebot {
/**
 * @brief Wall-clock time source (std::chrono::system_clock).
 *
 * The default when no [clock] section is configured, so hardware/playback behavior is unchanged.
 * set() is ignored; time always reflects the real wall clock.
 */
class SystemClock : public ClockInterface {
   public:
    double now() const override { return auto_battlebot::now(); }
};

}  // namespace auto_battlebot
