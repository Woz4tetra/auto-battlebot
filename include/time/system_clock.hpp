#pragma once

#include "time/clock_interface.hpp"
#include "time_utils.hpp"

namespace auto_battlebot {
class SystemClock : public ClockInterface {
   public:
    double now() const override { return auto_battlebot::now(); }
};

}  // namespace auto_battlebot
