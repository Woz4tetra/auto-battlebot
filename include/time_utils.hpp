#pragma once

#include <chrono>

namespace auto_battlebot
{
    /**
     * @brief Generate a timestamp using the system clock
     *
     * Returns the current time as seconds since epoch (with fractional seconds)
     *
     * @return double Time in seconds since epoch
     */
    inline double now()
    {
        auto time_point = std::chrono::system_clock::now();
        auto duration = time_point.time_since_epoch();
        auto seconds = std::chrono::duration_cast<std::chrono::duration<double>>(duration);
        return seconds.count();
    }

} // namespace auto_battlebot
