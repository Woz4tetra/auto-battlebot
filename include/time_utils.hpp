#pragma once

#include <chrono>

namespace auto_battlebot {
/**
 * @brief Generate a timestamp using the system clock
 *
 * Returns the current time as seconds since epoch (with fractional seconds)
 *
 * @return double Time in seconds since epoch
 */
inline double now() {
    auto time_point = std::chrono::system_clock::now();
    auto duration = time_point.time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::duration<double>>(duration);
    return seconds.count();
}

/**
 * @brief Convert a chrono duration to fractional milliseconds (double).
 */
template <typename Rep, typename Period>
inline double to_ms(const std::chrono::duration<Rep, Period>& d) {
    return std::chrono::duration<double, std::milli>(d).count();
}

/**
 * @brief Milliseconds elapsed (as a double) since the given steady_clock time point.
 */
inline double ms_since(const std::chrono::steady_clock::time_point& start) {
    return to_ms(std::chrono::steady_clock::now() - start);
}

}  // namespace auto_battlebot
