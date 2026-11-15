#pragma once

#include <chrono>

namespace auto_battlebot {
inline double now() {
    auto time_point = std::chrono::system_clock::now();
    auto duration = time_point.time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::duration<double>>(duration);
    return seconds.count();
}

template <typename Rep, typename Period>
inline double to_ms(const std::chrono::duration<Rep, Period>& d) {
    return std::chrono::duration<double, std::milli>(d).count();
}

inline double ms_since(const std::chrono::steady_clock::time_point& start) {
    return to_ms(std::chrono::steady_clock::now() - start);
}

}  // namespace auto_battlebot
