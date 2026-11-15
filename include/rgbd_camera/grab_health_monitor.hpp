#pragma once

#include <chrono>
#include <cstddef>
#include <deque>
#include <utility>

namespace auto_battlebot {

/**
 * @brief Tracks the camera grab() failure ratio over a sliding time window.
 *
 * Each grab outcome is recorded with its timestamp. Samples older than the
 * window are dropped. Once the window spans its full duration, the monitor can
 * report whether the error ratio has exceeded the shutdown threshold.
 *
 * Not thread-safe: only the capture thread records and queries it.
 */
class GrabHealthMonitor {
   public:
    GrabHealthMonitor(std::chrono::steady_clock::duration window, double shutdown_ratio);

    void record(bool is_error, std::chrono::steady_clock::time_point now);

    bool window_full() const;

    double error_ratio() const;

    bool should_shutdown() const;

    void reset();

   private:
    std::chrono::steady_clock::duration window_;
    double shutdown_ratio_;
    std::deque<std::pair<std::chrono::steady_clock::time_point, bool>> samples_;
    std::size_t error_count_ = 0;
    std::chrono::steady_clock::time_point last_now_{};
};

}  // namespace auto_battlebot
