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

    /// Record the outcome of a single grab() call at time @p now.
    void record(bool is_error, std::chrono::steady_clock::time_point now);

    /// True once the oldest retained sample is at least a full window old.
    bool window_full() const;

    /// Fraction of retained samples that were errors, in [0, 1].
    double error_ratio() const;

    /// True when the window is full and the error ratio exceeds the threshold.
    bool should_shutdown() const;

    /// Drop all recorded samples (used on re-initialization).
    void reset();

   private:
    std::chrono::steady_clock::duration window_;
    double shutdown_ratio_;
    std::deque<std::pair<std::chrono::steady_clock::time_point, bool>> samples_;
    std::size_t error_count_ = 0;
    std::chrono::steady_clock::time_point last_now_{};
};

}  // namespace auto_battlebot
