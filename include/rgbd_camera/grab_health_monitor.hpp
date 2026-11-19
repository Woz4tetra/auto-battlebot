#pragma once

#include <chrono>
#include <cstddef>
#include <deque>
#include <thread>
#include <utility>

namespace auto_battlebot {

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
    std::thread::id recording_thread_{};
};

}  // namespace auto_battlebot
