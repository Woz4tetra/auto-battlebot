#include "rgbd_camera/grab_health_monitor.hpp"

namespace auto_battlebot {

GrabHealthMonitor::GrabHealthMonitor(std::chrono::steady_clock::duration window,
                                     double shutdown_ratio)
    : window_(window), shutdown_ratio_(shutdown_ratio) {}

void GrabHealthMonitor::record(bool is_error, std::chrono::steady_clock::time_point now) {
    last_now_ = now;
    samples_.emplace_back(now, is_error);
    if (is_error) {
        error_count_++;
    }
    // Drop samples that have aged out of the window.
    while (!samples_.empty() && (now - samples_.front().first) > window_) {
        if (samples_.front().second) {
            error_count_--;
        }
        samples_.pop_front();
    }
}

bool GrabHealthMonitor::window_full() const {
    return !samples_.empty() && (last_now_ - samples_.front().first) >= window_;
}

double GrabHealthMonitor::error_ratio() const {
    if (samples_.empty()) {
        return 0.0;
    }
    return static_cast<double>(error_count_) / static_cast<double>(samples_.size());
}

bool GrabHealthMonitor::should_shutdown() const {
    return window_full() && error_ratio() > shutdown_ratio_;
}

void GrabHealthMonitor::reset() {
    samples_.clear();
    error_count_ = 0;
}

}  // namespace auto_battlebot
