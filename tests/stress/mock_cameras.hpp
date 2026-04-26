#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

#include "rgbd_camera/rgbd_camera_interface.hpp"

namespace auto_battlebot {

// Blocks forever inside get(). Simulates a ZED grab() hang.
class BlockingCamera : public RgbdCameraInterface {
   public:
    bool initialize() override { return true; }
    bool get(CameraData &, [[maybe_unused]] bool get_depth) override {
        std::unique_lock<std::mutex> lock(mutex_);
        never_.wait(lock);
        return true;
    }
    bool should_close() override { return false; }

   private:
    std::mutex mutex_;
    std::condition_variable never_;
};

// get() fails immediately; initialize() blocks forever.
// Simulates: camera stalls during re-initialization after a failure.
class FailThenBlockCamera : public RgbdCameraInterface {
   public:
    bool initialize() override {
        std::unique_lock<std::mutex> lock(mutex_);
        never_.wait(lock);
        return true;
    }
    bool get(CameraData &, [[maybe_unused]] bool get_depth) override { return false; }
    bool should_close() override { return false; }

   private:
    std::mutex mutex_;
    std::condition_variable never_;
};

// Succeeds for `success_count` frames (with optional per-frame delay), then signals should_close().
// Provides a clean exit path for survivability tests.
class CountdownCamera : public RgbdCameraInterface {
   public:
    explicit CountdownCamera(int success_count,
                             std::chrono::milliseconds delay = std::chrono::milliseconds(0))
        : remaining_(success_count), delay_(delay) {}

    bool initialize() override { return true; }
    bool get(CameraData &, [[maybe_unused]] bool get_depth) override {
        if (delay_.count() > 0) std::this_thread::sleep_for(delay_);
        if (remaining_.fetch_sub(1) > 0) return true;
        close_.store(true);
        return false;
    }
    bool should_close() override { return close_.load(); }

   private:
    std::atomic<int> remaining_;
    std::atomic<bool> close_{false};
    std::chrono::milliseconds delay_;
};

}  // namespace auto_battlebot
