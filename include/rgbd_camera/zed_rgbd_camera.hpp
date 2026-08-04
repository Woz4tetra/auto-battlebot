#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <sl/Camera.hpp>
#include <thread>

#include "diagnostics_logger/diagnostics_logger.hpp"
#include "rgbd_camera/config.hpp"
#include "rgbd_camera/grab_health_monitor.hpp"
#include "rgbd_camera/rgbd_camera_interface.hpp"
#include "rgbd_camera/svo_recorder.hpp"

namespace auto_battlebot {
inline sl::RESOLUTION get_zed_resolution(Resolution resolution) {
    switch (resolution) {
        case Resolution::RES_3856x2180:
            return sl::RESOLUTION::HD4K;
        case Resolution::RES_3800x1800:
            return sl::RESOLUTION::QHDPLUS;
        case Resolution::RES_2208x1242:
            return sl::RESOLUTION::HD2K;
        case Resolution::RES_1920x1536:
            return sl::RESOLUTION::HD1536;
        case Resolution::RES_1920x1080:
            return sl::RESOLUTION::HD1080;
        case Resolution::RES_1920x1200:
            return sl::RESOLUTION::HD1200;
        case Resolution::RES_1280x720:
            return sl::RESOLUTION::HD720;
        case Resolution::RES_960x600:
            return sl::RESOLUTION::SVGA;
        case Resolution::RES_672x376:
            return sl::RESOLUTION::VGA;
    }
    throw std::invalid_argument("Unknown Resolution value");
}

inline sl::DEPTH_MODE get_zed_depth_mode(DepthMode depth_mode) {
    switch (depth_mode) {
        case DepthMode::ZED_NONE:
            return sl::DEPTH_MODE::NONE;
        case DepthMode::ZED_PERFORMANCE:
            return sl::DEPTH_MODE::PERFORMANCE;
        case DepthMode::ZED_QUALITY:
            return sl::DEPTH_MODE::QUALITY;
        case DepthMode::ZED_ULTRA:
            return sl::DEPTH_MODE::ULTRA;
        case DepthMode::ZED_NEURAL_LIGHT:
            return sl::DEPTH_MODE::NEURAL_LIGHT;
        case DepthMode::ZED_NEURAL:
            return sl::DEPTH_MODE::NEURAL;
        case DepthMode::ZED_NEURAL_PLUS:
            return sl::DEPTH_MODE::NEURAL_PLUS;
    }
    throw std::invalid_argument("Unknown DepthMode value");
}

class ZedRgbdCamera : public RgbdCameraInterface {
   public:
    explicit ZedRgbdCamera(ZedRgbdCameraConfiguration &config);
    ~ZedRgbdCamera();
    bool initialize() override;
    void cancel_initialize() override;
    bool get(CameraData &data, bool get_depth) override;
    bool should_close() override;
    bool set_recording_enabled(bool enabled) override;
    bool is_recording_enabled() const override;
    std::string get_current_svo_path() const;

   private:
    void capture_thread_loop();
    bool capture_frame();
    void reset_capture_timing_stats() const;
    void reset_runtime_state();
    // Wait for the pending open() future, leaking it on hard timeout so shutdown never
    // blocks on a wedged SDK call. Returns true if open() completed, false if it was leaked.
    bool await_or_leak_open(const char *context, const char *validation_label);
    // Block until frame_ready() (or a lifecycle/disconnect event). `lock` must be held on
    // entry. Returns false if the camera disconnected while waiting.
    bool wait_for_new_frame(std::unique_lock<std::mutex> &lock, int &wait_loops,
                            const std::function<bool()> &frame_ready);

    // Asynchronous open() handling.
    std::atomic<bool> cancel_open_{false};
    std::future<sl::ERROR_CODE> pending_open_;
    std::atomic<bool> capture_thread_done_{false};

    // ZED SDK handles and the most recently captured frame.
    sl::Camera zed_;
    sl::InitParameters params_;
    sl::Mat zed_rgb_;
    sl::Mat zed_depth_;
    sl::Pose zed_pose_;
    CameraData latest_data_;
    mutable std::mutex data_mutex_;
    mutable std::condition_variable data_cv_;
    std::thread capture_thread_;

    // Lifecycle flags.
    //   is_initialized_:   open() succeeded and the capture thread is running.
    //   should_close_:     capture requested application shutdown (fatal error / end of SVO).
    //   stop_thread_:      request the capture thread to exit (re-init / destruction).
    //   camera_connected_: last grab() succeeded; cleared on grab failure.
    std::atomic<bool> is_initialized_;
    std::atomic<bool> should_close_;
    std::atomic<bool> stop_thread_;
    std::atomic<bool> camera_connected_;

    // Frame handoff between capture thread and get(). Counters guarded by data_mutex_.
    std::atomic<uint64_t> frame_counter_;
    uint64_t depth_frame_counter_;
    mutable uint64_t last_returned_frame_counter_;
    bool depth_requested_ = false;

    GrabHealthMonitor grab_health_;

    sl::POSITIONAL_TRACKING_STATE prev_tracking_state_;
    bool position_tracking_enabled_;
    bool is_playback_input_;
    /// Absolute path of the SVO being replayed, empty for a live camera.
    std::string playback_svo_path_;
    int svo_start_frame_;
    // Offset added to SVO frame stamps so replay output starts at the current wall clock
    // instead of the original recording time. Computed once from the first grabbed frame;
    // inter-frame deltas are preserved, so stamp-driven replay stays deterministic.
    bool playback_stamp_offset_initialized_ = false;
    double playback_stamp_offset_s_ = 0.0;

    std::unique_ptr<SvoRecorder> svo_recorder_;
    std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;

    // Capture timing statistics (protected by data_mutex_)
    mutable uint64_t captures_since_last_report_;
    mutable double capture_time_sum_ms_;
    mutable double capture_time_min_ms_;
    mutable double capture_time_max_ms_;
};

}  // namespace auto_battlebot
