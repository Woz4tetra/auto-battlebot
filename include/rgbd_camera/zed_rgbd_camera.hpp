#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <future>
#include <limits>
#include <mutex>
#include <queue>
#include <sl/Camera.hpp>
#include <thread>

#include "diagnostics_logger/diagnostics_logger.hpp"
#include "rgbd_camera/config.hpp"
#include "rgbd_camera/rgbd_camera_interface.hpp"

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
    ZedRgbdCamera(ZedRgbdCameraConfiguration &config);
    ~ZedRgbdCamera();
    bool initialize() override;
    void cancel_initialize() override;
    bool get(CameraData &data, bool get_depth) override;
    bool should_close() override;
    bool set_svo_recording_enabled(bool enabled) override;
    bool is_svo_recording_enabled() const override;
    std::string get_current_svo_path() const;

   private:
    void capture_thread_loop();
    bool capture_frame();
    void reset_capture_timing_stats() const;
    bool start_svo_recording();
    void stop_svo_recording();
    void enforce_holding_dir_size();
    std::string generate_svo_filename() const;

    std::atomic<bool> cancel_open_{false};
    std::future<sl::ERROR_CODE> pending_open_;
    std::atomic<bool> capture_thread_done_{false};

    sl::Camera zed_;
    sl::InitParameters params_;
    sl::Mat zed_rgb_;
    sl::Mat zed_depth_;
    sl::Pose zed_pose_;
    CameraData latest_data_;
    mutable std::mutex data_mutex_;
    mutable std::condition_variable data_cv_;
    std::thread capture_thread_;
    std::atomic<bool> is_initialized_;
    std::atomic<bool> should_close_;
    std::atomic<bool> stop_thread_;
    std::atomic<bool> camera_connected_;
    std::atomic<bool> has_new_frame_;
    std::atomic<uint64_t> frame_counter_;
    uint64_t depth_frame_counter_;
    mutable uint64_t last_returned_frame_counter_;
    mutable std::queue<int> depth_request_queue_;
    std::deque<std::pair<std::chrono::steady_clock::time_point, bool>> recent_grab_results_;
    size_t recent_grab_error_count_;
    sl::POSITIONAL_TRACKING_STATE prev_tracking_state_;
    bool position_tracking_enabled_;
    bool is_playback_input_;
    std::atomic<bool> svo_recording_enabled_;
    std::filesystem::path svo_holding_dir_;
    std::filesystem::path current_svo_path_;
    uint64_t svo_max_size_bytes_;
    uint64_t svo_holding_dir_max_size_bytes_;
    uint64_t frames_since_size_check_;
    int svo_start_frame_;

    std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;

    // Capture timing statistics (protected by data_mutex_)
    mutable uint64_t captures_since_last_report_;
    mutable double capture_time_sum_ms_;
    mutable double capture_time_min_ms_;
    mutable double capture_time_max_ms_;
};

}  // namespace auto_battlebot
