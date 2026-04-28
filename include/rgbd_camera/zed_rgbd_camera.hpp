#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <filesystem>
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
    explicit ZedRgbdCamera(ZedRgbdCameraConfiguration &config);
    ~ZedRgbdCamera();
    bool initialize() override;
    bool get(CameraData &data, bool get_depth) override;
    bool should_close() override;
    bool set_recording_enabled(bool enabled) override;
    bool is_recording_enabled() const override;
    std::string get_current_svo_path() const;

   private:
    // What the main thread is asking the capture thread to do. Close is just the initial
    // value while the thread idles between construction and the first initialize() call;
    // there is no public method that sets Close.
    enum class Request : uint8_t {
        Close,  // no camera; stay idle (initial value)
        Open,   // open and grab continuously (set by initialize())
        Stop    // exit the thread entirely (set by the destructor)
    };

    // What the capture thread is currently doing.
    enum class State : uint8_t {
        Idle,     // alive, no camera open
        Opening,  // currently inside zed_.open() / enablePositionalTracking()
        Ready,    // camera open, grab loop running
        Closing   // currently inside zed_.close()
    };

    // SVO recording change requested via set_recording_enabled(). Consumed and reset to
    // None by the capture thread between grabs.
    enum class RecordIntent : uint8_t {
        None,   // no change requested
        Stop,   // please stop recording
        Start   // please start recording
    };

    void capture_thread_loop();
    bool open_and_configure();
    bool grab_one_frame();
    void apply_record_intent();
    void reset_capture_timing_stats() const;
    bool start_svo_recording();
    void stop_svo_recording();
    void enforce_holding_dir_size();
    std::string generate_svo_filename() const;

    sl::Camera zed_;
    sl::InitParameters params_;
    sl::Mat zed_rgb_;
    sl::Mat zed_depth_;
    sl::Pose zed_pose_;
    CameraData latest_data_;
    mutable std::mutex data_mutex_;
    mutable std::condition_variable data_cv_;
    std::thread capture_thread_;

    std::atomic<Request> request_{Request::Close};
    std::atomic<State> state_{State::Idle};
    std::atomic<sl::ERROR_CODE> last_open_error_{sl::ERROR_CODE::SUCCESS};
    std::atomic<RecordIntent> record_intent_{RecordIntent::None};
    // Sticky "the application should shut down" flag (SVO end-of-file or grab error
    // ratio threshold exceeded). Reset by initialize() on a fresh attempt.
    std::atomic<bool> should_close_{false};

    std::atomic<uint64_t> frame_counter_;
    uint64_t depth_frame_counter_;
    mutable uint64_t last_returned_frame_counter_;
    mutable std::queue<int> depth_request_queue_;
    // Capture-thread-local error window (only touched by capture thread).
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
