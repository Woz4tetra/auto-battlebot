#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <future>
#include <limits>
#include <memory>
#include <sl/Camera.hpp>
#include <thread>

#include "diagnostics_logger/diagnostics_logger.hpp"
#include "rgbd_camera/config.hpp"
#include "rgbd_camera/grab_health_monitor.hpp"
#include "rgbd_camera/rgbd_camera_interface.hpp"
#include "rgbd_camera/svo_recorder.hpp"
#include "rgbd_camera/zed_device.hpp"
#include "thread_safety/mutex.hpp"

namespace auto_battlebot {
class ZedRgbdCamera : public RgbdCameraInterface {
   public:
    explicit ZedRgbdCamera(const ZedRgbdCameraConfiguration &config,
                           std::unique_ptr<ZedDevice> device = std::make_unique<ZedDevice>());
    ~ZedRgbdCamera();
    bool initialize() override;
    void cancel_initialize() override;
    bool get(CameraData &data) override;
    bool should_close() override;
    bool set_recording_enabled(bool enabled) override;
    bool is_recording_enabled() const override;
    std::string get_current_svo_path() const;

   private:
    void capture_thread_loop();
    bool capture_frame();
    void reset_capture_timing_stats() const REQUIRES(data_mutex_);
    void reset_runtime_state();
    bool frame_or_disconnect_ready() const REQUIRES(data_mutex_);
    bool wait_for_frame_or_disconnect(MutexLock &lock, int &wait_loops) REQUIRES(data_mutex_);

    std::atomic<bool> cancel_open_{false};
    std::atomic<bool> capture_thread_done_{false};

    std::unique_ptr<ZedDevice> device_;
    std::chrono::milliseconds capture_join_timeout_;
    mutable Mutex data_mutex_;
    CameraData latest_data_ GUARDED_BY(data_mutex_);
    mutable std::condition_variable data_cv_;
    std::thread capture_thread_;

    std::atomic<bool> capture_thread_abandoned_{false};
    std::atomic<bool> capture_running_;
    std::atomic<bool> shutdown_requested_;
    std::atomic<bool> stop_capture_thread_;
    std::atomic<bool> last_grab_ok_;

    std::atomic<uint64_t> frame_counter_;
    mutable uint64_t last_returned_frame_counter_ GUARDED_BY(data_mutex_);

    GrabHealthMonitor grab_health_;

    std::unique_ptr<SvoRecorder> svo_recorder_;
    std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;

    mutable uint64_t captures_since_last_report_ GUARDED_BY(data_mutex_);
    mutable double capture_time_sum_ms_ GUARDED_BY(data_mutex_);
    mutable double capture_time_min_ms_ GUARDED_BY(data_mutex_);
    mutable double capture_time_max_ms_ GUARDED_BY(data_mutex_);
};

}  // namespace auto_battlebot
