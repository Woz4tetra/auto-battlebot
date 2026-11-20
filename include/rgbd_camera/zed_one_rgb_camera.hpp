#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <sl/CameraOne.hpp>
#include <string>
#include <thread>
#include <variant>

#include "diagnostics_logger/diagnostics_logger.hpp"
#include "mcap_recorder/mcap_recorder.hpp"
#include "publisher/output_channel.hpp"
#include "rgbd_camera/camera_calibration.hpp"
#include "rgbd_camera/config.hpp"
#include "rgbd_camera/grab_health_monitor.hpp"
#include "rgbd_camera/rgbd_camera_interface.hpp"
#include "rgbd_camera/video_encoder.hpp"

namespace auto_battlebot {
class ZedOneRgbCamera : public RgbdCameraInterface {
   public:
    ZedOneRgbCamera(const ZedOneRgbCameraConfiguration &config,
                    std::shared_ptr<McapRecorder> mcap_recorder);
    ~ZedOneRgbCamera() override;

    bool initialize() override;
    void cancel_initialize() override { cancel_open_.store(true); }
    bool get(CameraData &data) override;
    bool should_close() override { return should_close_.load(); }
    bool set_recording_enabled(bool enabled) override;
    bool is_recording_enabled() const override;
    bool recording_encoder_stalled() const override { return encoder_.broken(); }
    bool supports_camera_settings() const override { return true; }
    void request_camera_settings(std::optional<CameraSettings> settings) override;
    std::optional<CameraSettingsState> camera_settings() const override;

   private:
    bool open_camera();
    CameraSettings read_settings(const CameraSettings &known);
    CameraSettingLimits read_limits();
    void apply_settings(const CameraSettings &settings);
    void service_settings(const CameraSettings &opened_settings);
    void read_calibration();
    bool start_encoder();
    void capture_thread_loop(CameraSettings opened_settings);
    bool capture_frame();

    ZedOneRgbCameraConfiguration config_;
    sl::CameraOne zed_;
    sl::Mat zed_bgra_;
    std::future<sl::ERROR_CODE> pending_open_;
    std::atomic<bool> cancel_open_{false};
    int width_ = 0;
    int height_ = 0;
    CameraCalibration calibration_;

    CameraData latest_data_;
    mutable std::mutex data_mutex_;
    mutable std::condition_variable data_cv_;
    std::thread capture_thread_;

    std::atomic<bool> is_initialized_{false};
    std::atomic<bool> should_close_{false};
    std::atomic<bool> stop_thread_{false};
    std::atomic<bool> capture_thread_done_{false};
    std::atomic<uint64_t> frame_counter_{0};
    mutable uint64_t last_returned_frame_counter_ = 0;

    GrabHealthMonitor grab_health_;

    std::shared_ptr<McapRecorder> mcap_recorder_;
    std::unique_ptr<OutputChannel> video_channel_;
    VideoEncoder encoder_;
    std::atomic<bool> recording_desired_{true};
    uint64_t reported_drops_ = 0;

    mutable std::mutex settings_mutex_;
    struct RestoreOpenedSettings {};
    using SettingsRequest = std::variant<CameraSettings, RestoreOpenedSettings>;
    std::optional<SettingsRequest> pending_settings_;
    std::optional<CameraSettingsState> settings_state_;
    int frames_since_settings_read_ = 0;

    std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;
};
}  // namespace auto_battlebot
