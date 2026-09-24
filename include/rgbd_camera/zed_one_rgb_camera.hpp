#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <sl/CameraOne.hpp>
#include <string>
#include <thread>

#include "diagnostics_logger/diagnostics_logger.hpp"
#include "mcap_recorder/mcap_recorder.hpp"
#include "publisher/output_channel.hpp"
#include "rgbd_camera/config.hpp"
#include "rgbd_camera/grab_health_monitor.hpp"
#include "rgbd_camera/rgbd_camera_interface.hpp"
#include "rgbd_camera/video_encoder.hpp"

namespace auto_battlebot {
/**
 * @brief Monocular ZED X One (the ZED X One S on the ZED Box Mini) through `sl::CameraOne`.
 *
 * The RGB-only counterpart of ZedRgbdCamera, shaped like V4l2RgbCamera: `CameraData::depth` is
 * empty, so the height gate abstains and the field filter must be one of the RGB ones, and
 * `tf_visodom_from_camera` is identity because a clamped camera does not move.
 *
 * Frames come back rectified by the SDK against the factory calibration. That lens model is
 * rational with thin-prism terms, which `CameraCalibration` cannot represent, so the rectified
 * stream is what gets published and recorded. The recording names a calibration_id whose file
 * holds the rectified intrinsics with zero distortion, which VideoPlaybackCamera loads to replay
 * it. The camera writes that file on first open if it is missing.
 */
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

   private:
    bool open_camera();
    /** Reads the rectified intrinsics into camera_info and makes sure a calibration file under
     *  config/cameras/ names them, so VideoPlaybackCamera can replay what this camera records. */
    void publish_calibration();
    bool start_encoder();
    void capture_thread_loop();
    bool capture_frame();

    ZedOneRgbCameraConfiguration config_;
    sl::CameraOne zed_;
    sl::Mat zed_bgra_;
    std::future<sl::ERROR_CODE> pending_open_;
    std::atomic<bool> cancel_open_{false};
    int width_ = 0;
    int height_ = 0;
    std::string calibration_id_;

    CameraData latest_data_;
    mutable std::mutex data_mutex_;
    mutable std::condition_variable data_cv_;
    std::thread capture_thread_;

    std::atomic<bool> is_initialized_{false};
    std::atomic<bool> should_close_{false};
    std::atomic<bool> stop_thread_{false};
    std::atomic<uint64_t> frame_counter_{0};
    mutable uint64_t last_returned_frame_counter_ = 0;
    std::atomic<int64_t> video_frame_index_{-1};

    GrabHealthMonitor grab_health_;

    std::shared_ptr<McapRecorder> mcap_recorder_;
    std::unique_ptr<OutputChannel> video_channel_;
    VideoEncoder encoder_;
    std::atomic<bool> recording_desired_{true};
    uint64_t reported_drops_ = 0;

    std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;
};
}  // namespace auto_battlebot
