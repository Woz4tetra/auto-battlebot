#pragma once

#include <atomic>
#include <string>

#include "diagnostics_logger/diagnostics_logger.hpp"
#include "rgbd_camera/config.hpp"
#include "rgbd_camera/rgbd_camera_interface.hpp"
#include "rgbd_camera/zed_device.hpp"

namespace auto_battlebot {

/**
 * Replays an SVO file synchronously: get() grabs on the caller's thread and returns that frame.
 *
 * No capture thread, so every frame reaches the pipeline in order and none are skipped. That is
 * what makes replay reproducible, and reproducible replay is what the project's regression testing
 * rests on. A live camera cannot work this way, because it must not block the pipeline on camera
 * I/O and must drop frames to keep up; ZedRgbdCamera keeps that behavior.
 *
 * Consequently there is no recorder, no reconnection, and no grab-health monitor here. Recording
 * while replaying is meaningless, and a file does not disconnect: it ends, which is a normal
 * shutdown rather than a failure to recover from.
 */
class ZedSvoPlaybackCamera : public RgbdCameraInterface {
   public:
    explicit ZedSvoPlaybackCamera(ZedSvoPlaybackCameraConfiguration &config);
    ~ZedSvoPlaybackCamera() override;

    bool initialize() override;
    void cancel_initialize() override;
    bool get(CameraData &data) override;
    bool should_close() override { return should_close_; }

    /** Recording an SVO while replaying one is meaningless; always refuses. */
    bool set_recording_enabled(bool enabled) override;
    bool is_recording_enabled() const override { return false; }

   private:
    ZedDevice device_;
    CameraData latest_data_;
    std::string svo_path_;
    int svo_start_frame_ = 0;

    std::atomic<bool> cancel_open_{false};
    std::atomic<bool> is_initialized_{false};
    std::atomic<bool> should_close_{false};

    /** Rebases frame stamps onto the current wall clock so a replay recording starts now rather
     *  than at the original recording time. Off for regression runs that compare recordings
     *  between builds, where a wall-clock offset makes every payload differ. */
    bool rebase_stamps_ = true;
    bool stamp_offset_initialized_ = false;
    double stamp_offset_s_ = 0.0;

    std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;
};

}  // namespace auto_battlebot
