#pragma once

#include <atomic>
#include <string>

#include "diagnostics_logger/diagnostics_logger.hpp"
#include "rgbd_camera/config.hpp"
#include "rgbd_camera/rgbd_camera_interface.hpp"
#include "rgbd_camera/zed_device.hpp"

namespace auto_battlebot {

class ZedSvoPlaybackCamera : public RgbdCameraInterface {
   public:
    explicit ZedSvoPlaybackCamera(ZedSvoPlaybackCameraConfiguration &config);
    ~ZedSvoPlaybackCamera() override;

    bool initialize() override;
    void cancel_initialize() override;
    bool get(CameraData &data) override;
    bool should_close() override { return should_close_; }

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

    bool rebase_stamps_ = true;
    bool stamp_offset_initialized_ = false;
    double stamp_offset_s_ = 0.0;

    std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;
};

}  // namespace auto_battlebot
