#pragma once

#include <optional>

#include "data_structures/camera.hpp"
#include "rgbd_camera/camera_settings.hpp"

namespace auto_battlebot {
class RgbdCameraInterface {
   public:
    virtual ~RgbdCameraInterface() = default;
    virtual bool initialize() = 0;
    // Request cancellation of a blocking initialize() call (e.g. while zed_.open() is pending).
    // Safe to call from another thread. Default is a no-op for cameras that open immediately.
    virtual void cancel_initialize() {}
    virtual bool get(CameraData &data) = 0;
    virtual bool should_close() = 0;
    virtual bool set_recording_enabled([[maybe_unused]] bool enabled) { return false; }
    virtual bool is_recording_enabled() const { return false; }
    virtual bool recording_encoder_stalled() const { return false; }
    virtual bool supports_camera_settings() const { return false; }
    virtual void request_camera_settings([[maybe_unused]] std::optional<CameraSettings> settings) {}
    virtual std::optional<CameraSettingsState> camera_settings() const { return std::nullopt; }
};

}  // namespace auto_battlebot
