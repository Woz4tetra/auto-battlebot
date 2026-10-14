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
    /** Runtime control for capture on cameras that support it. */
    virtual bool set_recording_enabled([[maybe_unused]] bool enabled) { return false; }
    virtual bool is_recording_enabled() const { return false; }
    /** The camera's recording failed in a way only an off and on fixes (its video encoder
     *  stopped taking frames). The runner checks this once a second and restarts recording. */
    virtual bool recording_needs_restart() const { return false; }
    /** Live image controls (exposure, gain, white balance) on cameras that support them. */
    virtual bool supports_camera_settings() const { return false; }
    /** Queue settings for the capture thread. nullopt restores what the camera opened with. */
    virtual void request_camera_settings([[maybe_unused]] std::optional<CameraSettings> settings) {}
    /** Values last read back from the camera and its limits; nullopt until it has opened. */
    virtual std::optional<CameraSettingsState> camera_settings() const { return std::nullopt; }
};

}  // namespace auto_battlebot
