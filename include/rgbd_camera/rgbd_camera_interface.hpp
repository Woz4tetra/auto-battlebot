#pragma once

#include "data_structures/camera.hpp"

namespace auto_battlebot {
class RgbdCameraInterface {
   public:
    virtual ~RgbdCameraInterface() = default;
    virtual bool initialize() = 0;
    // Request cancellation of a blocking initialize() call (e.g. while zed_.open() is pending).
    // Safe to call from another thread. Default is a no-op for cameras that open immediately.
    virtual void cancel_initialize() {}
    virtual bool get(CameraData &data, bool get_depth = false) = 0;
    virtual bool should_close() = 0;
    /** Runtime control for capture on cameras that support it. */
    virtual bool set_recording_enabled([[maybe_unused]] bool enabled) { return false; }
    virtual bool is_recording_enabled() const { return false; }

    /**
     * Pause/resume background frame capture, for callers that hold a frame across a long,
     * unpredictable gap (e.g. blocking on a keypress to single-step SVO playback). Cameras that
     * grab continuously in a background thread (ZedRgbdCamera) otherwise keep advancing --
     * including through SVO real-time pacing -- while the caller isn't calling get(), so the
     * next get() silently returns a frame far ahead of the last one instead of the next one.
     * Default is a no-op for cameras with no such background thread (Noop, sim).
     */
    virtual void pause_capture() {}
    virtual void resume_capture() {}
};

}  // namespace auto_battlebot
