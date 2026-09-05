#pragma once

#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <sl/Camera.hpp>
#include <string>

#include "data_structures/camera.hpp"
#include "rgbd_camera/config.hpp"

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

/**
 * Owns the ZED SDK handle and turns one grab into a CameraData.
 *
 * Knows nothing about threading, recording, replay, or reconnection: those differ between a live
 * camera and an SVO replay, and live in the two RgbdCameraInterface implementations that share
 * this class. What is here is everything both of them would otherwise have to duplicate, notably
 * the open() cancellation and leak-on-timeout handling, which is subtle enough that two copies
 * would drift.
 */
class ZedDevice {
   public:
    /** Separates end of file from a real failure. The two owners treat them oppositely: replay
     *  finishes normally at EndOfFile, a live camera can never see one. */
    enum class GrabStatus { Ok, TransientError, EndOfFile, Fatal };

    enum class OpenResult { Opened, Cancelled, Failed, Leaked };

    ZedDevice() = default;
    ZedDevice(const ZedDevice &) = delete;
    ZedDevice &operator=(const ZedDevice &) = delete;

    void set_params(const sl::InitParameters &params) { params_ = params; }
    sl::InitParameters &params() { return params_; }

    /**
     * Opens on a background thread so `cancel` can be polled while the SDK call blocks. Returns
     * Leaked when a cancelled open had to be abandoned, in which case the handle must not be
     * touched again.
     */
    OpenResult open(const std::atomic<bool> &cancel);

    /** Waits for an in-flight open, leaking the future on hard timeout. False means leaked. */
    bool await_or_leak_open(const char *context, const char *validation_label);
    bool has_pending_open() const { return pending_open_.valid(); }
    void close() { zed_.close(); }

    void set_tracking_enabled(bool enabled) { tracking_enabled_ = enabled; }
    bool tracking_enabled() const { return tracking_enabled_; }
    /** Enables positional tracking when configured. Closes the handle and returns false on
     *  failure, matching what a caller would otherwise have to do itself. */
    bool enable_tracking();

    /** Intrinsics and resolution, valid once open() has succeeded. */
    CameraInfo read_camera_info() const;

    /** One zed_.grab(). Kept separate from retrieve() so a live camera can roll its SVO recorder
     *  between the two, which is where the recorder needs to run. */
    GrabStatus grab();

    /**
     * Fills image, depth, pose, tracking_ok, and the raw capture stamp.
     *
     * Depth comes back on every frame. grab() computes it regardless (RuntimeParameters
     * ::enable_depth is always on), so retrieving it conditionally saved only the measure copy
     * while costing the callers a flag to get wrong.
     *
     * Leaves frame_identity.svo_frame_index and svo_path alone: a live camera takes them from the
     * file it is writing, a replay from the file it is reading. Stamps are the original capture
     * time, so any rebasing is the caller's business.
     */
    bool retrieve(CameraData &out);

    bool is_svo_input() const;
    int svo_frame_count() { return zed_.getSVONumberOfFrames(); }
    int svo_position() { return zed_.getSVOPosition(); }
    void set_svo_position(int frame) { zed_.setSVOPosition(frame); }

    /** For SvoRecorder, which needs the raw handle. */
    sl::Camera &handle() { return zed_; }

    /** Resets per-run logging state so a re-open does not inherit the old tracking state. */
    void reset_runtime_state() { prev_tracking_state_ = sl::POSITIONAL_TRACKING_STATE::LAST; }

   private:
    sl::Camera zed_;
    sl::InitParameters params_;
    sl::Mat zed_rgb_;
    sl::Mat zed_depth_;
    sl::Pose zed_pose_;
    std::future<sl::ERROR_CODE> pending_open_;
    bool tracking_enabled_ = true;
    sl::POSITIONAL_TRACKING_STATE prev_tracking_state_ = sl::POSITIONAL_TRACKING_STATE::LAST;
};

}  // namespace auto_battlebot
