#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
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

class ZedDevice {
   public:
    enum class GrabStatus { Ok, TransientError, EndOfFile, Fatal };

    enum class OpenResult { Opened, Cancelled, Failed, Leaked };

    struct Options {
        std::chrono::milliseconds open_hard_timeout{30000};
        std::function<sl::ERROR_CODE(sl::Camera &, const sl::InitParameters &)> open_call =
            [](sl::Camera &camera, const sl::InitParameters &params) {
                return camera.open(params);
            };
    };

    ZedDevice() = default;
    explicit ZedDevice(Options options) : options_(std::move(options)) {}
    virtual ~ZedDevice() = default;
    ZedDevice(const ZedDevice &) = delete;
    ZedDevice &operator=(const ZedDevice &) = delete;

    void set_params(const sl::InitParameters &params) { params_ = params; }
    sl::InitParameters &params() { return params_; }

    virtual OpenResult open(const std::atomic<bool> &cancel);

    virtual bool await_or_leak_open(const char *context, const char *validation_label);
    virtual bool has_pending_open() const { return pending_open_.valid(); }
    virtual void close() { zed_.close(); }

    void set_tracking_enabled(bool enabled) { tracking_enabled_ = enabled; }
    bool tracking_enabled() const { return tracking_enabled_; }
    virtual bool enable_tracking();

    virtual std::optional<CameraInfo> read_camera_info() const;

    virtual GrabStatus grab();

    virtual bool retrieve(CameraData &out);

    bool is_svo_input() const;
    virtual int svo_frame_count() { return zed_.getSVONumberOfFrames(); }
    virtual int svo_position() { return zed_.getSVOPosition(); }
    virtual void set_svo_position(int frame) { zed_.setSVOPosition(frame); }

    sl::Camera &handle() { return zed_; }

    virtual void reset_runtime_state() {
        prev_tracking_state_ = sl::POSITIONAL_TRACKING_STATE::LAST;
    }

   private:
    Options options_;
    std::shared_ptr<sl::Camera> camera_ = std::make_shared<sl::Camera>();
    sl::Camera &zed_ = *camera_;
    sl::InitParameters params_;
    sl::Mat zed_rgb_;
    sl::Mat zed_depth_;

    struct BufferGeometry {
        size_t width = 0;
        size_t height = 0;
        size_t step_bytes = 0;
        uintptr_t data = 0;
        bool seen = false;
    };
    BufferGeometry rgb_geometry_;
    BufferGeometry depth_geometry_;

    void log_buffer_geometry(sl::Mat &mat, const char *label, size_t bytes_per_pixel,
                             BufferGeometry &previous);
    sl::Pose zed_pose_;
    std::future<sl::ERROR_CODE> pending_open_;
    bool tracking_enabled_ = true;
    sl::POSITIONAL_TRACKING_STATE prev_tracking_state_ = sl::POSITIONAL_TRACKING_STATE::LAST;
};

}  // namespace auto_battlebot
