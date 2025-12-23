#pragma once

#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <future>
#include <sl/Camera.hpp>

#include "rgbd_camera/rgbd_camera_interface.hpp"
#include "rgbd_camera/config.hpp"

namespace auto_battlebot
{
    inline sl::RESOLUTION get_zed_resolution(Resolution resolution)
    {
        switch (resolution)
        {
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

    inline sl::DEPTH_MODE get_zed_depth_mode(DepthMode depth_mode)
    {
        switch (depth_mode)
        {
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

    class ZedRgbdCamera : public RgbdCameraInterface
    {
    public:
        ZedRgbdCamera(ZedRgbdCameraConfiguration &config);
        ~ZedRgbdCamera();
        bool initialize() override;
        bool update() override;
        const CameraData &get() const override;
        bool should_close() override;

    private:
        void capture_thread_loop();
        bool capture_frame();

        sl::Camera zed_;
        sl::InitParameters params_;
        sl::Mat zed_rgb_;
        sl::Mat zed_depth_;
        sl::Pose zed_pose_;
        CameraData latest_data_;
        mutable std::mutex data_mutex_;
        mutable std::condition_variable data_cv_;
        std::thread capture_thread_;
        std::atomic<bool> is_initialized_;
        std::atomic<bool> should_close_;
        std::atomic<bool> stop_thread_;
        std::atomic<bool> has_new_frame_;
        std::atomic<uint64_t> frame_counter_;
        sl::POSITIONAL_TRACKING_STATE prev_tracking_state_;
        bool position_tracking_enabled_;
    };

} // namespace auto_battlebot
