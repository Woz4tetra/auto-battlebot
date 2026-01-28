#pragma once

#include <thread>
#include <chrono>

#include "rgbd_camera/rgbd_camera_interface.hpp"
#include "rgbd_camera/config.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"

namespace auto_battlebot
{
    class SimRgbdCamera : public RgbdCameraInterface
    {
    public:
        SimRgbdCamera(SimRgbdCameraConfiguration &config);
        ~SimRgbdCamera();
        bool initialize() override;
        bool get(CameraData &data, bool get_depth) override;
        bool should_close() override;

    private:
        std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;

        int expected_width_, expected_height_;

        // Diagnostics tracking
        void log_stats();
        uint64_t frames_received_;
        std::chrono::steady_clock::time_point last_log_time_;
    };

} // namespace auto_battlebot
