#pragma once

#include <thread>
#include <chrono>

#include "rgbd_camera/rgbd_camera_interface.hpp"
#include "rgbd_camera/config.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"
#include "shared_memory/simulation/simulation_frame_header.hpp"
#include "shared_memory/shared_memory_reader.hpp"

namespace auto_battlebot
{
    class SimRgbdCamera : public RgbdCameraInterface
    {
    public:
        SimRgbdCamera(SimRgbdCameraConfiguration &config);
        ~SimRgbdCamera();
        bool initialize() override;
        bool get(CameraData &data, bool get_depth) const override;
        bool should_close() override;

    private:
        std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;

        int expected_width_, expected_height_;
        bool enable_double_buffering_;

        size_t buffer_size_;
        std::unique_ptr<SharedMemoryReader> frame_reader_;

        // Diagnostics tracking (mutable for const get() method)
        mutable uint64_t frames_received_;
        mutable uint64_t last_frame_id_;
        mutable std::chrono::steady_clock::time_point last_log_time_;
    };

} // namespace auto_battlebot
