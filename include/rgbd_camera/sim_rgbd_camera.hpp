#pragma once

#include <thread>
#include <chrono>
#include <memory>

#include "rgbd_camera/rgbd_camera_interface.hpp"
#include "rgbd_camera/config.hpp"
#include "communication/sim_tcp_client.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"

namespace auto_battlebot
{
    /**
     * @brief Simulation RGBD camera with TCP image transfer
     *
     * This class implements RgbdCameraInterface for simulation mode.
     * Images are received via TCP from Unity using AsyncGPUReadback.
     *
     * Data flow (all via TCP):
     * - RGB and depth images (raw bytes)
     * - Pose data
     * - Camera intrinsics
     *
     * Uses the SimTcpClient singleton for communication.
     */
    class SimRgbdCamera : public RgbdCameraInterface
    {
    public:
        SimRgbdCamera(SimRgbdCameraConfiguration& config);
        ~SimRgbdCamera();

        bool initialize() override;
        bool get(CameraData& data, bool get_depth) override;
        bool should_close() override;

    private:
        std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;

        int expected_width_, expected_height_;
        SimTcpClientConfig tcp_config_;

        // Last received frame info
        uint64_t last_frame_id_ = 0;

        // Diagnostics tracking
        void log_stats();
        uint64_t frames_received_ = 0;
        std::chrono::steady_clock::time_point last_log_time_;

        // Helper to populate CameraData from TCP messages
        void populate_camera_info(CameraData& data);
        void populate_pose(CameraData& data, const TcpFrameReadyWithDataMessage& frame);

        // Get frame with image data from TCP
        bool get_frame_with_data(CameraData& data, bool get_depth);
    };

} // namespace auto_battlebot
