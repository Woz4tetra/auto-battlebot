#pragma once

#include <thread>
#include <chrono>
#include <memory>

#include "rgbd_camera/rgbd_camera_interface.hpp"
#include "rgbd_camera/config.hpp"
#include "communication/sim_tcp_client.hpp"
#include "communication/gpu_image_types.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"

namespace auto_battlebot
{
    /**
     * @brief Simulation RGBD camera with CUDA Interop support
     *
     * This class implements RgbdCameraInterface for simulation mode with
     * zero-copy GPU texture sharing via CUDA Interop.
     *
     * Data flow:
     * - TCP: Pose data, frame synchronization, camera intrinsics
     * - CUDA Interop: RGB and depth textures (GPU-resident)
     *
     * When CUDA Interop is available:
     * - GPU image data available via get_gpu_frame_data()
     * - CameraData gets placeholder images (empty cv::Mat)
     *
     * Fallback (CUDA Interop unavailable):
     * - Returns placeholder images
     * - Pose and intrinsics still available via TCP
     */
    class SimRgbdCamera : public RgbdCameraInterface
    {
    public:
        SimRgbdCamera(SimRgbdCameraConfiguration& config);
        ~SimRgbdCamera();

        bool initialize() override;
        bool get(CameraData& data, bool get_depth) override;
        bool should_close() override;

        /**
         * @brief Get the shared TCP client instance
         *
         * This allows SimTransmitter to share the same TCP connection.
         * Returns nullptr if not initialized.
         */
        static std::shared_ptr<SimTcpClient> get_tcp_client();

    private:
        std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;

        int expected_width_, expected_height_;
        SimTcpClientConfig tcp_config_;

        // Shared TCP client (static so SimTransmitter can access it)
        static std::shared_ptr<SimTcpClient> tcp_client_;
        static std::mutex tcp_client_mutex_;

        // Last received frame info
        uint64_t last_frame_id_ = 0;
        bool resources_mapped_ = false;

        // Diagnostics tracking
        void log_stats();
        uint64_t frames_received_;
        std::chrono::steady_clock::time_point last_log_time_;

        // Helper to populate CameraData from TCP messages
        void populate_camera_info(CameraData& data);
        void populate_pose(CameraData& data, const TcpFrameReadyMessage& frame);

        // CUDA Interop helpers
        bool get_frame(CameraData& data, bool get_depth, const TcpFrameReadyMessage& frame);
    };

} // namespace auto_battlebot
