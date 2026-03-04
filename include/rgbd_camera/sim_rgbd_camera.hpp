#pragma once

#include <chrono>
#include <memory>
#include <opencv2/core.hpp>
#include <thread>

#include "communication/sim_tcp_client.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"
#include "rgbd_camera/config.hpp"
#include "rgbd_camera/rgbd_camera_interface.hpp"

namespace auto_battlebot {
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
class SimRgbdCamera : public RgbdCameraInterface {
   public:
    SimRgbdCamera(SimRgbdCameraConfiguration &config);
    ~SimRgbdCamera();

    bool initialize() override;
    bool get(CameraData &data, bool get_quality_depth) override;
    bool should_close() override;

   private:
    std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;

    SimTcpClientConfig tcp_config_;
    cv::Size low_res_depth_size_;

    // Last received frame info
    uint64_t last_frame_id_ = 0;

    // Diagnostics tracking
    void log_stats();
    uint64_t frames_received_ = 0;
    std::chrono::steady_clock::time_point last_log_time_;

    // Helper to populate CameraData from TCP messages
    bool populate_camera_info(CameraData &data);
    void populate_pose(CameraData &data, const TcpFrameReadyWithDataMessage &frame);

    // Get frame with image data from TCP
    bool get_frame_with_data(CameraData &data, bool get_quality_depth);
    bool request_frame(bool get_quality_depth);
};

}  // namespace auto_battlebot
