#pragma once

#include <thread>
#include <chrono>

#include "rgbd_camera/rgbd_camera_interface.hpp"
#include "rgbd_camera/config.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"
#include "shared_memory/simulation/simulation_frame_header.hpp"
#include "shared_memory/shared_memory_reader.hpp"

// Include GPU memory share header
#include "gpu_memory_share/gpu_memory_share.h"

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
        /// Read frame data from shared memory (internal helper)
        bool read_frame_from_shared_memory(CameraData &data, bool get_depth) const;
        
        /// Read frame data from GPU memory via CUDA (internal helper)
        bool read_frame_from_gpu(CameraData &data, bool get_depth) const;

        std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;

        int expected_width_, expected_height_;
        bool enable_double_buffering_;
        bool enable_sync_socket_;
        int sync_timeout_ms_;
        bool enable_gpu_sharing_;

        size_t buffer_size_;
        std::unique_ptr<SharedMemoryReader> frame_reader_;
        
        // GPU sharing state (CUDA backend)
        mutable GpuShareTexture gpu_color_texture_{};
        mutable GpuShareTexture gpu_depth_texture_{};
        std::vector<uint8_t> gpu_color_buffer_;  // Temporary CPU buffer for GPU→CPU copy
        std::vector<float> gpu_depth_buffer_;    // Temporary CPU buffer for GPU→CPU copy

        // Diagnostics tracking (mutable for const get() method)
        mutable uint64_t frames_received_;
        mutable uint64_t last_frame_count_;
        mutable std::chrono::steady_clock::time_point last_log_time_;
    };

} // namespace auto_battlebot
