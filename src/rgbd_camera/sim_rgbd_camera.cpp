#include "rgbd_camera/sim_rgbd_camera.hpp"
#include "communication/simulation_sync_manager.hpp"

// Include GPU memory share header
#include "gpu_memory_share/gpu_memory_share.h"

#ifdef CUDA_AVAILABLE
#include <cuda_runtime.h>
#include <opencv2/core/cuda.hpp>
#endif

namespace auto_battlebot
{
    SimRgbdCamera::SimRgbdCamera(SimRgbdCameraConfiguration &config)
        : expected_width_(config.width),
          expected_height_(config.height),
          enable_double_buffering_(config.enable_double_buffering),
          enable_sync_socket_(config.enable_sync_socket),
          sync_timeout_ms_(config.sync_timeout_ms),
          enable_gpu_sharing_(config.enable_gpu_sharing),
          frame_reader_(nullptr),
          frames_received_(0),
          last_frame_count_(0),
          last_log_time_(std::chrono::steady_clock::now())
    {
        buffer_size_ = get_simulation_frame_size(expected_width_, expected_height_);

        diagnostics_logger_ = DiagnosticsLogger::get_logger("sim_rgbd_camera");
        
        // Pre-allocate GPU copy buffers if GPU sharing is enabled
        if (enable_gpu_sharing_) {
            gpu_color_buffer_.resize(expected_width_ * expected_height_ * 3);  // BGR
            gpu_depth_buffer_.resize(expected_width_ * expected_height_);       // float
        }
    }

    SimRgbdCamera::~SimRgbdCamera()
    {
    }

    bool SimRgbdCamera::initialize()
    {
        diagnostics_logger_->info("config",
                                  {{"expected_width", expected_width_},
                                   {"expected_height", expected_height_},
                                   {"enable_double_buffering", enable_double_buffering_ ? 1 : 0},
                                   {"enable_sync_socket", enable_sync_socket_ ? 1 : 0},
                                   {"sync_timeout_ms", sync_timeout_ms_},
                                   {"buffer_size", static_cast<int>(buffer_size_)}},
                                  "Initializing SimRgbdCamera");

        // Initialize GPU sharing if enabled
        if (enable_gpu_sharing_)
        {
            GpuShareConfig gpu_config{};
            gpu_config.backend = GPU_SHARE_BACKEND_AUTO;  // Auto-detect CUDA/OpenGL
            gpu_config.timeout_ms = sync_timeout_ms_;
            
            GpuShareError init_err = GpuMemoryShare_Initialize(&gpu_config);
            if (init_err != GPU_SHARE_SUCCESS)
            {
                diagnostics_logger_->warning("GPU sharing initialization failed, falling back to shared memory");
                enable_gpu_sharing_ = false;
            }
            else
            {
                diagnostics_logger_->info("GPU sharing initialized successfully");
            }
        }
        
        // Initialize shared memory reader (fallback or primary path)
        if (!enable_gpu_sharing_)
        {
            if (frame_reader_ && frame_reader_->is_open())
            {
                std::cout << "Closing existing shared memory reader" << std::endl;
                frame_reader_->close();
            }

            size_t total_size;
            if (enable_double_buffering_)
                total_size = buffer_size_ * 2; // Double buffering
            else
                total_size = buffer_size_;

            diagnostics_logger_->info("shm",
                                      {{"total_size", static_cast<int>(total_size)},
                                       {"shm_name", "auto_battlebot_frame"}},
                                      "Opening shared memory");

            frame_reader_ = std::make_unique<SharedMemoryReader>("auto_battlebot_frame", total_size);

            bool success = frame_reader_->open();
            if (!success)
            {
                std::cerr << "Failed to open shared memory for SimRgbdCamera" << std::endl;
                return false;
            }
        }

        // Initialize sync socket if enabled
        if (enable_sync_socket_)
        {
            SyncSocketConfiguration sync_config;
            sync_config.timeout_ms = sync_timeout_ms_;

            if (!SimulationSyncManager::instance().initialize(sync_config))
            {
                diagnostics_logger_->warning("Failed to connect to Unity sync socket (will retry automatically). Running in free-running mode until connected.");
            }
            else
            {
                diagnostics_logger_->info("Connected to Unity sync socket");
            }
        }

        std::cout << "SimRgbdCamera initialized successfully" << std::endl;
        diagnostics_logger_->info("SimRgbdCamera initialized successfully");
        // Reset stats on re-initialization
        frames_received_ = 0;
        last_frame_count_ = 0;
        last_log_time_ = std::chrono::steady_clock::now();

        return true;
    }

    bool SimRgbdCamera::get(CameraData &data, bool get_depth) const
    {
        data = CameraData();

        // GPU sharing path
        if (enable_gpu_sharing_)
        {
            return read_frame_from_gpu(data, get_depth);
        }

        // Shared memory path (fallback or primary)
        if (!frame_reader_ || !frame_reader_->is_open())
        {
            diagnostics_logger_->warning("Shared memory not open");
            return false;
        }

        // If sync socket is enabled, request frame and wait
        if (enable_sync_socket_ && SimulationSyncManager::instance().is_connected())
        {
            // Request frame from Unity with depth flag
            if (!SimulationSyncManager::instance().request_frame(get_depth))
            {
                diagnostics_logger_->warning("Failed to send frame request to Unity");
                return false;
            }

            // Wait for Unity to signal frame ready
            bool has_depth = false;
            if (!SimulationSyncManager::instance().wait_for_frame(sync_timeout_ms_, &has_depth))
            {
                diagnostics_logger_->warning("Timeout waiting for frame from Unity");
                return false;
            }

            // Verify we got depth if we requested it
            if (get_depth && !has_depth)
            {
                diagnostics_logger_->warning("Requested depth but frame has no depth data");
                // Continue anyway - depth will be empty
            }
        }

        // Read frame from shared memory
        return read_frame_from_shared_memory(data, get_depth);
    }

    bool SimRgbdCamera::read_frame_from_shared_memory(CameraData &data, bool get_depth) const
    {
        // Determine which buffer to read from (for double buffering)
        // Each buffer has its own header. We need to find the most recently written buffer.
        size_t buffer_offset = 0;
        if (enable_double_buffering_)
        {
            // Read headers from both buffers and pick the one with higher frame_id
            const auto *header0 = frame_reader_->read_at<SimulationFrameHeader>(0);
            const auto *header1 = frame_reader_->read_at<SimulationFrameHeader>(buffer_size_);

            // Validate both headers have reasonable dimensions
            bool header0_valid = (header0->width > 0 && header0->width <= 4096 &&
                                  header0->height > 0 && header0->height <= 4096);
            bool header1_valid = (header1->width > 0 && header1->width <= 4096 &&
                                  header1->height > 0 && header1->height <= 4096);

            if (header0_valid && header1_valid)
            {
                // Both valid - pick the one with higher frame_id
                buffer_offset = (header1->frame_counter > header0->frame_counter) ? buffer_size_ : 0;
            }
            else if (header0_valid)
            {
                buffer_offset = 0;
            }
            else if (header1_valid)
            {
                buffer_offset = buffer_size_;
            }
            // else: both invalid, use buffer 0 (default)
        }

        const auto *active_header = frame_reader_->read_at<SimulationFrameHeader>(buffer_offset);

        // Log raw header values for debugging
        diagnostics_logger_->debug("header",
                                   {{"frame_id", static_cast<int>(active_header->frame_counter)},
                                    {"num_received", static_cast<int>(frames_received_)},
                                    {"timestamp", active_header->timestamp},
                                    {"width", active_header->width},
                                    {"height", active_header->height},
                                    {"buffer_offset", static_cast<int>(buffer_offset)},
                                    {"rgb_offset", active_header->rgb_offset},
                                    {"depth_offset", active_header->depth_offset},
                                    {"pose_offset", active_header->pose_offset}});

        if (active_header->width <= 0 || active_header->height <= 0)
        {
            diagnostics_logger_->warning("dimensions",
                                         {{"width", active_header->width},
                                          {"height", active_header->height}},
                                         "Invalid frame header dimensions (zero or negative)");
            return false;
        }
        if (active_header->width != expected_width_ || active_header->height != expected_height_)
        {
            diagnostics_logger_->warning("dimensions",
                                         {{"received_width", active_header->width},
                                          {"received_height", active_header->height},
                                          {"expected_width", expected_width_},
                                          {"expected_height", expected_height_}},
                                         "Frame dimensions mismatch");
            return false;
        }

        double stamp = active_header->timestamp;

        Header header;
        header.stamp = stamp;
        header.frame_id = FrameId::CAMERA;

        const size_t rgb_size = static_cast<size_t>(expected_width_) * expected_height_ * 3;
        const size_t depth_size = static_cast<size_t>(expected_width_) * expected_height_ * sizeof(float);
        const size_t pose_size = 128;

        auto is_offset_valid = [&](int32_t offset, size_t size)
        {
            if (offset < static_cast<int32_t>(SimulationFrameHeader::SIZE))
            {
                return false;
            }
            return static_cast<size_t>(offset) + size <= buffer_size_;
        };

        if (!is_offset_valid(active_header->rgb_offset, rgb_size) ||
            !is_offset_valid(active_header->pose_offset, pose_size) ||
            (get_depth && !is_offset_valid(active_header->depth_offset, depth_size)))
        {
            diagnostics_logger_->warning("Invalid frame header offsets");
            return false;
        }

        // Parse color image (cv::Mat takes rows, cols = height, width)
        const uint8_t *rgb = frame_reader_->read_at<uint8_t>(buffer_offset + active_header->rgb_offset);
        cv::Mat color_image(expected_height_, expected_width_, CV_8UC3, const_cast<void *>(static_cast<const void *>(rgb)));
        data.rgb.header = header;
        data.rgb.image = color_image;

        // Parse depth image (cv::Mat takes rows, cols = height, width)
        if (get_depth)
        {
            const float *depth = frame_reader_->read_at<float>(buffer_offset + active_header->depth_offset);
            cv::Mat depth_image(expected_height_, expected_width_, CV_32FC1, const_cast<void *>(static_cast<const void *>(depth)));
            data.depth.image = depth_image;
            data.depth.header = header;
        }

        // Parse "visual SLAM" pose (4x4 row-major matrix)
        const double *pose = frame_reader_->read_at<double>(buffer_offset + active_header->pose_offset);
        TransformStamped tf_visodom_from_camera;
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                tf_visodom_from_camera.transform.tf(i, j) = pose[i * 4 + j];
            }
        }
        tf_visodom_from_camera.header.stamp = stamp;
        tf_visodom_from_camera.header.frame_id = FrameId::VISUAL_ODOMETRY;
        tf_visodom_from_camera.child_frame_id = FrameId::CAMERA;
        data.tf_visodom_from_camera = tf_visodom_from_camera;

        // Set intrinsics matrix from header (fx, 0, cx; 0, fy, cy; 0, 0, 1)
        // Intrinsics are stored directly in the header, not as offsets
        CameraInfo camera_info;
        camera_info.width = expected_width_;
        camera_info.height = expected_height_;
        camera_info.intrinsics = cv::Mat::eye(3, 3, CV_64F);
        camera_info.intrinsics.at<double>(0, 0) = active_header->fx;
        camera_info.intrinsics.at<double>(1, 1) = active_header->fy;
        camera_info.intrinsics.at<double>(0, 2) = active_header->cx;
        camera_info.intrinsics.at<double>(1, 2) = active_header->cy;

        // Set distortion coefficients from header
        camera_info.distortion = cv::Mat::zeros(1, 5, CV_64F);
        camera_info.distortion.at<double>(0, 0) = active_header->k1;
        camera_info.distortion.at<double>(0, 1) = active_header->k2;
        camera_info.distortion.at<double>(0, 2) = active_header->p1;
        camera_info.distortion.at<double>(0, 3) = active_header->p2;
        camera_info.distortion.at<double>(0, 4) = active_header->k3;
        camera_info.header = header;

        data.camera_info = camera_info;

        // Track frame statistics
        frames_received_++;
        bool is_new_frame = (active_header->frame_counter != last_frame_count_);
        last_frame_count_ = active_header->frame_counter;

        // Periodic stats logging (every 1 second)
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time_).count();
        if (elapsed >= 1)
        {
            double fps = static_cast<double>(frames_received_) / elapsed;
            diagnostics_logger_->info("stats",
                                      {{"frames_received", static_cast<int>(frames_received_)},
                                       {"fps", fps},
                                       {"last_frame_id", static_cast<int>(last_frame_count_)},
                                       {"last_timestamp", stamp},
                                       {"is_new_frame", is_new_frame ? 1 : 0}});
            diagnostics_logger_->info("intrinsics",
                                      {{"fx", active_header->fx},
                                       {"fy", active_header->fy},
                                       {"cx", active_header->cx},
                                       {"cy", active_header->cy},
                                       {"k1", active_header->k1},
                                       {"k2", active_header->k2},
                                       {"p1", active_header->p1},
                                       {"p2", active_header->p2},
                                       {"k3", active_header->k3}});
            frames_received_ = 0;
            last_log_time_ = now;
        }

        return true;
    }

    bool SimRgbdCamera::read_frame_from_gpu(CameraData &data, bool get_depth) const
    {
        // Wait for Unity to signal frame ready via GPU share
        GpuShareFrameInfo frame_info;
        GpuShareError wait_err = GpuMemoryShare_WaitForFrame(sync_timeout_ms_, &frame_info);
        if (wait_err != GPU_SHARE_SUCCESS)
        {
            if (wait_err == GPU_SHARE_ERROR_TIMEOUT)
            {
                diagnostics_logger_->warning("Timeout waiting for GPU frame from Unity");
            }
            else
            {
                diagnostics_logger_->warning("Failed to wait for GPU frame");
            }
            return false;
        }

        // Look up texture handles by name (Unity registers with "auto_battlebot_color" and "auto_battlebot_depth")
        GpuShareError color_lookup = GpuMemoryShare_GetTextureByName("auto_battlebot_color", &gpu_color_texture_);
        if (color_lookup != GPU_SHARE_SUCCESS)
        {
            diagnostics_logger_->warning("Failed to find GPU color texture by name");
            return false;
        }
        
        // Copy color from GPU to CPU
        GpuShareError color_err = GpuMemoryShare_CopyToCpu(
            &gpu_color_texture_,
            gpu_color_buffer_.data(),
            gpu_color_buffer_.size());
        
        if (color_err != GPU_SHARE_SUCCESS)
        {
            diagnostics_logger_->warning("Failed to copy color from GPU");
            return false;
        }

        // Create OpenCV Mat from BGR buffer (Unity writes BGR format)
        cv::Mat color_image(expected_height_, expected_width_, CV_8UC3, gpu_color_buffer_.data());
        data.rgb.image = color_image.clone();  // Clone to own the data
        data.rgb.header.stamp = frame_info.timestamp;
        data.rgb.header.frame_id = FrameId::CAMERA;

        // Copy depth if requested and available
        if (get_depth && frame_info.has_depth)
        {
            GpuShareError depth_lookup = GpuMemoryShare_GetTextureByName("auto_battlebot_depth", &gpu_depth_texture_);
            if (depth_lookup == GPU_SHARE_SUCCESS)
            {
                GpuShareError depth_err = GpuMemoryShare_CopyToCpu(
                    &gpu_depth_texture_,
                    gpu_depth_buffer_.data(),
                    gpu_depth_buffer_.size() * sizeof(float));
            
                if (depth_err == GPU_SHARE_SUCCESS)
                {
                    cv::Mat depth_image(expected_height_, expected_width_, CV_32FC1, gpu_depth_buffer_.data());
                    data.depth.image = depth_image.clone();
                    data.depth.header.stamp = frame_info.timestamp;
                    data.depth.header.frame_id = FrameId::CAMERA;
                }
                else
                {
                    diagnostics_logger_->warning("Failed to copy depth from GPU");
                }
            }
            else
            {
                diagnostics_logger_->warning("Failed to find GPU depth texture by name");
            }
        }

        // TODO: Get intrinsics/pose from GPU share metadata or separate API
        // For now, use defaults (caller should set these separately)
        CameraInfo camera_info;
        camera_info.width = expected_width_;
        camera_info.height = expected_height_;
        camera_info.intrinsics = cv::Mat::eye(3, 3, CV_64F);
        camera_info.distortion = cv::Mat::zeros(1, 5, CV_64F);
        camera_info.header = data.rgb.header;
        data.camera_info = camera_info;

        // TODO: Get pose from GPU share metadata
        data.tf_visodom_from_camera = TransformStamped{};
        data.tf_visodom_from_camera.header.stamp = frame_info.timestamp;
        data.tf_visodom_from_camera.header.frame_id = FrameId::VISUAL_ODOMETRY;
        data.tf_visodom_from_camera.child_frame_id = FrameId::CAMERA;

        frames_received_++;
        return true;
    }

    bool SimRgbdCamera::should_close()
    {
        return false;
    }
} // namespace auto_battlebot
