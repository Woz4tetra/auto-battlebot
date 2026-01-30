#include "rgbd_camera/sim_rgbd_camera.hpp"
#include "time_utils.hpp"

#include <opencv2/opencv.hpp>

namespace auto_battlebot
{

// Static member definitions
std::shared_ptr<SimTcpClient> SimRgbdCamera::tcp_client_ = nullptr;
std::mutex SimRgbdCamera::tcp_client_mutex_;

SimRgbdCamera::SimRgbdCamera(SimRgbdCameraConfiguration& config)
    : expected_width_(config.width),
      expected_height_(config.height),
      frames_received_(0),
      gpu_frames_received_(0),
      last_log_time_(std::chrono::steady_clock::now())
{
    diagnostics_logger_ = DiagnosticsLogger::get_logger("sim_rgbd_camera");

    // Configure TCP client
    tcp_config_.host = config.tcp_host;
    tcp_config_.port = config.tcp_port;
    tcp_config_.connect_timeout_ms = config.tcp_connect_timeout_ms;
    tcp_config_.read_timeout_ms = config.tcp_read_timeout_ms;
    tcp_config_.auto_reconnect = config.tcp_auto_reconnect;

    // CUDA Interop config
    cuda_config_.cuda_device_id = 0;
    cuda_config_.enable_metrics = true;
}

SimRgbdCamera::~SimRgbdCamera()
{
    // Clear GPU frame data first (invalidates cuda_array pointers)
    last_gpu_frame_ = GpuFrameData{};

    // Ensure resources are unmapped before destruction
    if (cuda_interop_ && resources_mapped_)
    {
        cuda_interop_->unmap_resources();
        resources_mapped_ = false;
    }

    // Note: We don't destroy the static TCP client here as it may be
    // shared with SimTransmitter. It will be cleaned up on program exit.
}

bool SimRgbdCamera::initialize()
{
    // Initialize TCP client
    {
        std::lock_guard<std::mutex> lock(tcp_client_mutex_);

        // Create TCP client if not already created
        if (!tcp_client_)
        {
            tcp_client_ = std::make_shared<SimTcpClient>(tcp_config_);
        }

        // Connect to Unity
        if (!tcp_client_->is_connected())
        {
            if (!tcp_client_->connect())
            {
                diagnostics_logger_->error("tcp_connect_failed",
                    {{"host", tcp_config_.host},
                     {"port", tcp_config_.port}});
                return false;
            }
        }

        // Verify we received intrinsics
        if (!tcp_client_->has_intrinsics())
        {
            diagnostics_logger_->warning("no_intrinsics_received", {});
        }
        else
        {
            auto intrinsics = tcp_client_->get_intrinsics();
            if (intrinsics)
            {
                // Update expected dimensions from intrinsics
                if (intrinsics->width != expected_width_ || intrinsics->height != expected_height_)
                {
                    diagnostics_logger_->info("using_intrinsics_dimensions",
                        {{"width", intrinsics->width},
                         {"height", intrinsics->height}});
                    expected_width_ = intrinsics->width;
                    expected_height_ = intrinsics->height;
                }
            }
        }
    }

    // Try to initialize CUDA Interop (optional - will fall back to CPU if not available)
    cuda_interop_enabled_ = try_initialize_cuda_interop();

    diagnostics_logger_->info("initialized",
        {{"width", expected_width_},
         {"height", expected_height_},
         {"tcp_host", tcp_config_.host},
         {"tcp_port", tcp_config_.port},
         {"cuda_interop_enabled", cuda_interop_enabled_}});

    // Reset stats on re-initialization
    frames_received_ = 0;
    gpu_frames_received_ = 0;
    last_log_time_ = std::chrono::steady_clock::now();

    return true;
}

bool SimRgbdCamera::try_initialize_cuda_interop()
{
    cuda_interop_ = std::make_unique<CudaInteropWrapper>(cuda_config_);

    if (!cuda_interop_->initialize())
    {
        diagnostics_logger_->info("cuda_interop_unavailable",
            {{"fallback", "Using CPU image transfer"}});
        cuda_interop_.reset();
        return false;
    }

    diagnostics_logger_->info("cuda_interop_initialized",
        {{"device_id", cuda_config_.cuda_device_id}});
    return true;
}

bool SimRgbdCamera::get(CameraData& data, bool get_depth)
{
    if (!tcp_client_ || !tcp_client_->is_connected())
    {
        // Try to reconnect
        if (tcp_client_ && tcp_config_.auto_reconnect)
        {
            if (!tcp_client_->try_reconnect())
            {
                return false;
            }
        }
        else
        {
            return false;
        }
    }

    // Wait for frame-ready message from Unity via TCP
    auto frame = tcp_client_->wait_for_frame(
        std::chrono::milliseconds(tcp_config_.read_timeout_ms));

    if (!frame)
    {
        // Timeout or error
        return false;
    }

    last_frame_id_ = frame->frame_id;
    frames_received_++;

    // Populate camera info from intrinsics
    populate_camera_info(data);

    // Populate pose from frame message
    populate_pose(data, *frame);

    // Get image data - try GPU path first, fall back to CPU
    bool success = false;

    if (cuda_interop_enabled_)
    {
        success = get_gpu_frame(data, get_depth);
        if (success)
        {
            gpu_frames_received_++;
        }
    }

    if (!success)
    {
        // Fall back to CPU placeholder images
        success = get_cpu_frame(data, get_depth, *frame);
    }

    // Acknowledge frame processed
    tcp_client_->send_frame_processed(frame->frame_id);

    // Log stats periodically
    log_stats();

    return success;
}

bool SimRgbdCamera::get_gpu_frame(CameraData& data, bool get_depth)
{
    if (!cuda_interop_)
    {
        return false;
    }

    // Clear previous GPU frame data
    last_gpu_frame_ = GpuFrameData{};

    // Wait for GPU frame from CUDA Interop
    auto frame_info = cuda_interop_->wait_for_frame(tcp_config_.read_timeout_ms);
    if (!frame_info)
    {
        return false;
    }

    // Map resources for CUDA access
    if (!cuda_interop_->map_resources())
    {
        diagnostics_logger_->warning("gpu_map_failed", {});
        return false;
    }
    resources_mapped_ = true;

    // Get RGB CUDA array
    void* rgb_cuda = cuda_interop_->get_cuda_array(CudaInteropTextureType::RGB);
    if (rgb_cuda && frame_info->rgb_valid)
    {
        // Populate GPU frame data
        last_gpu_frame_.rgb.cuda_array = rgb_cuda;
        last_gpu_frame_.rgb.width = frame_info->rgb_width;
        last_gpu_frame_.rgb.height = frame_info->rgb_height;
        last_gpu_frame_.rgb.frame_id = frame_info->frame_id;
        last_gpu_frame_.rgb.timestamp = frame_info->timestamp;

        // Populate CameraData with placeholder (header only, no image data)
        data.rgb = last_gpu_frame_.rgb.to_placeholder();
    }
    else
    {
        // RGB not available
        cuda_interop_->unmap_resources();
        resources_mapped_ = false;
        return false;
    }

    // Get Depth CUDA array (if requested)
    if (get_depth)
    {
        void* depth_cuda = cuda_interop_->get_cuda_array(CudaInteropTextureType::Depth);
        if (depth_cuda && frame_info->depth_valid)
        {
            // Populate GPU frame data
            last_gpu_frame_.depth.cuda_array = depth_cuda;
            last_gpu_frame_.depth.width = frame_info->depth_width;
            last_gpu_frame_.depth.height = frame_info->depth_height;
            last_gpu_frame_.depth.frame_id = frame_info->frame_id;
            last_gpu_frame_.depth.timestamp = frame_info->timestamp;

            // Populate CameraData with placeholder
            data.depth = last_gpu_frame_.depth.to_placeholder();
        }
    }

    // Note: Resources will be unmapped after inference completes
    // The caller is responsible for accessing GPU data via get_gpu_frame_data()
    // before the next get() call which will unmap resources

    return true;
}

bool SimRgbdCamera::get_cpu_frame(CameraData& data, bool get_depth, const TcpFrameReadyMessage& frame)
{
    // Create placeholder CPU images
    // In a full implementation, these would be transferred from Unity via TCP
    // or the CPU fallback path of the CUDA Interop plugin

    data.rgb.is_gpu_resident = false;
    data.rgb.cuda_array = nullptr;
    data.rgb.header.timestamp = frame.timestamp_seconds();
    data.rgb.header.frame_id = frame.frame_id;
    data.rgb.width = expected_width_;
    data.rgb.height = expected_height_;

    // Create black placeholder image
    data.rgb.image = cv::Mat(expected_height_, expected_width_, CV_8UC3, cv::Scalar(0, 0, 0));

    if (get_depth)
    {
        data.depth.is_gpu_resident = false;
        data.depth.cuda_array = nullptr;
        data.depth.header.timestamp = frame.timestamp_seconds();
        data.depth.header.frame_id = frame.frame_id;
        data.depth.width = expected_width_;
        data.depth.height = expected_height_;

        // Create zero depth placeholder
        data.depth.image = cv::Mat(expected_height_, expected_width_, CV_32FC1, cv::Scalar(0.0f));
    }

    return true;
}

void SimRgbdCamera::log_stats()
{
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time_).count();

    if (elapsed >= 1)
    {
        double fps = static_cast<double>(frames_received_) / elapsed;
        double gpu_ratio = frames_received_ > 0 ?
            static_cast<double>(gpu_frames_received_) / frames_received_ : 0.0;

        diagnostics_logger_->info("stats",
            {{"frames_received", static_cast<int>(frames_received_)},
             {"gpu_frames", static_cast<int>(gpu_frames_received_)},
             {"gpu_ratio", gpu_ratio},
             {"fps", fps}});

        // Log CUDA Interop metrics if available
        if (cuda_interop_enabled_ && cuda_interop_)
        {
            auto metrics = cuda_interop_->get_metrics();
            diagnostics_logger_->debug("cuda_metrics",
                {{"avg_map_ms", metrics.avg_map_time_ms},
                 {"avg_unmap_ms", metrics.avg_unmap_time_ms},
                 {"total_frames", static_cast<int>(metrics.total_frames)},
                 {"map_errors", static_cast<int>(metrics.map_errors)}});
        }

        frames_received_ = 0;
        gpu_frames_received_ = 0;
        last_log_time_ = now;
    }
}

bool SimRgbdCamera::should_close()
{
    // Check if TCP connection is still alive
    if (tcp_client_ && !tcp_client_->is_connected())
    {
        return true;
    }
    return false;
}

std::shared_ptr<SimTcpClient> SimRgbdCamera::get_tcp_client()
{
    std::lock_guard<std::mutex> lock(tcp_client_mutex_);
    return tcp_client_;
}

void SimRgbdCamera::populate_camera_info(CameraData& data)
{
    auto intrinsics = tcp_client_->get_intrinsics();

    if (intrinsics)
    {
        data.camera_info.width = intrinsics->width;
        data.camera_info.height = intrinsics->height;

        // Create 3x3 intrinsic matrix (OpenCV format)
        data.camera_info.intrinsics = cv::Mat::eye(3, 3, CV_64F);
        data.camera_info.intrinsics.at<double>(0, 0) = intrinsics->fx;
        data.camera_info.intrinsics.at<double>(1, 1) = intrinsics->fy;
        data.camera_info.intrinsics.at<double>(0, 2) = intrinsics->cx;
        data.camera_info.intrinsics.at<double>(1, 2) = intrinsics->cy;

        // Create distortion coefficients (k1, k2, p1, p2, k3)
        data.camera_info.distortion = cv::Mat(1, 5, CV_64F);
        data.camera_info.distortion.at<double>(0, 0) = intrinsics->k1;
        data.camera_info.distortion.at<double>(0, 1) = intrinsics->k2;
        data.camera_info.distortion.at<double>(0, 2) = intrinsics->p1;
        data.camera_info.distortion.at<double>(0, 3) = intrinsics->p2;
        data.camera_info.distortion.at<double>(0, 4) = intrinsics->k3;
    }
    else
    {
        // Use expected dimensions with default intrinsics
        data.camera_info.width = expected_width_;
        data.camera_info.height = expected_height_;

        // Default intrinsics (approximately 90 degree FOV)
        double fx = expected_width_ / 2.0;
        double fy = expected_width_ / 2.0;
        double cx = expected_width_ / 2.0;
        double cy = expected_height_ / 2.0;

        data.camera_info.intrinsics = cv::Mat::eye(3, 3, CV_64F);
        data.camera_info.intrinsics.at<double>(0, 0) = fx;
        data.camera_info.intrinsics.at<double>(1, 1) = fy;
        data.camera_info.intrinsics.at<double>(0, 2) = cx;
        data.camera_info.intrinsics.at<double>(1, 2) = cy;

        // No distortion
        data.camera_info.distortion = cv::Mat::zeros(1, 5, CV_64F);
    }
}

void SimRgbdCamera::populate_pose(CameraData& data, const TcpFrameReadyMessage& frame)
{
    // Convert pose array to Transform
    // Pose is 4x4 matrix in row-major order
    Transform tf;

    // Create rotation matrix from pose
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix(0, 0) = frame.pose[0];
    rotation_matrix(0, 1) = frame.pose[1];
    rotation_matrix(0, 2) = frame.pose[2];
    rotation_matrix(1, 0) = frame.pose[4];
    rotation_matrix(1, 1) = frame.pose[5];
    rotation_matrix(1, 2) = frame.pose[6];
    rotation_matrix(2, 0) = frame.pose[8];
    rotation_matrix(2, 1) = frame.pose[9];
    rotation_matrix(2, 2) = frame.pose[10];

    // Convert rotation matrix to quaternion
    tf.rotation = Eigen::Quaterniond(rotation_matrix);

    // Extract translation (4th column)
    tf.translation.x() = frame.pose[3];
    tf.translation.y() = frame.pose[7];
    tf.translation.z() = frame.pose[11];

    data.tf_visodom_from_camera.transform = tf;
    data.tf_visodom_from_camera.header.timestamp = frame.timestamp_seconds();
    data.tf_visodom_from_camera.header.frame_id = frame.frame_id;
}

} // namespace auto_battlebot
