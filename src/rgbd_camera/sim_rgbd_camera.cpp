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
      last_log_time_(std::chrono::steady_clock::now())
{
    diagnostics_logger_ = DiagnosticsLogger::get_logger("sim_rgbd_camera");

    // Configure TCP client
    tcp_config_.host = config.tcp_host;
    tcp_config_.port = config.tcp_port;
    tcp_config_.connect_timeout_ms = config.tcp_connect_timeout_ms;
    tcp_config_.read_timeout_ms = config.tcp_read_timeout_ms;
    tcp_config_.auto_reconnect = config.tcp_auto_reconnect;
}

SimRgbdCamera::~SimRgbdCamera()
{
    // Note: We don't destroy the static TCP client here as it may be
    // shared with SimTransmitter. It will be cleaned up on program exit.
}

bool SimRgbdCamera::initialize()
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
            // Validate intrinsics match expected dimensions
            if (intrinsics->width != expected_width_ || intrinsics->height != expected_height_)
            {
                diagnostics_logger_->warning("intrinsics_dimension_mismatch",
                    {{"expected_width", expected_width_},
                     {"expected_height", expected_height_},
                     {"received_width", intrinsics->width},
                     {"received_height", intrinsics->height}});
            }
        }
    }

    diagnostics_logger_->info("initialized",
        {{"width", expected_width_},
         {"height", expected_height_},
         {"tcp_host", tcp_config_.host},
         {"tcp_port", tcp_config_.port}});

    // Reset stats on re-initialization
    frames_received_ = 0;
    last_log_time_ = std::chrono::steady_clock::now();

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

    // Wait for frame-ready message from Unity
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

    // Note: In the full CUDA Interop implementation, the RGB and depth
    // images would be accessed via shared GPU memory here.
    // For now, we just create placeholder images.
    data.rgb.header.timestamp = frame->timestamp_seconds();
    data.rgb.header.frame_id = frame->frame_id;
    data.rgb.data = cv::Mat(expected_height_, expected_width_, CV_8UC3, cv::Scalar(0, 0, 0));

    if (get_depth)
    {
        data.depth.header.timestamp = frame->timestamp_seconds();
        data.depth.header.frame_id = frame->frame_id;
        data.depth.data = cv::Mat(expected_height_, expected_width_, CV_32FC1, cv::Scalar(0.0f));
    }

    // Acknowledge frame processed
    tcp_client_->send_frame_processed(frame->frame_id);

    // Log stats periodically
    log_stats();

    return true;
}

void SimRgbdCamera::log_stats()
{
    // Track frame statistics
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time_).count();

    if (elapsed >= 1)
    {
        double fps = static_cast<double>(frames_received_) / elapsed;
        diagnostics_logger_->info("stats",
            {{"frames_received", static_cast<int>(frames_received_)},
             {"fps", fps},
             {"tcp_frames_total", static_cast<int>(tcp_client_ ? tcp_client_->get_frames_received() : 0)}});
        frames_received_ = 0;
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

    // Extract rotation matrix (top-left 3x3)
    tf.rotation.w() = 0;  // Will be computed from rotation matrix
    
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
