#include "rgbd_camera/sim_rgbd_camera.hpp"
#include "time_utils.hpp"
#include "enums/frame_id.hpp"

#include <opencv2/opencv.hpp>

namespace auto_battlebot
{

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
    // Singleton handles cleanup
}

bool SimRgbdCamera::initialize()
{
    // Configure and connect to the singleton TCP client
    auto& client = SimTcpClient::instance();
    client.configure(tcp_config_);

    // Connect to Unity
    if (!client.is_connected())
    {
        if (!client.connect())
        {
            diagnostics_logger_->error("tcp_connect_failed",
                {{"host", tcp_config_.host},
                 {"port", tcp_config_.port}});
            return false;
        }
    }

    // Verify we received intrinsics
    if (!client.has_intrinsics())
    {
        diagnostics_logger_->warning("no_intrinsics_received", "No intrinsics received from server");
    }
    else
    {
        auto intrinsics = client.get_intrinsics();
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
    auto& client = SimTcpClient::instance();
    
    if (!client.is_connected())
    {
        // Try to reconnect
        if (tcp_config_.auto_reconnect)
        {
            if (!client.try_reconnect())
            {
                return false;
            }
        }
        else
        {
            return false;
        }
    }

    // Populate camera info from intrinsics
    populate_camera_info(data);

    // Get frame with image data from TCP
    bool success = get_frame_with_data(data, get_depth);

    // Log stats periodically
    log_stats();

    return success;
}

bool SimRgbdCamera::get_frame_with_data(CameraData& data, bool get_depth)
{
    auto& client = SimTcpClient::instance();
    
    // Wait for frame with image data from Unity via TCP
    auto frame = client.wait_for_frame_with_data(
        std::chrono::milliseconds(tcp_config_.read_timeout_ms));

    if (!frame)
    {
        // Timeout or error
        return false;
    }

    last_frame_id_ = frame->frame_id;
    frames_received_++;

    // Populate pose from frame message
    populate_pose(data, *frame);

    // Process RGB image data
    data.rgb.header.stamp = frame->timestamp_seconds();
    data.rgb.header.frame_id = FrameId::CAMERA;

    if (frame->rgb_data_size > 0 && frame->rgb_width > 0 && frame->rgb_height > 0)
    {
        // Convert RGBA raw bytes to BGR cv::Mat
        cv::Mat rgba(frame->rgb_height, frame->rgb_width, CV_8UC4, frame->rgb_data.data());
        cv::cvtColor(rgba, data.rgb.image, cv::COLOR_RGBA2BGR);
        
        // Update expected dimensions
        expected_width_ = frame->rgb_width;
        expected_height_ = frame->rgb_height;
    }
    else
    {
        // No image data - create placeholder
        data.rgb.image = cv::Mat(expected_height_, expected_width_, CV_8UC3, cv::Scalar(0, 0, 0));
    }

    // Process depth image data
    if (get_depth)
    {
        data.depth.header.stamp = frame->timestamp_seconds();
        data.depth.header.frame_id = FrameId::CAMERA;

        if (frame->depth_data_size > 0 && frame->depth_width > 0 && frame->depth_height > 0)
        {
            // Depth is float32 raw bytes
            cv::Mat depth_raw(frame->depth_height, frame->depth_width, CV_32FC1, frame->depth_data.data());
            data.depth.image = depth_raw.clone();
        }
        else
        {
            // No depth data - create placeholder
            data.depth.image = cv::Mat(expected_height_, expected_width_, CV_32FC1, cv::Scalar(0.0f));
        }
    }

    // Acknowledge frame processed
    client.send_frame_processed(frame->frame_id);

    return true;
}

void SimRgbdCamera::log_stats()
{
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time_).count();

    if (elapsed >= 1)
    {
        double fps = static_cast<double>(frames_received_) / elapsed;

        diagnostics_logger_->info("stats",
            {{"frames_received", static_cast<int>(frames_received_)},
             {"fps", fps}});

        frames_received_ = 0;
        last_log_time_ = now;
    }
}

bool SimRgbdCamera::should_close()
{
    // Check if TCP connection is still alive
    return !SimTcpClient::instance().is_connected();
}

void SimRgbdCamera::populate_camera_info(CameraData& data)
{
    auto intrinsics = SimTcpClient::instance().get_intrinsics();

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

void SimRgbdCamera::populate_pose(CameraData& data, const TcpFrameReadyWithDataMessage& frame)
{
    // Convert pose array to Transform
    // Pose is 4x4 matrix in row-major order
    Transform tf;
    tf.tf = Eigen::Matrix4d::Identity();

    // Copy 4x4 matrix from row-major pose array
    for (int row = 0; row < 4; row++)
    {
        for (int col = 0; col < 4; col++)
        {
            tf.tf(row, col) = frame.pose[row * 4 + col];
        }
    }

    data.tf_visodom_from_camera.transform = tf;
    data.tf_visodom_from_camera.header.stamp = frame.timestamp_seconds();
    data.tf_visodom_from_camera.header.frame_id = FrameId::VISUAL_ODOMETRY;
    data.tf_visodom_from_camera.child_frame_id = FrameId::CAMERA;
}

} // namespace auto_battlebot
