#include "rgbd_camera/sim_rgbd_camera.hpp"
#include "time_utils.hpp"
#include "enums/frame_id.hpp"

#include <opencv2/opencv.hpp>

namespace auto_battlebot
{

    SimRgbdCamera::SimRgbdCamera(SimRgbdCameraConfiguration &config)
        : frames_received_(0),
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
        auto &client = SimTcpClient::instance();
        client.configure(tcp_config_);

        // Connect to Unity
        if (!client.is_connected())
        {
            if (!client.connect())
            {
                diagnostics_logger_->error("initialized",
                                           {{"host", tcp_config_.host},
                                            {"port", tcp_config_.port},
                                            {"success", false}});
                return false;
            }
        }

        diagnostics_logger_->info("initialized",
                                  {{"tcp_host", tcp_config_.host},
                                   {"tcp_port", tcp_config_.port},
                                   {"success", true}});

        // Reset stats on re-initialization
        frames_received_ = 0;
        last_log_time_ = std::chrono::steady_clock::now();

        return true;
    }

    bool SimRgbdCamera::get(CameraData &data, bool get_depth)
    {
        auto &client = SimTcpClient::instance();

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
        if (!populate_camera_info(data))
            return false;

        // Get frame with image data from TCP
        bool success = get_frame_with_data(data, get_depth);

        // Log stats periodically
        log_stats();

        return success;
    }

    bool SimRgbdCamera::get_frame_with_data(CameraData &data, bool get_depth)
    {
        auto &client = SimTcpClient::instance();

        // If depth is requested, send a frame request to Unity
        // This tells Unity to include depth data in the next frame
        if (get_depth)
        {
            if (!client.request_frame(true))
            {
                diagnostics_logger_->error("request_frame", {{"with_depth", true}, {"success", false}});
                return false;
            }
        }

        // Wait for frame with image data from Unity via TCP
        // When depth is requested, we may need to skip RGB-only frames that were
        // already in flight before our request was processed
        auto start_time = std::chrono::steady_clock::now();
        int timeout_ms = get_depth ? tcp_config_.read_timeout_ms * 3 : tcp_config_.read_timeout_ms;
        int frames_skipped = 0;
        const int max_frames_to_skip = 5;  // Don't skip forever

        std::optional<TcpFrameReadyWithDataMessage> frame;

        while (true)
        {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start_time).count();
            
            if (elapsed >= timeout_ms)
            {
                diagnostics_logger_->error("get_frame", {{"error", "timeout"},
                                                         {"get_depth", get_depth},
                                                         {"frames_skipped", frames_skipped}});
                return false;
            }

            int remaining_ms = timeout_ms - static_cast<int>(elapsed);
            frame = client.wait_for_frame_with_data(
                std::chrono::milliseconds(remaining_ms));

            if (!frame)
            {
                diagnostics_logger_->error("get_frame", {{"get_depth", get_depth}, {"success", false}});
                return false;
            }

            // If we requested depth, wait until we get a frame with depth data
            // (skip RGB-only frames that were in flight before our request)
            if (get_depth && frame->depth_data_size == 0)
            {
                frames_skipped++;
                if (frames_skipped >= max_frames_to_skip)
                {
                    diagnostics_logger_->error("get_frame", {{"error", "depth_not_received_after_skips"},
                                                             {"frames_skipped", frames_skipped}});
                    return false;
                }
                // Acknowledge this frame and wait for next one
                client.send_frame_processed(frame->frame_id);
                continue;
            }

            // Got a suitable frame
            break;
        }

        if (frames_skipped > 0)
        {
            diagnostics_logger_->debug("get_frame_skipped", {{"frames_skipped", frames_skipped}});
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
            cv::flip(rgba, rgba, 0);
            cv::cvtColor(rgba, data.rgb.image, cv::COLOR_RGBA2BGR);
        }
        else
        {
            diagnostics_logger_->error("get_frame", {{"error", "no_rgb_data"}});
            return false;
        }

        // Process depth image data if requested
        if (get_depth)
        {
            data.depth.header.stamp = frame->timestamp_seconds();
            data.depth.header.frame_id = FrameId::CAMERA;

            // Depth is float32 raw bytes
            cv::Mat depth_raw(frame->depth_height, frame->depth_width, CV_32FC1, frame->depth_data.data());
            cv::flip(depth_raw, depth_raw, 0);  // Flip to match RGB
            data.depth.image = depth_raw.clone();
        }

        // Acknowledge frame processed
        client.send_frame_processed(frame->frame_id);

        diagnostics_logger_->debug("get_frame", {{"frame_id", static_cast<int>(last_frame_id_)},
                                                 {"rgb_size", frame->rgb_data_size},
                                                 {"depth_size", frame->depth_data_size},
                                                 {"get_depth", get_depth}});
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
        return false;
    }

    bool SimRgbdCamera::populate_camera_info(CameraData &data)
    {
        auto intrinsics = SimTcpClient::instance().get_intrinsics();

        if (!intrinsics)
        {
            std::cerr << "No intrinsics found." << std::endl;
            return false;
        }
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
        return true;
    }

    void SimRgbdCamera::populate_pose(CameraData &data, const TcpFrameReadyWithDataMessage &frame)
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
