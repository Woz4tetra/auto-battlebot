#include "rgbd_camera/sim_rgbd_camera.hpp"

namespace auto_battlebot
{
    SimRgbdCamera::SimRgbdCamera(SimRgbdCameraConfiguration &config) : expected_width_(config.width),
                                                                       expected_height_(config.height),
                                                                       enable_double_buffering_(config.enable_double_buffering)
    {
        // Frame shared memory: header + RGB + depth + pose
        size_t rgb_size = expected_width_ * expected_height_ * 3;
        size_t depth_size = expected_width_ * expected_height_ * sizeof(float);
        size_t pose_size = 128; // 4x4 double matrix
        size_t intrinsics_size = sizeof(double);
        buffer_size_ = FrameHeader::SIZE + rgb_size + depth_size + pose_size + intrinsics_size;

        diagnostics_logger_ = DiagnosticsLogger::get_logger("sim_rgbd_camera");
    }

    SimRgbdCamera::~SimRgbdCamera()
    {
    }

    bool SimRgbdCamera::initialize()
    {
        size_t total_size;
        if (enable_double_buffering_)
            total_size = buffer_size_ * 2; // Double buffering
        else
            total_size = buffer_size_;
        frame_reader_ = std::make_unique<SharedMemoryReader>("auto_battlebot_frame", total_size);

        return frame_reader_->open();
    }

    bool SimRgbdCamera::get(CameraData &data, bool get_depth) const
    {
        data = CameraData();

        const auto *frame_header = frame_reader_->read_at<FrameHeader>(0);
        double stamp = frame_header->timestamp;

        Header header;
        header.stamp = stamp;
        header.frame_id = FrameId::CAMERA;

        // Determine which buffer to read from (this is the double buffering implementation)
        size_t buffer_offset = frame_header->active_buffer == 0
                                   ? buffer_size_ // Read inactive buffer
                                   : 0;

        const auto *active_header = frame_reader_->read_at<FrameHeader>(buffer_offset);

        // Parse color image
        const uint8_t *rgb = frame_reader_->read_at<uint8_t>(buffer_offset + active_header->rgb_offset);
        cv::Mat color_image(expected_width_, expected_height_, CV_8UC3, const_cast<void *>(static_cast<const void *>(rgb)));
        data.rgb.header = header;
        data.rgb.image = color_image;

        // Parse depth image
        if (get_depth)
        {
            const float *depth = frame_reader_->read_at<float>(buffer_offset + active_header->depth_offset);
            cv::Mat depth_image(expected_width_, expected_height_, CV_32FC1, const_cast<void *>(static_cast<const void *>(depth)));
            data.depth.image = depth_image;
            data.depth.header = header;
        }

        // Parse "visual SLAM" pose
        const double *pose = frame_reader_->read_at<double>(buffer_offset + active_header->pose_offset);
        TransformStamped tf_visodom_from_camera;
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                tf_visodom_from_camera.transform.tf(i, j) = pose[i * j];
            }
        }
        tf_visodom_from_camera.header.stamp = stamp;
        tf_visodom_from_camera.header.frame_id = FrameId::VISUAL_ODOMETRY;
        tf_visodom_from_camera.child_frame_id = FrameId::CAMERA;
        data.tf_visodom_from_camera = tf_visodom_from_camera;

        // Parse camera intrinsics
        const double *intrinsics_fx = frame_reader_->read_at<double>(buffer_offset + active_header->intrinsics_fx_offset);
        const double *intrinsics_fy = frame_reader_->read_at<double>(buffer_offset + active_header->intrinsics_fy_offset);
        const double *intrinsics_cx = frame_reader_->read_at<double>(buffer_offset + active_header->intrinsics_cx_offset);
        const double *intrinsics_cy = frame_reader_->read_at<double>(buffer_offset + active_header->intrinsics_cy_offset);
        const double *distortion_k1 = frame_reader_->read_at<double>(buffer_offset + active_header->distortion_k1_offset);
        const double *distortion_k2 = frame_reader_->read_at<double>(buffer_offset + active_header->distortion_k2_offset);
        const double *distortion_p1 = frame_reader_->read_at<double>(buffer_offset + active_header->distortion_p1_offset);
        const double *distortion_p2 = frame_reader_->read_at<double>(buffer_offset + active_header->distortion_p2_offset);
        const double *distortion_k3 = frame_reader_->read_at<double>(buffer_offset + active_header->distortion_k3_offset);

        // Set intrinsics matrix (fx, 0, cx; 0, fy, cy; 0, 0, 1)
        CameraInfo camera_info;
        camera_info.intrinsics = cv::Mat::eye(3, 3, CV_64F);
        camera_info.intrinsics.at<double>(0, 0) = *intrinsics_fx;
        camera_info.intrinsics.at<double>(1, 1) = *intrinsics_fy;
        camera_info.intrinsics.at<double>(0, 2) = *intrinsics_cx;
        camera_info.intrinsics.at<double>(1, 2) = *intrinsics_cy;

        // Set distortion coefficients
        camera_info.distortion = cv::Mat::zeros(1, 5, CV_64F);
        camera_info.distortion.at<double>(0, 0) = *distortion_k1;
        camera_info.distortion.at<double>(0, 1) = *distortion_k2;
        camera_info.distortion.at<double>(0, 2) = *distortion_p1;
        camera_info.distortion.at<double>(0, 3) = *distortion_p2;
        camera_info.distortion.at<double>(0, 4) = *distortion_k3;
        camera_info.header = header;

        data.camera_info = camera_info;

        return true;
    }

    bool SimRgbdCamera::should_close()
    {
        return false;
    }
} // namespace auto_battlebot
