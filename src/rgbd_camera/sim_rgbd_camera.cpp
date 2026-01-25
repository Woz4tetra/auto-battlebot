#include "rgbd_camera/sim_rgbd_camera.hpp"

namespace auto_battlebot
{
    SimRgbdCamera::SimRgbdCamera(SimRgbdCameraConfiguration &config)
        : expected_width_(config.width),
          expected_height_(config.height),
          enable_double_buffering_(config.enable_double_buffering),
          frame_reader_(nullptr),
          frames_received_(0),
          last_frame_id_(0),
          last_log_time_(std::chrono::steady_clock::now())
    {
        buffer_size_ = get_simulation_frame_size(expected_width_, expected_height_);

        diagnostics_logger_ = DiagnosticsLogger::get_logger("sim_rgbd_camera");
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
                                   {"buffer_size", static_cast<int>(buffer_size_)}},
                                  "Initializing SimRgbdCamera");

        std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // cooldown for repeated opens

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
        if (success)
        {
            diagnostics_logger_->info("SimRgbdCamera initialized successfully");
            // Reset stats on re-initialization
            frames_received_ = 0;
            last_frame_id_ = 0;
            last_log_time_ = std::chrono::steady_clock::now();
        }
        else
        {
            std::cerr << "Failed to open shared memory for SimRgbdCamera" << std::endl;
        }

        return success;
    }

    bool SimRgbdCamera::get(CameraData &data, bool get_depth) const
    {
        data = CameraData();

        if (!frame_reader_ || !frame_reader_->is_open())
        {
            diagnostics_logger_->warning("Shared memory not open");
            return false;
        }

        const auto *frame_header = frame_reader_->read_at<SimulationFrameHeader>(0);

        // Log raw header values for debugging
        diagnostics_logger_->debug("header",
                                   {{"frame_id", static_cast<int>(frame_header->frame_id)},
                                    {"timestamp", frame_header->timestamp},
                                    {"width", frame_header->width},
                                    {"height", frame_header->height},
                                    {"active_buffer", frame_header->active_buffer},
                                    {"rgb_offset", frame_header->rgb_offset},
                                    {"depth_offset", frame_header->depth_offset},
                                    {"pose_offset", frame_header->pose_offset}});

        if (frame_header->width <= 0 || frame_header->height <= 0)
        {
            diagnostics_logger_->warning("dimensions",
                                         {{"width", frame_header->width},
                                          {"height", frame_header->height}},
                                         "Invalid frame header dimensions (zero or negative)");
            return false;
        }
        if (frame_header->width != expected_width_ || frame_header->height != expected_height_)
        {
            diagnostics_logger_->warning("dimensions",
                                         {{"received_width", frame_header->width},
                                          {"received_height", frame_header->height},
                                          {"expected_width", expected_width_},
                                          {"expected_height", expected_height_}},
                                         "Frame dimensions mismatch");
            return false;
        }

        double stamp = frame_header->timestamp;

        Header header;
        header.stamp = stamp;
        header.frame_id = FrameId::CAMERA;

        // Determine which buffer to read from (for double buffering)
        // When double buffering is enabled, read the inactive buffer
        // When disabled, always read from offset 0
        size_t buffer_offset = 0;
        if (enable_double_buffering_)
        {
            int active_buffer = frame_header->active_buffer;
            if (active_buffer != 0 && active_buffer != 1)
            {
                diagnostics_logger_->warning("Invalid active buffer index");
                active_buffer = 0;
            }
            buffer_offset = active_buffer == 0
                                ? buffer_size_ // Read inactive buffer
                                : 0;
        }

        const auto *active_header = frame_reader_->read_at<SimulationFrameHeader>(buffer_offset);

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
        bool is_new_frame = (frame_header->frame_id != last_frame_id_);
        last_frame_id_ = frame_header->frame_id;

        // Log intrinsics on first frame or when they change
        if (frames_received_ == 1)
        {
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
        }

        // Periodic stats logging (every 5 seconds)
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time_).count();
        if (elapsed >= 5)
        {
            double fps = static_cast<double>(frames_received_) / elapsed;
            diagnostics_logger_->info("stats",
                                      {{"frames_received", static_cast<int>(frames_received_)},
                                       {"fps", fps},
                                       {"last_frame_id", static_cast<int>(last_frame_id_)},
                                       {"last_timestamp", stamp},
                                       {"is_new_frame", is_new_frame ? 1 : 0}});
            frames_received_ = 0;
            last_log_time_ = now;
        }

        return true;
    }

    bool SimRgbdCamera::should_close()
    {
        return false;
    }
} // namespace auto_battlebot
