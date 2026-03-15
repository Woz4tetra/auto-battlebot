#include "rgbd_camera/zed_rgbd_camera.hpp"

#include <spdlog/spdlog.h>

#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>

#include "directories.hpp"

namespace auto_battlebot {

namespace {
constexpr uint64_t kBytesPerGb = 1024ULL * 1024ULL * 1024ULL;
constexpr uint64_t kSizeCheckIntervalFrames = 100;
}  // namespace

ZedRgbdCamera::ZedRgbdCamera(ZedRgbdCameraConfiguration &config)
    : zed_(sl::Camera()),
      is_initialized_(false),
      should_close_(false),
      stop_thread_(false),
      has_new_frame_(false),
      frame_counter_(0),
      depth_frame_counter_(0),
      last_returned_frame_counter_(0),
      prev_tracking_state_(sl::POSITIONAL_TRACKING_STATE::LAST),
      position_tracking_enabled_(config.position_tracking),
      svo_recording_enabled_(config.svo_recording && config.svo_file_path.empty()),
      svo_holding_dir_(get_project_path("data/temp_svo")),
      svo_max_size_bytes_(config.svo_max_size_gb * kBytesPerGb),
      svo_holding_dir_max_size_bytes_(config.svo_holding_dir_max_size_gb * kBytesPerGb),
      frames_since_size_check_(0) {
    diagnostics_logger_ = DiagnosticsLogger::get_logger("zed_rgbd_camera");
    reset_capture_timing_stats();

    params_ = sl::InitParameters();
    params_.camera_fps = config.camera_fps;
    params_.camera_resolution = get_zed_resolution(config.camera_resolution);
    params_.depth_mode = get_zed_depth_mode(config.depth_mode);
    params_.coordinate_system = sl::COORDINATE_SYSTEM::IMAGE;
    params_.coordinate_units = sl::UNIT::METER;

    // Set SVO file path if provided (for playback instead of live camera)
    if (!config.svo_file_path.empty()) {
        std::filesystem::path svo_file_path = config.svo_file_path.c_str();
        std::filesystem::path svo_file_abs_path = std::filesystem::absolute(svo_file_path);
        spdlog::info("Resolved SVO path: {}", svo_file_abs_path.string());

        params_.input.setFromSVOFile(svo_file_abs_path.c_str());
        params_.svo_real_time_mode = config.svo_real_time_mode;
    }
}

ZedRgbdCamera::~ZedRgbdCamera() {
    if (is_initialized_) {
        stop_thread_ = true;
        if (capture_thread_.joinable()) {
            capture_thread_.join();
        }
        stop_svo_recording();
        zed_.close();
    }
}

bool ZedRgbdCamera::initialize() {
    // Clean up previous thread if it exists (support re-initialization)
    if (capture_thread_.joinable()) {
        stop_thread_ = true;
        data_cv_.notify_all();
        capture_thread_.join();
    }

    // Reset state variables for re-initialization
    stop_thread_ = false;
    should_close_ = false;
    has_new_frame_ = false;
    frame_counter_ = 0;
    depth_frame_counter_ = 0;
    last_returned_frame_counter_ = 0;
    prev_tracking_state_ = sl::POSITIONAL_TRACKING_STATE::LAST;
    reset_capture_timing_stats();
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        while (!depth_request_queue_.empty()) {
            depth_request_queue_.pop();
        }
    }

    sl::ERROR_CODE returned_state = zed_.open(params_);
    if (returned_state != sl::ERROR_CODE::SUCCESS) {
        spdlog::error("Failed to open ZED camera: {}", sl::toString(returned_state).c_str());
        return false;
    }

    if (position_tracking_enabled_) {
        // Enable positional tracking for getting camera pose
        sl::PositionalTrackingParameters tracking_params;
        tracking_params.enable_imu_fusion = true;
        returned_state = zed_.enablePositionalTracking(tracking_params);
        if (returned_state != sl::ERROR_CODE::SUCCESS) {
            spdlog::error("Failed to enable positional tracking: {}",
                          sl::toString(returned_state).c_str());
            zed_.close();
            return false;
        }
    } else {
        spdlog::info("Position tracking is disabled");
    }

    // Get camera information for intrinsics
    sl::CalibrationParameters calibration =
        zed_.getCameraInformation().camera_configuration.calibration_parameters;
    sl::Resolution image_size = zed_.getCameraInformation().camera_configuration.resolution;

    // Initialize camera info
    latest_data_.camera_info.width = static_cast<int>(image_size.width);
    latest_data_.camera_info.height = static_cast<int>(image_size.height);

    // Set intrinsics matrix (fx, 0, cx; 0, fy, cy; 0, 0, 1)
    latest_data_.camera_info.intrinsics = cv::Mat::eye(3, 3, CV_64F);
    latest_data_.camera_info.intrinsics.at<double>(0, 0) = calibration.left_cam.fx;
    latest_data_.camera_info.intrinsics.at<double>(1, 1) = calibration.left_cam.fy;
    latest_data_.camera_info.intrinsics.at<double>(0, 2) = calibration.left_cam.cx;
    latest_data_.camera_info.intrinsics.at<double>(1, 2) = calibration.left_cam.cy;

    // Set distortion coefficients
    latest_data_.camera_info.distortion = cv::Mat::zeros(1, 5, CV_64F);
    latest_data_.camera_info.distortion.at<double>(0, 0) = calibration.left_cam.disto[0];  // k1
    latest_data_.camera_info.distortion.at<double>(0, 1) = calibration.left_cam.disto[1];  // k2
    latest_data_.camera_info.distortion.at<double>(0, 2) = calibration.left_cam.disto[2];  // p1
    latest_data_.camera_info.distortion.at<double>(0, 3) = calibration.left_cam.disto[3];  // p2
    latest_data_.camera_info.distortion.at<double>(0, 4) = calibration.left_cam.disto[4];  // k3

    is_initialized_ = true;

    if (svo_recording_enabled_) {
        std::filesystem::create_directories(svo_holding_dir_);
        enforce_holding_dir_size();
        start_svo_recording();
    }

    // Start capture thread
    capture_thread_ = std::thread(&ZedRgbdCamera::capture_thread_loop, this);

    return true;
}

void ZedRgbdCamera::reset_capture_timing_stats() const {
    captures_since_last_report_ = 0;
    capture_time_sum_ms_ = 0.0;
    capture_time_min_ms_ = std::numeric_limits<double>::infinity();
    capture_time_max_ms_ = 0.0;
}

void ZedRgbdCamera::capture_thread_loop() {
    while (!stop_thread_) {
        if (capture_frame()) {
            has_new_frame_ = true;
            frame_counter_++;
            data_cv_.notify_all();
        } else if (should_close_) {
            break;
        }
    }
}

bool ZedRgbdCamera::capture_frame() {
    if (!is_initialized_) {
        return false;
    }

    auto capture_start = std::chrono::steady_clock::now();

    bool need_depth = false;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        need_depth = !depth_request_queue_.empty();
        if (need_depth) {
            depth_request_queue_.pop();
        }
    }

    // Grab new frame (without lock)
    sl::RuntimeParameters rt_params;
    rt_params.enable_depth = true;
    sl::ERROR_CODE grab_status = zed_.grab(rt_params);

    if (grab_status != sl::ERROR_CODE::SUCCESS) {
        if (grab_status == sl::ERROR_CODE::END_OF_SVOFILE_REACHED) {
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                should_close_ = true;
            }
            data_cv_.notify_all();
            spdlog::info("End of SVO file reached.");
        } else {
            spdlog::error("Failed to grab frame: {}", sl::toString(grab_status).c_str());
        }
        return false;
    }

    // Periodically check SVO file size and roll over if needed
    if (svo_recording_enabled_ && !current_svo_path_.empty()) {
        frames_since_size_check_++;
        if (frames_since_size_check_ >= kSizeCheckIntervalFrames) {
            frames_since_size_check_ = 0;
            std::error_code ec;
            auto file_size = std::filesystem::file_size(current_svo_path_, ec);
            if (!ec && file_size >= svo_max_size_bytes_) {
                stop_svo_recording();
                enforce_holding_dir_size();
                start_svo_recording();
            }
        }
    }

    // Lock mutex to modify data
    std::lock_guard<std::mutex> lock(data_mutex_);

    // Retrieve RGB image
    sl::ERROR_CODE retrieve_status = zed_.retrieveImage(zed_rgb_, sl::VIEW::LEFT);

    if (retrieve_status != sl::ERROR_CODE::SUCCESS) {
        spdlog::error("Failed to retrieve RGB image: {}", sl::toString(retrieve_status).c_str());
        return false;
    }

    // Retrieve depth map only if requested
    if (need_depth) {
        retrieve_status = zed_.retrieveMeasure(zed_depth_, sl::MEASURE::DEPTH);
        if (retrieve_status != sl::ERROR_CODE::SUCCESS) {
            spdlog::error("Failed to retrieve depth image: {}",
                          sl::toString(retrieve_status).c_str());
            depth_request_queue_.push(1);
            return false;
        }
    }

    // Get timestamp
    sl::Timestamp timestamp = zed_.getTimestamp(sl::TIME_REFERENCE::IMAGE);
    double stamp = static_cast<double>(timestamp.getNanoseconds()) / 1e9;

    latest_data_.tf_visodom_from_camera.header.stamp = stamp;
    latest_data_.tf_visodom_from_camera.header.frame_id = FrameId::VISUAL_ODOMETRY;
    latest_data_.tf_visodom_from_camera.child_frame_id = FrameId::CAMERA;

    // Get camera pose
    if (position_tracking_enabled_) {
        sl::POSITIONAL_TRACKING_STATE tracking_state =
            zed_.getPosition(zed_pose_, sl::REFERENCE_FRAME::WORLD);
        if (tracking_state != prev_tracking_state_) {
            spdlog::info("Tracking state: {}", sl::toString(tracking_state).c_str());
            prev_tracking_state_ = tracking_state;
        }

        // Convert pose to transform matrix (4x4 Eigen matrix)
        sl::Transform zed_transform = zed_pose_.pose_data;
        latest_data_.tf_visodom_from_camera.transform.tf = Eigen::MatrixXd(4, 4);
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                latest_data_.tf_visodom_from_camera.transform.tf(i, j) = zed_transform(i, j);
            }
        }
    } else {
        latest_data_.tf_visodom_from_camera.transform.tf = Eigen::MatrixXd::Identity(4, 4);
    }

    // Convert ZED RGB image to OpenCV Mat (BGRA to BGR)
    cv::Mat zed_rgb_mat(zed_rgb_.getHeight(), zed_rgb_.getWidth(), CV_8UC4,
                        zed_rgb_.getPtr<sl::uchar1>());
    cv::cvtColor(zed_rgb_mat, latest_data_.rgb.image, cv::COLOR_BGRA2BGR);

    // Convert ZED depth image to OpenCV Mat (float32) if requested
    if (need_depth) {
        cv::Mat zed_depth_mat(zed_depth_.getHeight(), zed_depth_.getWidth(), CV_32FC1,
                              zed_depth_.getPtr<sl::uchar1>());
        zed_depth_mat.copyTo(latest_data_.depth.image);
        depth_frame_counter_ =
            frame_counter_ + 1;  // capture_thread_loop will increment after this function returns
    } else {
        latest_data_.depth.image.release();
    }

    Header header;
    header.stamp = stamp;
    header.frame_id = FrameId::CAMERA;

    latest_data_.rgb.header = header;
    latest_data_.depth.header = header;
    latest_data_.camera_info.header = header;

    auto capture_end = std::chrono::steady_clock::now();
    double capture_time_ms =
        std::chrono::duration<double, std::milli>(capture_end - capture_start).count();

    captures_since_last_report_++;
    capture_time_sum_ms_ += capture_time_ms;
    capture_time_min_ms_ = std::min(capture_time_min_ms_, capture_time_ms);
    capture_time_max_ms_ = std::max(capture_time_max_ms_, capture_time_ms);

    return true;
}

bool ZedRgbdCamera::get(CameraData &data, bool get_depth) {
    if (!is_initialized_) return false;
    std::unique_lock<std::mutex> lock(data_mutex_);

    if (get_depth) {
        // Request depth for the next frame and wait for a new frame with depth
        depth_request_queue_.push(1);

        uint64_t current_frame = frame_counter_;
        uint64_t current_depth_frame = depth_frame_counter_;
        data_cv_.wait(lock, [this, current_frame, current_depth_frame]() {
            bool new_frame = frame_counter_ > current_frame;
            bool depth_ready = depth_frame_counter_ > current_depth_frame;
            return (new_frame && depth_ready) || should_close_ || stop_thread_;
        });
    } else {
        // If a prefetched frame is available, return immediately; otherwise, wait for one
        if (!(frame_counter_ > last_returned_frame_counter_)) {
            data_cv_.wait(lock, [this]() {
                bool new_frame_available = frame_counter_ > last_returned_frame_counter_;
                return new_frame_available || should_close_ || stop_thread_;
            });
        }
    }

    if (should_close_ || stop_thread_) return false;

    data = latest_data_;
    // Mark the frame as consumed for non-depth path; harmless for depth path
    last_returned_frame_counter_ = frame_counter_;

    if (diagnostics_logger_ && captures_since_last_report_ > 0) {
        double avg_ms = capture_time_sum_ms_ / static_cast<double>(captures_since_last_report_);

        diagnostics_logger_->info(
            "capture_frame", {{"frames_since_last", std::to_string(captures_since_last_report_)},
                              {"capture_ms_avg", std::to_string(avg_ms)},
                              {"capture_ms_min", std::to_string(capture_time_min_ms_)},
                              {"capture_ms_max", std::to_string(capture_time_max_ms_)}});

        reset_capture_timing_stats();
    }
    return true;
}

bool ZedRgbdCamera::should_close() { return should_close_; }

std::string ZedRgbdCamera::get_current_svo_path() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return current_svo_path_.string();
}

std::string ZedRgbdCamera::generate_svo_filename() const {
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    localtime_r(&t, &tm);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%dT%H-%M-%S") << ".svo2";
    return (svo_holding_dir_ / oss.str()).string();
}

bool ZedRgbdCamera::start_svo_recording() {
    std::string path = generate_svo_filename();
    sl::RecordingParameters rec_params;
    rec_params.video_filename = sl::String(path.c_str());
    rec_params.compression_mode = sl::SVO_COMPRESSION_MODE::H264;
    sl::ERROR_CODE err = zed_.enableRecording(rec_params);
    if (err != sl::ERROR_CODE::SUCCESS) {
        spdlog::error("Failed to start SVO recording: {}", sl::toString(err).c_str());
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        current_svo_path_ = path;
    }
    frames_since_size_check_ = 0;
    spdlog::info("SVO recording started: {}", path);
    return true;
}

void ZedRgbdCamera::stop_svo_recording() {
    if (current_svo_path_.empty()) return;
    zed_.disableRecording();
    std::lock_guard<std::mutex> lock(data_mutex_);
    spdlog::info("SVO recording stopped: {}", current_svo_path_.string());
    current_svo_path_.clear();
}

void ZedRgbdCamera::enforce_holding_dir_size() {
    std::error_code ec;
    if (!std::filesystem::exists(svo_holding_dir_, ec)) return;

    std::vector<std::filesystem::path> svo_files;
    for (const auto &entry : std::filesystem::directory_iterator(svo_holding_dir_, ec)) {
        if (entry.is_regular_file() && entry.path().extension() == ".svo2") {
            svo_files.push_back(entry.path());
        }
    }

    std::sort(svo_files.begin(), svo_files.end(), [](const auto &a, const auto &b) {
        return std::filesystem::last_write_time(a) < std::filesystem::last_write_time(b);
    });

    uintmax_t total_size = 0;
    for (const auto &f : svo_files) {
        total_size += std::filesystem::file_size(f, ec);
    }

    for (const auto &f : svo_files) {
        if (total_size <= svo_holding_dir_max_size_bytes_) break;
        uintmax_t file_size = std::filesystem::file_size(f, ec);
        if (std::filesystem::remove(f, ec)) {
            spdlog::info("Deleted oldest SVO to free space: {}", f.string());
            total_size -= file_size;
        } else {
            spdlog::error("Failed to delete SVO: {}", f.string());
        }
    }
}

}  // namespace auto_battlebot
