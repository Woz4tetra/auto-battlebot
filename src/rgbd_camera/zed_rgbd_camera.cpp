#include "rgbd_camera/zed_rgbd_camera.hpp"

#include <spdlog/spdlog.h>

#include <chrono>
#include <ctime>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <utility>
#include <vector>

#include "directories.hpp"

namespace auto_battlebot {

namespace {
constexpr uint64_t kBytesPerGb = 1024ULL * 1024ULL * 1024ULL;
constexpr uint64_t kSizeCheckIntervalFrames = 100;
constexpr auto kGrabErrorWindow = std::chrono::seconds(10);
constexpr double kGrabErrorExitThreshold = 0.70;
constexpr auto kGetWaitTimeout = std::chrono::milliseconds(100);
constexpr double kOpenWaitWarnMs = 2000.0;
constexpr double kJoinWaitWarnMs = 1000.0;
constexpr double kGrabWarnMs = 250.0;
constexpr double kCaptureLockWarnMs = 150.0;
constexpr double kGetWaitWarnMs = 500.0;
constexpr auto kJoinHardTimeout = std::chrono::seconds(10);
constexpr auto kOpenHardTimeout = std::chrono::seconds(30);

double elapsed_ms(const std::chrono::steady_clock::time_point &start) {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start)
        .count();
}

// Deliberate static leak: a std::future whose underlying task is wedged inside the ZED SDK
// cannot be safely destroyed (its destructor blocks on the task). On a hard timeout we move
// the stuck future here so it outlives the rest of the program; the OS reclaims memory at
// process exit. Allocated with `new` and never freed so neither the vector nor the mutex
// ever has its destructor invoked at static teardown.
std::mutex *g_leaked_open_futures_mutex = new std::mutex;
std::vector<std::future<sl::ERROR_CODE>> *g_leaked_open_futures =
    new std::vector<std::future<sl::ERROR_CODE>>;

void leak_open_future(std::future<sl::ERROR_CODE> &&future) {
    std::lock_guard<std::mutex> lock(*g_leaked_open_futures_mutex);
    g_leaked_open_futures->push_back(std::move(future));
}

// Returns true if the thread joined cleanly within kJoinHardTimeout, false if it had to be
// detached because it appeared wedged. We poll a "done" flag the capture thread sets on exit
// rather than calling thread.join() directly with a separate timer, because we need to be
// able to give up without blocking the process shutdown path.
bool join_with_timeout(std::thread &thread, const std::atomic<bool> &done_flag,
                       const char *context) {
    const auto deadline = std::chrono::steady_clock::now() + kJoinHardTimeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (done_flag.load(std::memory_order_acquire)) {
            thread.join();
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    spdlog::critical(
        "{}: capture thread stuck in ZED grab() after {}s; detaching for soft shutdown", context,
        kJoinHardTimeout.count());
    spdlog::default_logger()->flush();
    thread.detach();
    return false;
}

bool is_transient_grab_error(sl::ERROR_CODE error_code) {
    switch (error_code) {
        case sl::ERROR_CODE::CORRUPTED_FRAME:
        case sl::ERROR_CODE::CAMERA_REBOOTING:
            return true;
        default:
            return false;
    }
}
}  // namespace

ZedRgbdCamera::ZedRgbdCamera(ZedRgbdCameraConfiguration &config)
    : zed_(sl::Camera()),
      is_initialized_(false),
      should_close_(false),
      stop_thread_(false),
      camera_connected_(false),
      has_new_frame_(false),
      frame_counter_(0),
      depth_frame_counter_(0),
      last_returned_frame_counter_(0),
      recent_grab_error_count_(0),
      prev_tracking_state_(sl::POSITIONAL_TRACKING_STATE::LAST),
      position_tracking_enabled_(config.position_tracking),
      is_playback_input_(!config.svo_file_path.empty()),
      svo_recording_enabled_(config.svo_recording && config.svo_file_path.empty()),
      svo_holding_dir_(get_project_path("data/temp_svo")),
      svo_max_size_bytes_(config.svo_max_size_gb * kBytesPerGb),
      svo_holding_dir_max_size_bytes_(config.svo_holding_dir_max_size_gb * kBytesPerGb),
      frames_since_size_check_(0),
      svo_start_frame_(config.svo_start_frame) {
    diagnostics_logger_ = DiagnosticsLogger::get_logger("zed_rgbd_camera");
    reset_capture_timing_stats();

    params_ = sl::InitParameters();
    params_.camera_fps = config.camera_fps;
    params_.camera_resolution = get_zed_resolution(config.camera_resolution);
    params_.depth_mode = get_zed_depth_mode(config.depth_mode);
    params_.coordinate_system = sl::COORDINATE_SYSTEM::IMAGE;
    params_.coordinate_units = sl::UNIT::METER;

    // Set SVO file path if provided (for playback instead of live camera)
    if (is_playback_input_) {
        std::filesystem::path svo_file_path = config.svo_file_path.c_str();
        std::filesystem::path svo_file_abs_path = std::filesystem::absolute(svo_file_path);
        spdlog::info("Resolved SVO path: {}", svo_file_abs_path.string());

        params_.input.setFromSVOFile(svo_file_abs_path.c_str());
        params_.svo_real_time_mode = config.svo_real_time_mode;
    }
}

ZedRgbdCamera::~ZedRgbdCamera() {
    // Cancel any in-progress initialize() before doing anything else.
    cancel_open_ = true;
    if (pending_open_.valid()) {
        const auto open_wait_start = std::chrono::steady_clock::now();
        if (pending_open_.wait_for(kOpenHardTimeout) == std::future_status::timeout) {
            spdlog::critical(
                "ZedRgbdCamera destructor: zed_.open() stuck after {}s; leaking the open future "
                "for soft shutdown (the ZED SDK call will complete in the background and be "
                "reaped at process exit)",
                kOpenHardTimeout.count());
            spdlog::default_logger()->flush();
            leak_open_future(std::move(pending_open_));
            // is_initialized_ is false (only set after a successful open returns), so the
            // capture-thread / zed_.close() block below is correctly skipped. Returning here
            // also avoids racing zed_.close() against the still-running zed_.open() call.
            return;
        }
        const double open_wait_ms = elapsed_ms(open_wait_start);
        if (open_wait_ms > kOpenWaitWarnMs) {
            spdlog::warn("validation: pending_open_wait_destructor slow elapsed_ms={:.2f}",
                         open_wait_ms);
        }
    }
    if (is_initialized_) {
        stop_thread_ = true;
        if (capture_thread_.joinable()) {
            data_cv_.notify_all();
            const auto join_start = std::chrono::steady_clock::now();
            const bool joined = join_with_timeout(capture_thread_, capture_thread_done_,
                                                  "ZedRgbdCamera destructor");
            const double join_ms = elapsed_ms(join_start);
            if (join_ms > kJoinWaitWarnMs) {
                spdlog::warn("validation: capture_thread_join_destructor slow elapsed_ms={:.2f}",
                             join_ms);
            }
            if (!joined) {
                // Detached capture thread is potentially still inside zed_.grab(); calling
                // stop_svo_recording() / zed_.close() concurrently against the same handle
                // would race the SDK. Skip them and let the OS reap on process exit.
                return;
            }
        }
        stop_svo_recording();
        zed_.close();
    }
}

void ZedRgbdCamera::cancel_initialize() { cancel_open_ = true; }

bool ZedRgbdCamera::initialize() {
    const auto initialize_start = std::chrono::steady_clock::now();

    // Clean up previous thread if it exists (support re-initialization)
    if (capture_thread_.joinable()) {
        stop_thread_ = true;
        data_cv_.notify_all();
        const auto join_start = std::chrono::steady_clock::now();
        const bool joined =
            join_with_timeout(capture_thread_, capture_thread_done_, "ZedRgbdCamera::initialize");
        const double join_ms = elapsed_ms(join_start);
        if (join_ms > kJoinWaitWarnMs) {
            spdlog::warn("validation: capture_thread_join_initialize slow elapsed_ms={:.2f}",
                         join_ms);
        }
        if (!joined) {
            // Old capture thread is detached and may still be inside zed_.grab(). Re-opening
            // the same SDK handle while another thread is using it would race; signal a soft
            // shutdown so the runner exits cleanly via Restart=on-failure rather than racing.
            should_close_ = true;
            return false;
        }
    }

    // Ensure the SDK camera handle is reset before calling open() again.
    if (is_initialized_) {
        stop_svo_recording();
        zed_.close();
        is_initialized_ = false;
    }

    // Reset state variables for re-initialization
    stop_thread_ = false;
    should_close_ = false;
    camera_connected_ = true;
    has_new_frame_ = false;
    capture_thread_done_.store(false, std::memory_order_release);
    frame_counter_ = 0;
    depth_frame_counter_ = 0;
    last_returned_frame_counter_ = 0;
    recent_grab_results_.clear();
    recent_grab_error_count_ = 0;
    prev_tracking_state_ = sl::POSITIONAL_TRACKING_STATE::LAST;
    reset_capture_timing_stats();
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        while (!depth_request_queue_.empty()) {
            depth_request_queue_.pop();
        }
    }

    // Run zed_.open() on a background thread so we can check cancel_open_ while it blocks.
    cancel_open_ = false;
    pending_open_ =
        std::async(std::launch::async, [this]() -> sl::ERROR_CODE { return zed_.open(params_); });

    while (pending_open_.wait_for(std::chrono::milliseconds(50)) != std::future_status::ready) {
        if (cancel_open_) {
            // We can't interrupt zed_.open() itself; wait for it to return before we leave so
            // the ZED object is never used concurrently or destroyed mid-open.
            const auto cancel_wait_start = std::chrono::steady_clock::now();
            if (pending_open_.wait_for(kOpenHardTimeout) == std::future_status::timeout) {
                spdlog::critical(
                    "ZedRgbdCamera::initialize cancel: zed_.open() stuck after {}s; leaking the "
                    "open future and signalling soft shutdown",
                    kOpenHardTimeout.count());
                spdlog::default_logger()->flush();
                leak_open_future(std::move(pending_open_));
                should_close_ = true;
                return false;
            }
            const double cancel_wait_ms = elapsed_ms(cancel_wait_start);
            if (cancel_wait_ms > kOpenWaitWarnMs) {
                spdlog::warn("validation: pending_open_wait_cancel slow elapsed_ms={:.2f}",
                             cancel_wait_ms);
            }
            return false;
        }
    }

    sl::ERROR_CODE returned_state = pending_open_.get();
    if (returned_state != sl::ERROR_CODE::SUCCESS) {
        spdlog::error("Failed to open ZED camera: {}", sl::toString(returned_state).c_str());
        return false;
    }

    if (svo_start_frame_ > 0 && zed_.getCameraInformation().input_type == sl::INPUT_TYPE::SVO) {
        const int n_frames = zed_.getSVONumberOfFrames();
        if (n_frames > 0) {
            const int frame = std::clamp(svo_start_frame_, 0, n_frames - 1);
            if (frame != svo_start_frame_) {
                spdlog::warn("svo_start_frame {} out of range for this SVO ({} frames), using {}",
                             svo_start_frame_, n_frames, frame);
            }
            spdlog::info("SVO starting at frame {} of {}", frame, n_frames);
            zed_.setSVOPosition(frame);
        }
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

    if (svo_recording_enabled_.load()) {
        std::filesystem::create_directories(svo_holding_dir_);
        enforce_holding_dir_size();
        start_svo_recording();
    }

    // Start capture thread
    capture_thread_ = std::thread(&ZedRgbdCamera::capture_thread_loop, this);

    const double initialize_ms = elapsed_ms(initialize_start);
    if (initialize_ms > kOpenWaitWarnMs) {
        spdlog::warn("validation: zed_initialize_total slow elapsed_ms={:.2f}", initialize_ms);
    }

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
    capture_thread_done_.store(true, std::memory_order_release);
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
    const auto grab_start = std::chrono::steady_clock::now();
    sl::ERROR_CODE grab_status = zed_.grab(rt_params);
    const double grab_ms = elapsed_ms(grab_start);
    if (grab_ms > kGrabWarnMs) {
        spdlog::warn("validation: zed_grab slow elapsed_ms={:.2f}", grab_ms);
    }

    if (is_transient_grab_error(grab_status)) {
        // Treat transient frame grab issues as recoverable: wait for the next good frame.
        return false;
    }

    if (grab_status != sl::ERROR_CODE::SUCCESS) {
        camera_connected_ = false;
        const auto now = std::chrono::steady_clock::now();
        recent_grab_results_.emplace_back(now, true);
        recent_grab_error_count_++;
        while (!recent_grab_results_.empty() &&
               (now - recent_grab_results_.front().first) > kGrabErrorWindow) {
            if (recent_grab_results_.front().second) {
                recent_grab_error_count_--;
            }
            recent_grab_results_.pop_front();
        }

        const bool window_is_full = !recent_grab_results_.empty() &&
                                    (now - recent_grab_results_.front().first) >= kGrabErrorWindow;
        if (window_is_full && !should_close_.load()) {
            const double grab_error_ratio = static_cast<double>(recent_grab_error_count_) /
                                            static_cast<double>(recent_grab_results_.size());
            if (grab_error_ratio > kGrabErrorExitThreshold) {
                {
                    std::lock_guard<std::mutex> lock(data_mutex_);
                    should_close_ = true;
                }
                data_cv_.notify_all();
                spdlog::error(
                    "Camera grab error ratio {:.1f}% over last 10s exceeded {:.0f}% "
                    "threshold. Requesting application shutdown.",
                    grab_error_ratio * 100.0, kGrabErrorExitThreshold * 100.0);
                return false;
            }
        }

        data_cv_.notify_all();
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
    camera_connected_ = true;
    {
        const auto now = std::chrono::steady_clock::now();
        recent_grab_results_.emplace_back(now, false);
        while (!recent_grab_results_.empty() &&
               (now - recent_grab_results_.front().first) > kGrabErrorWindow) {
            if (recent_grab_results_.front().second) {
                recent_grab_error_count_--;
            }
            recent_grab_results_.pop_front();
        }
    }

    // Periodically check SVO file size and roll over if needed
    std::filesystem::path active_svo_path;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        active_svo_path = current_svo_path_;
    }
    if (svo_recording_enabled_.load() && !active_svo_path.empty()) {
        frames_since_size_check_++;
        if (frames_since_size_check_ >= kSizeCheckIntervalFrames) {
            frames_since_size_check_ = 0;
            std::error_code ec;
            auto file_size = std::filesystem::file_size(active_svo_path, ec);
            if (!ec && file_size >= svo_max_size_bytes_) {
                stop_svo_recording();
                enforce_holding_dir_size();
                start_svo_recording();
            }
        }
    }

    // Lock mutex to modify data
    const auto lock_hold_start = std::chrono::steady_clock::now();
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
    sl::Timestamp timestamp = zed_.getTimestamp(sl::TIME_REFERENCE::CURRENT);
    double stamp = static_cast<double>(timestamp.getNanoseconds()) / 1e9;

    latest_data_.tf_visodom_from_camera.header.stamp = stamp;
    latest_data_.tf_visodom_from_camera.header.frame_id = FrameId::VISUAL_ODOMETRY;
    latest_data_.tf_visodom_from_camera.child_frame_id = FrameId::CAMERA;

    // Get camera pose
    if (position_tracking_enabled_) {
        sl::POSITIONAL_TRACKING_STATE tracking_state =
            zed_.getPosition(zed_pose_, sl::REFERENCE_FRAME::WORLD);
        latest_data_.tracking_ok = (tracking_state == sl::POSITIONAL_TRACKING_STATE::OK);
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
        latest_data_.tracking_ok = true;
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

    const double lock_hold_ms = elapsed_ms(lock_hold_start);
    if (lock_hold_ms > kCaptureLockWarnMs) {
        spdlog::warn("validation: capture_data_mutex_hold slow elapsed_ms={:.2f} need_depth={}",
                     lock_hold_ms, need_depth);
    }

    return true;
}

bool ZedRgbdCamera::get(CameraData &data, bool get_depth) {
    if (!is_initialized_) return false;
    const auto get_start = std::chrono::steady_clock::now();
    int wait_loops = 0;
    std::unique_lock<std::mutex> lock(data_mutex_);

    if (get_depth) {
        // Request depth for the next frame and wait for a new frame with depth
        depth_request_queue_.push(1);

        uint64_t current_frame = frame_counter_;
        uint64_t current_depth_frame = depth_frame_counter_;
        while (
            !data_cv_.wait_for(lock, kGetWaitTimeout, [this, current_frame, current_depth_frame]() {
                bool new_frame = frame_counter_ > current_frame;
                bool depth_ready = depth_frame_counter_ > current_depth_frame;
                return (new_frame && depth_ready) || should_close_ || stop_thread_ ||
                       !camera_connected_.load();
            })) {
            wait_loops++;
            if (!camera_connected_.load()) return false;
        }
    } else {
        // If a prefetched frame is available, return immediately; otherwise, wait for one
        if (!(frame_counter_ > last_returned_frame_counter_)) {
            while (!data_cv_.wait_for(lock, kGetWaitTimeout, [this]() {
                bool new_frame_available = frame_counter_ > last_returned_frame_counter_;
                return new_frame_available || should_close_ || stop_thread_ ||
                       !camera_connected_.load();
            })) {
                wait_loops++;
                if (!camera_connected_.load()) return false;
            }
        }
    }

    if (should_close_ || stop_thread_) return false;
    if (!camera_connected_) return false;

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

    const double get_wait_ms = elapsed_ms(get_start);
    if (wait_loops > 0 && get_wait_ms > kGetWaitWarnMs) {
        spdlog::warn("validation: zed_get_wait slow elapsed_ms={:.2f} wait_loops={} get_depth={}",
                     get_wait_ms, wait_loops, get_depth);
    }
    return true;
}

bool ZedRgbdCamera::should_close() { return should_close_; }

bool ZedRgbdCamera::set_svo_recording_enabled(bool enabled) {
    if (is_playback_input_) {
        spdlog::warn("Ignoring SVO recording toggle while using SVO playback input.");
        return false;
    }

    const bool already_enabled = svo_recording_enabled_.load();
    if (already_enabled == enabled) return true;

    if (!is_initialized_) {
        svo_recording_enabled_.store(enabled);
        return true;
    }

    if (enabled) {
        std::filesystem::create_directories(svo_holding_dir_);
        enforce_holding_dir_size();
        if (!start_svo_recording()) return false;
        svo_recording_enabled_.store(true);
        return true;
    }

    stop_svo_recording();
    svo_recording_enabled_.store(false);
    return true;
}

bool ZedRgbdCamera::is_svo_recording_enabled() const {
    if (!svo_recording_enabled_.load()) return false;
    std::lock_guard<std::mutex> lock(data_mutex_);
    return !current_svo_path_.empty();
}

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
    svo_recording_enabled_.store(true);
    frames_since_size_check_ = 0;
    spdlog::info("SVO recording started: {}", path);
    return true;
}

void ZedRgbdCamera::stop_svo_recording() {
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (current_svo_path_.empty()) {
            svo_recording_enabled_.store(false);
            return;
        }
    }
    zed_.disableRecording();
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        spdlog::info("SVO recording stopped: {}", current_svo_path_.string());
        current_svo_path_.clear();
    }
    svo_recording_enabled_.store(false);
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
