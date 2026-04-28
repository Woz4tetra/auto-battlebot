#include "rgbd_camera/zed_rgbd_camera.hpp"

#include <spdlog/spdlog.h>

#include <chrono>
#include <cstdlib>
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
constexpr auto kInitializePollInterval = std::chrono::milliseconds(20);
constexpr auto kIdleSleepInterval = std::chrono::milliseconds(20);
constexpr auto kThreadJoinHardTimeout = std::chrono::seconds(30);
constexpr double kGrabWarnMs = 250.0;
constexpr double kCaptureLockWarnMs = 150.0;
constexpr double kGetWaitWarnMs = 500.0;

double elapsed_ms(const std::chrono::steady_clock::time_point &start) {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start)
        .count();
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

    if (is_playback_input_) {
        std::filesystem::path svo_file_path = config.svo_file_path.c_str();
        std::filesystem::path svo_file_abs_path = std::filesystem::absolute(svo_file_path);
        spdlog::info("Resolved SVO path: {}", svo_file_abs_path.string());

        params_.input.setFromSVOFile(svo_file_abs_path.c_str());
        params_.svo_real_time_mode = config.svo_real_time_mode;
    }

    // Start the single capture thread that owns the sl::Camera handle for the entire
    // lifetime of this object. It idles (Request::Close) until initialize() is called.
    capture_thread_ = std::thread(&ZedRgbdCamera::capture_thread_loop, this);
}

ZedRgbdCamera::~ZedRgbdCamera() {
    request_.store(Request::Stop, std::memory_order_release);
    data_cv_.notify_all();

    if (!capture_thread_.joinable()) return;

    // Poll for thread completion with a hard deadline. If an SDK call (open/grab/close)
    // is genuinely wedged, the thread will never return; calling thread.join() unbounded
    // would hang the destructor too. Bail out via _exit(1) so systemd's Restart=always
    // brings the process back.
    const auto deadline = std::chrono::steady_clock::now() + kThreadJoinHardTimeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (state_.load(std::memory_order_acquire) == State::Idle &&
            request_.load(std::memory_order_acquire) == Request::Stop) {
            // Thread loop has observed Stop and is exiting; safe to join briefly.
            capture_thread_.join();
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    spdlog::critical(
        "ZedRgbdCamera destructor: capture thread stuck in an SDK call after {}s; "
        "_exit(1) so systemd can restart the process.",
        std::chrono::duration_cast<std::chrono::seconds>(kThreadJoinHardTimeout).count());
    spdlog::default_logger()->flush();
    std::_Exit(1);
}

bool ZedRgbdCamera::initialize() {
    const auto initialize_start = std::chrono::steady_clock::now();

    // Tear down any existing session. If the camera is already open from a previous
    // initialize() (re-init after a transient failure), ask the capture thread to close
    // and wait until it reaches Idle.
    request_.store(Request::Close, std::memory_order_release);
    while (state_.load(std::memory_order_acquire) != State::Idle) {
        std::this_thread::sleep_for(kInitializePollInterval);
    }

    // Now that the capture thread is idle, reset everything that get() and the capture
    // loop read so the next session starts from a clean slate.
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        frame_counter_.store(0);
        depth_frame_counter_ = 0;
        last_returned_frame_counter_ = 0;
        while (!depth_request_queue_.empty()) depth_request_queue_.pop();
    }
    recent_grab_results_.clear();
    recent_grab_error_count_ = 0;
    prev_tracking_state_ = sl::POSITIONAL_TRACKING_STATE::LAST;
    should_close_.store(false, std::memory_order_release);
    last_open_error_.store(sl::ERROR_CODE::SUCCESS, std::memory_order_release);
    reset_capture_timing_stats();

    // Ask the capture thread to open and wait until it reports Ready (success) or comes
    // back to Idle after passing through Opening (failure).
    request_.store(Request::Open, std::memory_order_release);
    bool saw_opening = false;
    while (true) {
        const State current = state_.load(std::memory_order_acquire);
        if (current == State::Ready) {
            const double initialize_ms = elapsed_ms(initialize_start);
            if (initialize_ms > 2000.0) {
                spdlog::warn("validation: zed_initialize_total slow elapsed_ms={:.2f}",
                             initialize_ms);
            }
            return true;
        }
        if (current == State::Opening) saw_opening = true;
        if (saw_opening && current == State::Idle) {
            spdlog::error("Failed to open ZED camera: {}",
                          sl::toString(last_open_error_.load()).c_str());
            return false;
        }
        std::this_thread::sleep_for(kInitializePollInterval);
    }
}

void ZedRgbdCamera::reset_capture_timing_stats() const {
    captures_since_last_report_ = 0;
    capture_time_sum_ms_ = 0.0;
    capture_time_min_ms_ = std::numeric_limits<double>::infinity();
    capture_time_max_ms_ = 0.0;
}

void ZedRgbdCamera::capture_thread_loop() {
    // Park in Idle until the request changes away from Open. Used after a failed open
    // or after a sticky fatal flag is raised, so the thread does not auto-retry; the
    // runner's recovery loop must call initialize() again, which clears the flag and
    // cycles request_ Close→Open to trigger a fresh attempt.
    auto park_until_request_changes = [this]() {
        state_.store(State::Idle, std::memory_order_release);
        data_cv_.notify_all();
        while (request_.load(std::memory_order_acquire) == Request::Open) {
            std::this_thread::sleep_for(kIdleSleepInterval);
        }
    };

    while (request_.load(std::memory_order_acquire) != Request::Stop) {
        if (request_.load(std::memory_order_acquire) != Request::Open) {
            state_.store(State::Idle, std::memory_order_release);
            std::this_thread::sleep_for(kIdleSleepInterval);
            continue;
        }
        if (should_close_.load(std::memory_order_acquire)) {
            park_until_request_changes();
            continue;
        }

        // Open phase
        state_.store(State::Opening, std::memory_order_release);
        if (!open_and_configure()) {
            // open_and_configure() guarantees the camera is closed on failure.
            park_until_request_changes();
            continue;
        }
        state_.store(State::Ready, std::memory_order_release);

        if (svo_recording_enabled_.load()) {
            std::filesystem::create_directories(svo_holding_dir_);
            enforce_holding_dir_size();
            start_svo_recording();
        }

        // Grab phase: stay until request flips off Open or a sticky fatal flag is set.
        while (request_.load(std::memory_order_acquire) == Request::Open &&
               !should_close_.load(std::memory_order_acquire)) {
            apply_record_intent();
            grab_one_frame();
        }

        // Close phase
        state_.store(State::Closing, std::memory_order_release);
        stop_svo_recording();
        zed_.close();
        state_.store(State::Idle, std::memory_order_release);
        data_cv_.notify_all();
    }

    // Final exit: ensure we leave the camera closed even if a Stop arrives mid-run.
    state_.store(State::Idle, std::memory_order_release);
}

bool ZedRgbdCamera::open_and_configure() {
    sl::ERROR_CODE returned_state = zed_.open(params_);
    last_open_error_.store(returned_state, std::memory_order_release);
    if (returned_state != sl::ERROR_CODE::SUCCESS) {
        return false;  // open() failure leaves nothing to close.
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
        sl::PositionalTrackingParameters tracking_params;
        tracking_params.enable_imu_fusion = true;
        returned_state = zed_.enablePositionalTracking(tracking_params);
        if (returned_state != sl::ERROR_CODE::SUCCESS) {
            spdlog::error("Failed to enable positional tracking: {}",
                          sl::toString(returned_state).c_str());
            last_open_error_.store(returned_state, std::memory_order_release);
            zed_.close();
            return false;
        }
    } else {
        spdlog::info("Position tracking is disabled");
    }

    sl::CalibrationParameters calibration =
        zed_.getCameraInformation().camera_configuration.calibration_parameters;
    sl::Resolution image_size = zed_.getCameraInformation().camera_configuration.resolution;

    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_data_.camera_info.width = static_cast<int>(image_size.width);
    latest_data_.camera_info.height = static_cast<int>(image_size.height);
    latest_data_.camera_info.intrinsics = cv::Mat::eye(3, 3, CV_64F);
    latest_data_.camera_info.intrinsics.at<double>(0, 0) = calibration.left_cam.fx;
    latest_data_.camera_info.intrinsics.at<double>(1, 1) = calibration.left_cam.fy;
    latest_data_.camera_info.intrinsics.at<double>(0, 2) = calibration.left_cam.cx;
    latest_data_.camera_info.intrinsics.at<double>(1, 2) = calibration.left_cam.cy;

    latest_data_.camera_info.distortion = cv::Mat::zeros(1, 5, CV_64F);
    latest_data_.camera_info.distortion.at<double>(0, 0) = calibration.left_cam.disto[0];  // k1
    latest_data_.camera_info.distortion.at<double>(0, 1) = calibration.left_cam.disto[1];  // k2
    latest_data_.camera_info.distortion.at<double>(0, 2) = calibration.left_cam.disto[2];  // p1
    latest_data_.camera_info.distortion.at<double>(0, 3) = calibration.left_cam.disto[3];  // p2
    latest_data_.camera_info.distortion.at<double>(0, 4) = calibration.left_cam.disto[4];  // k3

    return true;
}

void ZedRgbdCamera::apply_record_intent() {
    const RecordIntent intent = record_intent_.exchange(RecordIntent::None);
    if (intent == RecordIntent::None) return;

    if (intent == RecordIntent::Start) {
        std::filesystem::create_directories(svo_holding_dir_);
        enforce_holding_dir_size();
        start_svo_recording();
    } else {
        stop_svo_recording();
    }
}

bool ZedRgbdCamera::grab_one_frame() {
    auto capture_start = std::chrono::steady_clock::now();

    bool need_depth = false;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        need_depth = !depth_request_queue_.empty();
        if (need_depth) depth_request_queue_.pop();
    }

    sl::RuntimeParameters rt_params;
    rt_params.enable_depth = true;
    const auto grab_start = std::chrono::steady_clock::now();
    sl::ERROR_CODE grab_status = zed_.grab(rt_params);
    const double grab_ms = elapsed_ms(grab_start);
    if (grab_ms > kGrabWarnMs) {
        spdlog::warn("validation: zed_grab slow elapsed_ms={:.2f}", grab_ms);
    }

    if (is_transient_grab_error(grab_status)) {
        return false;  // wait for the next good frame
    }

    const auto now = std::chrono::steady_clock::now();
    const bool grab_failed = (grab_status != sl::ERROR_CODE::SUCCESS);
    recent_grab_results_.emplace_back(now, grab_failed);
    if (grab_failed) recent_grab_error_count_++;
    while (!recent_grab_results_.empty() &&
           (now - recent_grab_results_.front().first) > kGrabErrorWindow) {
        if (recent_grab_results_.front().second) recent_grab_error_count_--;
        recent_grab_results_.pop_front();
    }

    if (grab_failed) {
        const bool window_is_full =
            !recent_grab_results_.empty() &&
            (now - recent_grab_results_.front().first) >= kGrabErrorWindow;
        if (window_is_full) {
            const double grab_error_ratio = static_cast<double>(recent_grab_error_count_) /
                                            static_cast<double>(recent_grab_results_.size());
            if (grab_error_ratio > kGrabErrorExitThreshold) {
                should_close_.store(true, std::memory_order_release);
                data_cv_.notify_all();
                spdlog::error(
                    "Camera grab error ratio {:.1f}% over last 10s exceeded {:.0f}% "
                    "threshold. Requesting application shutdown.",
                    grab_error_ratio * 100.0, kGrabErrorExitThreshold * 100.0);
                return false;
            }
        }

        if (grab_status == sl::ERROR_CODE::END_OF_SVOFILE_REACHED) {
            should_close_.store(true, std::memory_order_release);
            data_cv_.notify_all();
            spdlog::info("End of SVO file reached.");
        } else {
            spdlog::error("Failed to grab frame: {}", sl::toString(grab_status).c_str());
            data_cv_.notify_all();
        }
        return false;
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

    const auto lock_hold_start = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(data_mutex_);

    sl::ERROR_CODE retrieve_status = zed_.retrieveImage(zed_rgb_, sl::VIEW::LEFT);
    if (retrieve_status != sl::ERROR_CODE::SUCCESS) {
        spdlog::error("Failed to retrieve RGB image: {}", sl::toString(retrieve_status).c_str());
        return false;
    }

    if (need_depth) {
        retrieve_status = zed_.retrieveMeasure(zed_depth_, sl::MEASURE::DEPTH);
        if (retrieve_status != sl::ERROR_CODE::SUCCESS) {
            spdlog::error("Failed to retrieve depth image: {}",
                          sl::toString(retrieve_status).c_str());
            depth_request_queue_.push(1);
            return false;
        }
    }

    sl::Timestamp timestamp = zed_.getTimestamp(sl::TIME_REFERENCE::CURRENT);
    double stamp = static_cast<double>(timestamp.getNanoseconds()) / 1e9;

    latest_data_.tf_visodom_from_camera.header.stamp = stamp;
    latest_data_.tf_visodom_from_camera.header.frame_id = FrameId::VISUAL_ODOMETRY;
    latest_data_.tf_visodom_from_camera.child_frame_id = FrameId::CAMERA;

    if (position_tracking_enabled_) {
        sl::POSITIONAL_TRACKING_STATE tracking_state =
            zed_.getPosition(zed_pose_, sl::REFERENCE_FRAME::WORLD);
        latest_data_.tracking_ok = (tracking_state == sl::POSITIONAL_TRACKING_STATE::OK);
        if (tracking_state != prev_tracking_state_) {
            spdlog::info("Tracking state: {}", sl::toString(tracking_state).c_str());
            prev_tracking_state_ = tracking_state;
        }

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

    cv::Mat zed_rgb_mat(zed_rgb_.getHeight(), zed_rgb_.getWidth(), CV_8UC4,
                        zed_rgb_.getPtr<sl::uchar1>());
    cv::cvtColor(zed_rgb_mat, latest_data_.rgb.image, cv::COLOR_BGRA2BGR);

    if (need_depth) {
        cv::Mat zed_depth_mat(zed_depth_.getHeight(), zed_depth_.getWidth(), CV_32FC1,
                              zed_depth_.getPtr<sl::uchar1>());
        zed_depth_mat.copyTo(latest_data_.depth.image);
        depth_frame_counter_ = frame_counter_.load() + 1;
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

    frame_counter_.fetch_add(1);
    data_cv_.notify_all();
    return true;
}

bool ZedRgbdCamera::get(CameraData &data, bool get_depth) {
    if (state_.load(std::memory_order_acquire) != State::Ready) return false;
    const auto get_start = std::chrono::steady_clock::now();
    int wait_loops = 0;
    std::unique_lock<std::mutex> lock(data_mutex_);

    auto capture_stopped = [this]() {
        return state_.load(std::memory_order_acquire) != State::Ready ||
               should_close_.load(std::memory_order_acquire) ||
               request_.load(std::memory_order_acquire) != Request::Open;
    };

    if (get_depth) {
        depth_request_queue_.push(1);
        uint64_t current_frame = frame_counter_.load();
        uint64_t current_depth_frame = depth_frame_counter_;
        while (!data_cv_.wait_for(lock, kGetWaitTimeout,
                                  [this, current_frame, current_depth_frame, &capture_stopped]() {
                                      bool new_frame = frame_counter_.load() > current_frame;
                                      bool depth_ready = depth_frame_counter_ > current_depth_frame;
                                      return (new_frame && depth_ready) || capture_stopped();
                                  })) {
            wait_loops++;
            if (capture_stopped()) return false;
        }
    } else {
        if (!(frame_counter_.load() > last_returned_frame_counter_)) {
            while (!data_cv_.wait_for(lock, kGetWaitTimeout, [this, &capture_stopped]() {
                bool new_frame_available = frame_counter_.load() > last_returned_frame_counter_;
                return new_frame_available || capture_stopped();
            })) {
                wait_loops++;
                if (capture_stopped()) return false;
            }
        }
    }

    if (capture_stopped()) return false;

    data = latest_data_;
    last_returned_frame_counter_ = frame_counter_.load();

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

bool ZedRgbdCamera::should_close() { return should_close_.load(std::memory_order_acquire); }

bool ZedRgbdCamera::set_recording_enabled(bool enabled) {
    if (is_playback_input_) {
        spdlog::warn("Ignoring SVO recording toggle while using SVO playback input.");
        return false;
    }

    const bool already_enabled = svo_recording_enabled_.load();
    if (already_enabled == enabled) return true;

    svo_recording_enabled_.store(enabled);

    // If the capture thread is not currently running a session, the new value will be
    // honored on the next Open transition (see capture_thread_loop). Otherwise queue an
    // intent for the running thread to apply between grabs.
    if (state_.load(std::memory_order_acquire) == State::Ready) {
        record_intent_.store(enabled ? RecordIntent::Start : RecordIntent::Stop,
                             std::memory_order_release);
    }
    return true;
}

bool ZedRgbdCamera::is_recording_enabled() const {
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
