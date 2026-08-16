#include "rgbd_camera/zed_rgbd_camera.hpp"

#include <spdlog/spdlog.h>

#include <chrono>
#include <filesystem>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

#include "directories.hpp"

namespace auto_battlebot {

namespace {
constexpr uint64_t kBytesPerGb = 1024ULL * 1024ULL * 1024ULL;
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

// Emit the standard "validation:" timing warning if the elapsed time since `start` exceeds
// `warn_ms`. Centralizes the repeated measure-and-warn pattern while keeping the log text
// byte-identical to the historical format.
void warn_if_slow(const std::chrono::steady_clock::time_point &start, const char *label,
                  double warn_ms) {
    const double ms = elapsed_ms(start);
    if (ms > warn_ms) {
        spdlog::warn("validation: {} slow elapsed_ms={:.2f}", label, ms);
    }
}

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
}  // namespace

ZedRgbdCamera::ZedRgbdCamera(ZedRgbdCameraConfiguration &config)
    : is_initialized_(false),
      should_close_(false),
      stop_thread_(false),
      camera_connected_(false),
      frame_counter_(0),
      depth_frame_counter_(0),
      last_returned_frame_counter_(0),
      grab_health_(kGrabErrorWindow, kGrabErrorExitThreshold) {
    diagnostics_logger_ = DiagnosticsLogger::get_logger("zed_rgbd_camera");
    reset_capture_timing_stats();

    device_.set_tracking_enabled(config.position_tracking);
    svo_recorder_ = std::make_unique<SvoRecorder>(
        device_.handle(), get_project_path("data/temp_svo"), config.svo_max_size_gb * kBytesPerGb,
        config.svo_holding_dir_max_size_gb * kBytesPerGb);
    svo_recorder_->set_desired(config.svo_recording);

    sl::InitParameters params;
    params.camera_fps = config.camera_fps;
    params.camera_resolution = get_zed_resolution(config.camera_resolution);
    params.depth_mode = get_zed_depth_mode(config.depth_mode);
    params.coordinate_system = sl::COORDINATE_SYSTEM::IMAGE;
    params.coordinate_units = sl::UNIT::METER;
    device_.set_params(params);
}

ZedRgbdCamera::~ZedRgbdCamera() {
    // Cancel any in-progress initialize() before doing anything else.
    cancel_open_ = true;
    if (device_.has_pending_open()) {
        // is_initialized_ is false (only set after a successful open returns), so on a leaked
        // open the capture-thread / zed_.close() block below is correctly skipped. Returning
        // also avoids racing zed_.close() against the still-running zed_.open() call.
        if (!device_.await_or_leak_open("ZedRgbdCamera destructor",
                                        "pending_open_wait_destructor")) {
            return;
        }
    }
    if (is_initialized_) {
        stop_thread_ = true;
        if (capture_thread_.joinable()) {
            data_cv_.notify_all();
            const auto join_start = std::chrono::steady_clock::now();
            const bool joined = join_with_timeout(capture_thread_, capture_thread_done_,
                                                  "ZedRgbdCamera destructor");
            warn_if_slow(join_start, "capture_thread_join_destructor", kJoinWaitWarnMs);
            if (!joined) {
                // Detached capture thread is potentially still inside zed_.grab(); calling
                // svo_recorder_->stop() / zed_.close() concurrently against the same handle
                // would race the SDK. Skip them and let the OS reap on process exit.
                return;
            }
        }
        svo_recorder_->stop();
        device_.close();
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
        warn_if_slow(join_start, "capture_thread_join_initialize", kJoinWaitWarnMs);
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
        svo_recorder_->stop();
        device_.close();
        is_initialized_ = false;
    }

    reset_runtime_state();

    cancel_open_ = false;
    const ZedDevice::OpenResult open_result = device_.open(cancel_open_);
    if (open_result == ZedDevice::OpenResult::Leaked) {
        should_close_ = true;
        return false;
    }
    if (open_result != ZedDevice::OpenResult::Opened) return false;

    if (!device_.enable_tracking()) return false;

    latest_data_.camera_info = device_.read_camera_info();

    is_initialized_ = true;

    if (svo_recorder_->desired()) {
        svo_recorder_->start();
    }

    // Start capture thread
    capture_thread_ = std::thread(&ZedRgbdCamera::capture_thread_loop, this);

    warn_if_slow(initialize_start, "zed_initialize_total", kOpenWaitWarnMs);
    return true;
}

void ZedRgbdCamera::reset_runtime_state() {
    stop_thread_ = false;
    should_close_ = false;
    camera_connected_ = true;
    capture_thread_done_.store(false, std::memory_order_release);
    frame_counter_ = 0;
    depth_frame_counter_ = 0;
    last_returned_frame_counter_ = 0;
    grab_health_.reset();
    device_.reset_runtime_state();
    reset_capture_timing_stats();
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        depth_requested_ = false;
    }
}

void ZedRgbdCamera::reset_capture_timing_stats() const {
    captures_since_last_report_ = 0;
    capture_time_sum_ms_ = 0.0;
    capture_time_min_ms_ = std::numeric_limits<double>::infinity();
    capture_time_max_ms_ = 0.0;
}

void ZedRgbdCamera::capture_thread_loop() {
    while (!stop_thread_) {
        // capture_frame() bumps frame_counter_ under data_mutex_ once the frame is fully
        // populated; we just wake any waiters here.
        if (capture_frame()) {
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
        need_depth = depth_requested_;
        depth_requested_ = false;
    }

    const ZedDevice::GrabStatus grab_status = device_.grab();

    if (grab_status == ZedDevice::GrabStatus::TransientError) {
        // Treat transient frame grab issues as recoverable: wait for the next good frame.
        return false;
    }

    if (grab_status != ZedDevice::GrabStatus::Ok) {
        camera_connected_ = false;
        grab_health_.record(true, std::chrono::steady_clock::now());

        if (grab_health_.should_shutdown() && !should_close_.load()) {
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                should_close_ = true;
            }
            data_cv_.notify_all();
            spdlog::error(
                "Camera grab error ratio {:.1f}% over last 10s exceeded {:.0f}% "
                "threshold. Requesting application shutdown.",
                grab_health_.error_ratio() * 100.0, kGrabErrorExitThreshold * 100.0);
            return false;
        }

        data_cv_.notify_all();
        return false;
    }
    camera_connected_ = true;
    grab_health_.record(false, std::chrono::steady_clock::now());

    // Roll the SVO file over if it has grown past the size cap. The return value says where
    // this frame landed, which is what lets a recording be joined back to its SVO frames.
    const SvoRecorder::FrameRef svo_frame = svo_recorder_->on_frame_grabbed();

    // Lock mutex to modify data
    const auto lock_hold_start = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(data_mutex_);

    if (!device_.retrieve(need_depth, latest_data_)) {
        if (need_depth) depth_requested_ = true;  // re-request depth for the next frame
        return false;
    }

    // Record which camera frame this is, independent of the stamp. TIME_REFERENCE::IMAGE runs
    // about half a frame ahead of what the SVO recorder writes for the same grab, so a
    // timestamp alone cannot join recorded output back to SVO frames after the fact.
    latest_data_.frame_identity.svo_frame_index = svo_frame.index;
    latest_data_.frame_identity.svo_path = svo_frame.path;
    // Publish the frame while holding data_mutex_: bump frame_counter_ (and depth_frame_counter_
    // when this frame carries depth) so waiting get() calls observe a fully populated frame.
    frame_counter_++;
    if (need_depth) {
        depth_frame_counter_ = frame_counter_;
    }

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
        // Request depth for the next frame and wait for a new frame that carries it.
        depth_requested_ = true;
        const uint64_t requested_after_frame = frame_counter_;
        const uint64_t requested_after_depth = depth_frame_counter_;
        if (!wait_for_new_frame(lock, wait_loops, [&]() {
                return frame_counter_ > requested_after_frame &&
                       depth_frame_counter_ > requested_after_depth;
            })) {
            return false;
        }
    } else {
        // Return a prefetched frame immediately; otherwise wait for the next one.
        if (!(frame_counter_ > last_returned_frame_counter_)) {
            if (!wait_for_new_frame(lock, wait_loops, [&]() {
                    return frame_counter_ > last_returned_frame_counter_;
                })) {
                return false;
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

bool ZedRgbdCamera::wait_for_new_frame(std::unique_lock<std::mutex> &lock, int &wait_loops,
                                       const std::function<bool()> &frame_ready) {
    while (!data_cv_.wait_for(lock, kGetWaitTimeout, [&]() {
        return frame_ready() || should_close_ || stop_thread_ || !camera_connected_.load();
    })) {
        wait_loops++;
        if (!camera_connected_.load()) return false;
    }
    return true;
}

bool ZedRgbdCamera::should_close() { return should_close_; }

bool ZedRgbdCamera::set_recording_enabled(bool enabled) {
    if (svo_recorder_->desired() == enabled) return true;

    // Before the camera is open, just remember the desired state; initialize() acts on it.
    if (!is_initialized_) {
        svo_recorder_->set_desired(enabled);
        return true;
    }

    if (enabled) {
        return svo_recorder_->start();
    }

    svo_recorder_->stop();
    return true;
}

bool ZedRgbdCamera::is_recording_enabled() const { return svo_recorder_->is_recording(); }

std::string ZedRgbdCamera::get_current_svo_path() const { return svo_recorder_->current_path(); }

}  // namespace auto_battlebot
