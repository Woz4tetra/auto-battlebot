#include "rgbd_camera/zed_one_rgb_camera.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <foxglove/schemas.hpp>
#include <magic_enum.hpp>
#include <stdexcept>
#include <utility>

#include "foxglove_adapters/common.hpp"
#include "rgbd_camera/camera_calibration.hpp"
#include "rgbd_camera/zed_device.hpp"
#include "tensorrt_inference/pinned_mat_allocator.hpp"

namespace auto_battlebot {
namespace {
constexpr auto kGrabErrorWindow = std::chrono::seconds(10);
constexpr double kGrabErrorExitThreshold = 0.70;
constexpr auto kGetWaitTimeout = std::chrono::milliseconds(100);
/** grab() returns within a frame period on a working camera. */
constexpr auto kJoinTimeout = std::chrono::seconds(3);
constexpr const char *kVideoTopic = "/camera/video";
}  // namespace

// AE_ANTIBANDING is in SDK 5.5 (dev machines) but not 5.2 (the ZED Box). Without it the control
// reports unsupported and the dashboard hides it; a profile's anti_banding is kept but not sent.
#define AB_ZED_HAS_ANTIBANDING \
    (ZED_SDK_MAJOR_VERSION > 5 || (ZED_SDK_MAJOR_VERSION == 5 && ZED_SDK_MINOR_VERSION >= 5))

ZedOneRgbCamera::ZedOneRgbCamera(const ZedOneRgbCameraConfiguration &config,
                                 std::shared_ptr<McapRecorder> mcap_recorder)
    : config_(config),
      grab_health_(kGrabErrorWindow, kGrabErrorExitThreshold),
      mcap_recorder_(std::move(mcap_recorder)),
      diagnostics_logger_(DiagnosticsLogger::get_logger("zed_one_rgb_camera")) {
    recording_desired_.store(config_.video_recording);
}

ZedOneRgbCamera::~ZedOneRgbCamera() {
    stop_thread_.store(true);
    data_cv_.notify_all();
    if (capture_thread_.joinable()) {
        const auto deadline = std::chrono::steady_clock::now() + kJoinTimeout;
        while (!capture_thread_done_.load() && std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        if (!capture_thread_done_.load()) {
            // Joining would hang until systemd kills the process 90 s later. Detaching would
            // leave the thread inside grab() on a camera this destructor is about to free.
            // Runner::run() closed the recordings before any destructor ran, so exit now.
            spdlog::critical(
                "[ZedOneRgbCamera] Capture thread stuck in grab() for {} s at "
                "shutdown; exiting",
                kJoinTimeout.count());
            spdlog::default_logger()->flush();
            std::_Exit(EXIT_FAILURE);
        }
        capture_thread_.join();
    }
    encoder_.stop();
    if (pending_open_.valid()) {
        // open() cannot be interrupted, and the handle must not be closed while it runs.
        pending_open_.wait();
    }
    zed_.close();
}

bool ZedOneRgbCamera::open_camera() {
    sl::InitParametersOne params;
    params.camera_resolution = get_zed_resolution(config_.camera_resolution);
    params.camera_fps = config_.camera_fps;
    params.sdk_verbose = 0;

    cancel_open_.store(false);
    pending_open_ = std::async(std::launch::async, [this, params]() { return zed_.open(params); });
    while (pending_open_.wait_for(std::chrono::milliseconds(50)) != std::future_status::ready) {
        if (cancel_open_.load()) {
            pending_open_.wait();
            pending_open_.get();
            zed_.close();
            return false;
        }
    }
    const sl::ERROR_CODE result = pending_open_.get();
    if (result != sl::ERROR_CODE::SUCCESS) {
        spdlog::error("[ZedOneRgbCamera] Failed to open ZED X One: {}",
                      sl::toString(result).c_str());
        return false;
    }
    return true;
}

void ZedOneRgbCamera::read_calibration() {
    const sl::CameraOneInformation information = zed_.getCameraInformation();
    const sl::CameraParameters &rectified = information.camera_configuration.calibration_parameters;
    width_ = static_cast<int>(information.camera_configuration.resolution.width);
    height_ = static_cast<int>(information.camera_configuration.resolution.height);

    CameraInfo info;
    info.width = width_;
    info.height = height_;
    info.intrinsics = cv::Mat::eye(3, 3, CV_64F);
    info.intrinsics.at<double>(0, 0) = rectified.fx;
    info.intrinsics.at<double>(1, 1) = rectified.fy;
    info.intrinsics.at<double>(0, 2) = rectified.cx;
    info.intrinsics.at<double>(1, 2) = rectified.cy;
    // Zeros: the SDK already removed the distortion, so camera_info emits a trivial plumb_bob.
    info.distortion = cv::Mat::zeros(1, 5, CV_64F);
    latest_data_.camera_info = info;

    // Rectified intrinsics with zero distortion describe the recorded video exactly. Resolution
    // is in the id because the cropped 1920x1080 mode shifts cy.
    calibration_.calibration_id = "zed_x_one_" + std::to_string(information.serial_number) + "_" +
                                  std::to_string(width_) + "x" + std::to_string(height_);
    calibration_.width = width_;
    calibration_.height = height_;
    calibration_.fx = rectified.fx;
    calibration_.fy = rectified.fy;
    calibration_.cx = rectified.cx;
    calibration_.cy = rectified.cy;

    spdlog::info("[ZedOneRgbCamera] {} serial {} at {}x{} {} fps, fx {:.1f} fy {:.1f}",
                 sl::toString(information.camera_model).c_str(), information.serial_number, width_,
                 height_, information.camera_configuration.fps, rectified.fx, rectified.fy);
}

bool ZedOneRgbCamera::start_encoder() {
    video_channel_ = std::make_unique<OutputChannel>(
        kVideoTopic, "protobuf", VizSchema::from_sdk(foxglove::schemas::CompressedVideo::schema()),
        false, nullptr, mcap_recorder_);
    VideoEncoderOptions encoder_options;
    encoder_options.width = width_;
    encoder_options.height = height_;
    encoder_options.fps = config_.camera_fps;
    encoder_options.bitrate = config_.video_bitrate_kbps * 1000;
    encoder_options.input_format = VideoInputFormat::BGRA;
    if (!encoder_.start(encoder_options, [this](const std::byte *data, size_t len,
                                                uint64_t log_time_ns, bool /*keyframe*/) {
            foxglove::schemas::CompressedVideo message;
            // Foxglove's decoder rejects a frame whose timestamp is not after the previous one.
            message.timestamp =
                foxglove_adapters::to_timestamp(static_cast<double>(log_time_ns) * 1e-9);
            message.frame_id = "camera";
            message.format = "h264";
            message.data.assign(data, data + len);
            video_channel_->log_message(message, log_time_ns);
        })) {
        video_channel_.reset();
        return false;
    }
    return true;
}

bool ZedOneRgbCamera::initialize() {
    if (is_initialized_.load()) {
        return true;
    }
    if (!open_camera()) {
        return false;
    }
    read_calibration();
    // Nothing is set on the camera at startup: DEFAULT is whatever it opened with, kept so a
    // dashboard switch back to DEFAULT can restore it.
    opened_settings_ = read_settings();
    settings_state_ = CameraSettingsState{opened_settings_, read_limits()};
    latest_data_.tracking_ok = true;

    if (mcap_recorder_) {
        // VideoPlaybackCamera reads this to replay the file; no config/cameras/ file needed.
        mcap_recorder_->write_metadata(kCameraCalibrationMetadataKey,
                                       camera_calibration_to_toml(calibration_));
    }
    // Fatal: a session recorded without its video cannot be replayed, and nobody finds out until
    // the morning after. Turning recording off in the config is the explicit way past it.
    if (mcap_recorder_ && recording_desired_.load() && !start_encoder()) {
        throw std::runtime_error(
            "[ZedOneRgbCamera] video_recording is on but no H.264 encoder opened (tried " +
            VideoEncoder::candidate_names() +
            "). Fix the encoder or set [rgbd_camera] video_recording = false.");
    }

    is_initialized_.store(true);
    stop_thread_.store(false);
    capture_thread_ = std::thread(&ZedOneRgbCamera::capture_thread_loop, this);
    return true;
}

bool ZedOneRgbCamera::capture_frame() {
    // grab() blocks until the next frame, so the loop runs at the sensor rate.
    const sl::ERROR_CODE grab_result = zed_.grab();
    if (grab_result == sl::ERROR_CODE::SUCCESS) {
        grab_health_.record(false, std::chrono::steady_clock::now());
    } else {
        grab_health_.record(true, std::chrono::steady_clock::now());
        if (grab_health_.should_shutdown() && !should_close_.load()) {
            should_close_.store(true);
            data_cv_.notify_all();
            spdlog::error(
                "[ZedOneRgbCamera] Grab error ratio {:.1f}% over last 10s exceeded {:.0f}% "
                "(last: {}). Requesting application shutdown.",
                grab_health_.error_ratio() * 100.0, kGrabErrorExitThreshold * 100.0,
                sl::toString(grab_result).c_str());
        }
        return false;
    }
    const uint64_t capture_time_ns = zed_.getTimestamp(sl::TIME_REFERENCE::IMAGE).getNanoseconds();

    // VIEW::LEFT is rectified BGRA. LEFT_BGR would skip the conversion, but it is newer than the
    // SDK 5.2 on the ZED Box Mini.
    if (zed_.retrieveImage(zed_bgra_, sl::VIEW::LEFT, sl::MEM::CPU) != sl::ERROR_CODE::SUCCESS) {
        return false;
    }
    const cv::Mat bgra(static_cast<int>(zed_bgra_.getHeight()),
                       static_cast<int>(zed_bgra_.getWidth()), CV_8UC4,
                       zed_bgra_.getPtr<sl::uchar1>(sl::MEM::CPU), zed_bgra_.getStepBytes());
    // A fresh Mat every frame: the pipeline holds the previous one by reference. Pinned, so the
    // keypoint model's GPU preprocess reads it in place instead of uploading it.
    cv::Mat bgr;
    bgr.allocator = PinnedMatAllocator::instance();
    cv::cvtColor(bgra, bgr, cv::COLOR_BGRA2BGR);

    // The encoder numbers the frames it takes; frame_meta carries that number, so it matches the
    // video packet in whichever file the frame lands in. -1 when it took none. BGRA, not the BGR
    // copy: the Jetson's nvvidconv takes 4-channel input directly.
    const int64_t video_frame_index = encoder_.submit(bgra, wall_time_ns());

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_data_.rgb.image = bgr;
        latest_data_.rgb.header.stamp = static_cast<double>(capture_time_ns) * 1e-9;
        latest_data_.rgb.header.frame_id = FrameId::CAMERA;
        // Foxglove image panels reject a calibration whose frame differs from the image's.
        latest_data_.camera_info.header = latest_data_.rgb.header;
        latest_data_.depth.image = cv::Mat();
        latest_data_.tracking_ok = true;
        // A clamped camera does not move: no visual odometry, no camera-world rebase.
        latest_data_.tf_visodom_from_camera.header.stamp = latest_data_.rgb.header.stamp;
        latest_data_.tf_visodom_from_camera.header.frame_id = FrameId::VISUAL_ODOMETRY;
        latest_data_.tf_visodom_from_camera.child_frame_id = FrameId::CAMERA;
        latest_data_.tf_visodom_from_camera.transform.tf = Eigen::Matrix4d::Identity();
        latest_data_.frame_identity.image_stamp_ns = capture_time_ns;
        latest_data_.frame_identity.video_frame_index = video_frame_index;
        frame_counter_++;
    }
    return true;
}

void ZedOneRgbCamera::capture_thread_loop() {
    while (!stop_thread_.load()) {
        service_settings();
        if (capture_frame()) {
            data_cv_.notify_all();
        } else if (should_close_.load()) {
            break;
        }
    }
    capture_thread_done_.store(true);
}

bool ZedOneRgbCamera::get(CameraData &data) {
    if (!is_initialized_.load()) {
        return false;
    }
    std::unique_lock<std::mutex> lock(data_mutex_);
    if (frame_counter_ <= last_returned_frame_counter_) {
        data_cv_.wait_for(lock, kGetWaitTimeout, [this] {
            return frame_counter_ > last_returned_frame_counter_ || should_close_.load() ||
                   stop_thread_.load();
        });
    }
    if (should_close_.load() || stop_thread_.load()) {
        return false;
    }
    if (frame_counter_ <= last_returned_frame_counter_) {
        return false;
    }
    data = latest_data_;
    last_returned_frame_counter_ = frame_counter_;

    const uint64_t drops = encoder_.dropped_frames();
    if (drops != reported_drops_) {
        spdlog::warn("[ZedOneRgbCamera] Encoder has dropped {} frame(s); it is behind the capture",
                     drops);
        diagnostics_logger_->log(1, "video_encoder", {{"dropped_frames", std::to_string(drops)}});
        reported_drops_ = drops;
    }
    return true;
}

bool ZedOneRgbCamera::set_recording_enabled(bool enabled) {
    recording_desired_.store(enabled);
    if (!is_initialized_.load()) {
        return true;
    }
    if (!enabled) {
        encoder_.stop();
        return true;
    }
    if (encoder_.running()) {
        return true;
    }
    // A recording started from the remote UI after launch lands here, not in initialize(). Not
    // fatal here: the UI reports the failed toggle, and killing a running match over it is worse.
    if (!mcap_recorder_ || !start_encoder()) {
        spdlog::error("[ZedOneRgbCamera] Recording toggled on but no H.264 encoder opened");
        return false;
    }
    return true;
}

bool ZedOneRgbCamera::is_recording_enabled() const { return encoder_.running(); }
void ZedOneRgbCamera::request_camera_settings(std::optional<CameraSettings> settings) {
    std::lock_guard<std::mutex> lock(settings_mutex_);
    pending_settings_ = std::move(settings);
}

std::optional<CameraSettingsState> ZedOneRgbCamera::camera_settings() const {
    std::lock_guard<std::mutex> lock(settings_mutex_);
    return settings_state_;
}

CameraSettings ZedOneRgbCamera::read_settings() {
    auto get = [this](sl::VIDEO_SETTINGS setting, int fallback) {
        int value = fallback;
        if (!zed_.isCameraSettingSupported(setting) ||
            zed_.getCameraSettings(setting, value) != sl::ERROR_CODE::SUCCESS) {
            return fallback;
        }
        return value;
    };
    CameraSettings s;
    s.auto_exposure = get(sl::VIDEO_SETTINGS::AEC_AGC, 1) != 0;
    s.exposure_us = get(sl::VIDEO_SETTINGS::EXPOSURE_TIME, s.exposure_us);
    s.analog_gain_mdb = get(sl::VIDEO_SETTINGS::ANALOG_GAIN, s.analog_gain_mdb);
    s.digital_gain = get(sl::VIDEO_SETTINGS::DIGITAL_GAIN, s.digital_gain);
    s.auto_white_balance = get(sl::VIDEO_SETTINGS::WHITEBALANCE_AUTO, 1) != 0;
    s.white_balance_k = get(sl::VIDEO_SETTINGS::WHITEBALANCE_TEMPERATURE, s.white_balance_k);
    s.saturation = get(sl::VIDEO_SETTINGS::SATURATION, s.saturation);
    s.gamma = get(sl::VIDEO_SETTINGS::GAMMA, s.gamma);
    s.sharpness = get(sl::VIDEO_SETTINGS::SHARPNESS, s.sharpness);
#if AB_ZED_HAS_ANTIBANDING
    s.anti_banding = magic_enum::enum_cast<AntiBanding>(get(sl::VIDEO_SETTINGS::AE_ANTIBANDING, 0))
                         .value_or(AntiBanding::OFF);
#endif
    return s;
}

CameraSettingLimits ZedOneRgbCamera::read_limits() {
    auto range = [this](sl::VIDEO_SETTINGS setting) {
        CameraSettingRange r;
        r.supported = zed_.isCameraSettingSupported(setting) &&
                      zed_.getCameraSettingsRange(setting, r.min, r.max) == sl::ERROR_CODE::SUCCESS;
        return r;
    };
    CameraSettingLimits limits;
    limits.exposure_us = range(sl::VIDEO_SETTINGS::EXPOSURE_TIME);
    // The sensor cannot expose longer than one frame at the rate it opened with.
    limits.exposure_us.max = std::min(limits.exposure_us.max, 1'000'000 / config_.camera_fps);
    limits.analog_gain_mdb = range(sl::VIDEO_SETTINGS::ANALOG_GAIN);
    limits.digital_gain = range(sl::VIDEO_SETTINGS::DIGITAL_GAIN);
    limits.white_balance_k = range(sl::VIDEO_SETTINGS::WHITEBALANCE_TEMPERATURE);
    limits.saturation = range(sl::VIDEO_SETTINGS::SATURATION);
    limits.gamma = range(sl::VIDEO_SETTINGS::GAMMA);
    limits.sharpness = range(sl::VIDEO_SETTINGS::SHARPNESS);
#if AB_ZED_HAS_ANTIBANDING
    limits.anti_banding = range(sl::VIDEO_SETTINGS::AE_ANTIBANDING);
#endif
    return limits;
}

void ZedOneRgbCamera::apply_settings(const CameraSettings &settings) {
    CameraSettingLimits limits;
    {
        std::lock_guard<std::mutex> lock(settings_mutex_);
        limits = settings_state_->limits;
    }
    auto set = [this](sl::VIDEO_SETTINGS setting, int value, const CameraSettingRange &range) {
        if (!range.supported) return;
        const int clamped = std::clamp(value, range.min, range.max);
        const sl::ERROR_CODE result = zed_.setCameraSettings(setting, clamped);
        if (result != sl::ERROR_CODE::SUCCESS) {
            spdlog::warn("[ZedOneRgbCamera] setCameraSettings({}, {}) failed: {}",
                         sl::toString(setting).c_str(), clamped, sl::toString(result).c_str());
        }
    };
    const CameraSettingRange on_off{true, 0, 1};
    if (settings.auto_exposure) {
        set(sl::VIDEO_SETTINGS::AEC_AGC, 1, on_off);
    } else {
        set(sl::VIDEO_SETTINGS::AEC_AGC, 0, on_off);
        set(sl::VIDEO_SETTINGS::EXPOSURE_TIME, settings.exposure_us, limits.exposure_us);
        set(sl::VIDEO_SETTINGS::ANALOG_GAIN, settings.analog_gain_mdb, limits.analog_gain_mdb);
        set(sl::VIDEO_SETTINGS::DIGITAL_GAIN, settings.digital_gain, limits.digital_gain);
    }
    if (settings.auto_white_balance) {
        set(sl::VIDEO_SETTINGS::WHITEBALANCE_AUTO, 1, on_off);
    } else {
        set(sl::VIDEO_SETTINGS::WHITEBALANCE_TEMPERATURE, settings.white_balance_k,
            limits.white_balance_k);
    }
    set(sl::VIDEO_SETTINGS::SATURATION, settings.saturation, limits.saturation);
    set(sl::VIDEO_SETTINGS::GAMMA, settings.gamma, limits.gamma);
    set(sl::VIDEO_SETTINGS::SHARPNESS, settings.sharpness, limits.sharpness);
#if AB_ZED_HAS_ANTIBANDING
    set(sl::VIDEO_SETTINGS::AE_ANTIBANDING, static_cast<int>(settings.anti_banding),
        limits.anti_banding);
#endif
}

void ZedOneRgbCamera::service_settings() {
    std::optional<std::optional<CameraSettings>> request;
    {
        std::lock_guard<std::mutex> lock(settings_mutex_);
        request.swap(pending_settings_);
    }
    if (request) {
        apply_settings(request->value_or(opened_settings_));
    }
    // Auto exposure and white balance move on their own; read them back about twice a second
    // and right after a change.
    constexpr int kReadbackPeriodFrames = 30;
    if (!request && ++frames_since_settings_read_ < kReadbackPeriodFrames) return;
    frames_since_settings_read_ = 0;
    const CameraSettings current = read_settings();
    std::lock_guard<std::mutex> lock(settings_mutex_);
    settings_state_->settings = current;
}

}  // namespace auto_battlebot
