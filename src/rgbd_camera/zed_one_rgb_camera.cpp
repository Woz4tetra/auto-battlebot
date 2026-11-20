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
#include "rgbd_camera/zed_one_device.hpp"
#include "tensorrt_inference/pinned_mat_allocator.hpp"

namespace auto_battlebot {
namespace {
CameraSettings default_manual_settings(const CameraSettingLimits &limits) {
    CameraSettings unknown;
    unknown.exposure_us = limits.exposure_us.max / 2;
    unknown.analog_gain_mdb = limits.analog_gain_mdb.min;
    unknown.digital_gain = std::max(1, limits.digital_gain.min);
    return unknown;
}

constexpr auto kGrabErrorWindow = std::chrono::seconds(10);
constexpr double kGrabErrorExitThreshold = 0.70;
constexpr auto kGetWaitTimeout = std::chrono::milliseconds(100);
constexpr auto kCaptureThreadJoinTimeout = std::chrono::seconds(3);
constexpr const char *kVideoTopic = "/camera/video";
}  // namespace

#define AB_ZED_HAS_ANTIBANDING \
    (ZED_SDK_MAJOR_VERSION > 5 || (ZED_SDK_MAJOR_VERSION == 5 && ZED_SDK_MINOR_VERSION >= 5))

ZedOneRgbCamera::ZedOneRgbCamera(const ZedOneRgbCameraConfiguration &config,
                                 std::shared_ptr<McapRecorder> mcap_recorder,
                                 std::unique_ptr<ZedOneDevice> device,
                                 VideoEncoder::BackendFactory encoder_factory)
    : config_(config),
      device_(std::move(device)),
      encoder_factory_(std::move(encoder_factory)),
      grab_health_(kGrabErrorWindow, kGrabErrorExitThreshold),
      mcap_recorder_(std::move(mcap_recorder)),
      diagnostics_logger_(DiagnosticsLogger::get_logger("zed_one_rgb_camera")) {
    recording_desired_.store(config_.video_recording);
}

ZedOneRgbCamera::~ZedOneRgbCamera() {
    stop_thread_.store(true);
    data_cv_.notify_all();
    if (capture_thread_.joinable()) {
        const auto deadline = std::chrono::steady_clock::now() + kCaptureThreadJoinTimeout;
        while (!capture_thread_done_.load() && std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        if (!capture_thread_done_.load()) {
            spdlog::critical(
                "[ZedOneRgbCamera] Capture thread stuck in grab() for {} s at "
                "shutdown; exiting",
                kCaptureThreadJoinTimeout.count());
            spdlog::default_logger()->flush();
            std::_Exit(EXIT_FAILURE);
        }
        capture_thread_.join();
    }
    encoder_.stop();
    device_->close();
}

bool ZedOneRgbCamera::open_camera() {
    sl::InitParametersOne params;
    params.camera_resolution = get_zed_resolution(config_.camera_resolution);
    params.camera_fps = config_.camera_fps;
    params.sdk_verbose = 0;

    std::future<sl::ERROR_CODE> pending_open =
        std::async(std::launch::async, [this, params]() { return device_->open(params); });
    while (pending_open.wait_for(std::chrono::milliseconds(50)) != std::future_status::ready) {
        if (cancel_open_.load()) {
            pending_open.wait();
            pending_open.get();
            device_->close();
            return false;
        }
    }
    const sl::ERROR_CODE result = pending_open.get();
    if (result != sl::ERROR_CODE::SUCCESS) {
        spdlog::error("[ZedOneRgbCamera] Failed to open ZED X One: {}",
                      sl::toString(result).c_str());
        return false;
    }
    return true;
}

void ZedOneRgbCamera::read_calibration() {
    const sl::CameraOneInformation information = device_->information();
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
    info.distortion = cv::Mat::zeros(1, 5, CV_64F);
    latest_data_.camera_info = info;

    calibration_.calibration_id =
        zed_x_one_calibration_id(information.serial_number, width_, height_);
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
    if (!encoder_.start(
            encoder_options,
            [this](const std::byte *data, size_t len, uint64_t log_time_ns, bool) {
                foxglove::schemas::CompressedVideo message;
                message.timestamp =
                    foxglove_adapters::to_timestamp(static_cast<double>(log_time_ns) * 1e-9);
                message.frame_id = "camera";
                message.format = "h264";
                message.data.assign(data, data + len);
                video_channel_->log_message(message, log_time_ns);
            },
            encoder_factory_)) {
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
    const CameraSettingLimits limits = read_limits();
    const CameraSettings opened_settings = read_settings(default_manual_settings(limits));
    settings_state_ = CameraSettingsState{opened_settings, limits};
    latest_data_.pose_tracking_ready = true;

    if (mcap_recorder_) {
        mcap_recorder_->write_metadata(kCameraCalibrationMetadataKey,
                                       camera_calibration_to_toml(calibration_));
    }
    if (mcap_recorder_ && recording_desired_.load() && !start_encoder()) {
        throw std::runtime_error(
            "[ZedOneRgbCamera] video_recording is on but no H.264 encoder opened (tried " +
            VideoEncoder::candidate_names() +
            "). Fix the encoder or set [rgbd_camera] video_recording = false.");
    }

    is_initialized_.store(true);
    stop_thread_.store(false);
    capture_thread_ = std::thread(&ZedOneRgbCamera::capture_thread_loop, this, opened_settings);
    return true;
}

bool ZedOneRgbCamera::capture_frame() {
    const sl::ERROR_CODE grab_result = device_->grab();
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
    const uint64_t capture_time_ns = device_->image_timestamp_ns();
    const std::optional<cv::Mat> retrieved = device_->retrieve_bgra();
    if (!retrieved) {
        return false;
    }
    const cv::Mat &bgra = *retrieved;
    cv::Mat bgr;
    bgr.allocator = PinnedMatAllocator::instance();
    cv::cvtColor(bgra, bgr, cv::COLOR_BGRA2BGR);

    const int64_t video_frame_index = encoder_.submit(bgra, mcap_log_time_ns());

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_data_.rgb.image = require_rgb_layout(bgr);
        latest_data_.rgb.header.stamp = static_cast<double>(capture_time_ns) * 1e-9;
        latest_data_.rgb.header.frame_id = FrameId::CAMERA;
        latest_data_.camera_info.header = latest_data_.rgb.header;
        latest_data_.depth.image = cv::Mat();
        latest_data_.pose_tracking_ready = true;
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

void ZedOneRgbCamera::capture_thread_loop(const CameraSettings opened_settings) {
    while (!stop_thread_.load()) {
        service_settings(opened_settings);
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
        diagnostics_logger_->log(DiagnosticLevel::WARN, "video_encoder",
                                 {{"dropped_frames", std::to_string(drops)}});
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
    if (!mcap_recorder_ || !start_encoder()) {
        spdlog::error("[ZedOneRgbCamera] Recording toggled on but no H.264 encoder opened");
        return false;
    }
    return true;
}

bool ZedOneRgbCamera::is_recording_enabled() const { return encoder_.running(); }
void ZedOneRgbCamera::request_camera_settings(std::optional<CameraSettings> settings) {
    std::lock_guard<std::mutex> lock(settings_mutex_);
    if (settings) {
        pending_settings_ = SettingsRequest{std::move(*settings)};
    } else {
        pending_settings_ = SettingsRequest{RestoreOpenedSettings{}};
    }
}

std::optional<CameraSettingsState> ZedOneRgbCamera::camera_settings() const {
    std::lock_guard<std::mutex> lock(settings_mutex_);
    return settings_state_;
}

CameraSettings ZedOneRgbCamera::read_settings(const CameraSettings &known) {
    auto get = [this](sl::VIDEO_SETTINGS setting, int fallback) {
        int value = fallback;
        if (!device_->is_setting_supported(setting) ||
            device_->get_setting(setting, value) != sl::ERROR_CODE::SUCCESS) {
            return fallback;
        }
        return value;
    };
    auto get_or = [&](sl::VIDEO_SETTINGS setting, int when_unknown) {
        const int value = get(setting, -1);
        return value < 0 ? when_unknown : value;
    };
    CameraSettings s;
    s.auto_exposure = get(sl::VIDEO_SETTINGS::AEC_AGC, 1) != 0;
    s.exposure_us = get_or(sl::VIDEO_SETTINGS::EXPOSURE_TIME, known.exposure_us);
    s.analog_gain_mdb = get_or(sl::VIDEO_SETTINGS::ANALOG_GAIN, known.analog_gain_mdb);
    s.digital_gain = get_or(sl::VIDEO_SETTINGS::DIGITAL_GAIN, known.digital_gain);
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
        r.supported = device_->is_setting_supported(setting) &&
                      device_->get_range(setting, r.min, r.max) == sl::ERROR_CODE::SUCCESS;
        return r;
    };
    CameraSettingLimits limits;
    limits.exposure_us = exposure_range_capped_at_frame_period(
        range(sl::VIDEO_SETTINGS::EXPOSURE_TIME), config_.camera_fps);
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
    auto set = [this](sl::VIDEO_SETTINGS setting, int value, const CameraSettingRange &range,
                      bool nudge = true) {
        if (!range.supported) return;
        const int clamped = std::clamp(value, range.min, range.max);
        if (nudge && range.max > range.min) {
            device_->set_setting(setting, clamped == range.max ? clamped - 1 : clamped + 1);
        }
        const sl::ERROR_CODE result = device_->set_setting(setting, clamped);
        if (result != sl::ERROR_CODE::SUCCESS) {
            spdlog::warn("[ZedOneRgbCamera] setCameraSettings({}, {}) failed: {}",
                         sl::toString(setting).c_str(), clamped, sl::toString(result).c_str());
        }
    };
    const CameraSettingRange on_off{true, 0, 1};
    if (settings.auto_exposure) {
        set(sl::VIDEO_SETTINGS::AEC_AGC, 1, on_off, false);
    } else {
        set(sl::VIDEO_SETTINGS::AEC_AGC, 0, on_off, false);
        set(sl::VIDEO_SETTINGS::EXPOSURE_TIME, settings.exposure_us, limits.exposure_us);
        set(sl::VIDEO_SETTINGS::ANALOG_GAIN, settings.analog_gain_mdb, limits.analog_gain_mdb);
        set(sl::VIDEO_SETTINGS::DIGITAL_GAIN, settings.digital_gain, limits.digital_gain);
    }
    if (settings.auto_white_balance) {
        set(sl::VIDEO_SETTINGS::WHITEBALANCE_AUTO, 1, on_off, false);
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

void ZedOneRgbCamera::service_settings(const CameraSettings &opened_settings) {
    std::optional<SettingsRequest> request;
    {
        std::lock_guard<std::mutex> lock(settings_mutex_);
        request.swap(pending_settings_);
    }
    CameraSettings known = settings_state_->settings;
    if (request) {
        const CameraSettings *requested = std::get_if<CameraSettings>(&*request);
        const CameraSettings target = requested != nullptr ? *requested : opened_settings;
        apply_settings(target);
        const CameraSettingLimits &limits = settings_state_->limits;
        auto clamp = [](int value, const CameraSettingRange &range) {
            return range.supported ? std::clamp(value, range.min, range.max) : value;
        };
        known.exposure_us = clamp(target.exposure_us, limits.exposure_us);
        known.analog_gain_mdb = clamp(target.analog_gain_mdb, limits.analog_gain_mdb);
        known.digital_gain = clamp(target.digital_gain, limits.digital_gain);
    }
    constexpr int kSettingsReadbackPeriodFrames = 30;
    if (!request && ++frames_since_settings_read_ < kSettingsReadbackPeriodFrames) return;
    frames_since_settings_read_ = 0;
    const CameraSettings current = read_settings(known);
    std::lock_guard<std::mutex> lock(settings_mutex_);
    settings_state_->settings = current;
}

}  // namespace auto_battlebot
