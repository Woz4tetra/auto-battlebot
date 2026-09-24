#include "rgbd_camera/zed_one_rgb_camera.hpp"

#include <spdlog/spdlog.h>

#include <chrono>
#include <foxglove/schemas.hpp>
#include <utility>

#include "rgbd_camera/camera_calibration.hpp"
#include "rgbd_camera/zed_device.hpp"

namespace auto_battlebot {
namespace {
constexpr auto kGrabErrorWindow = std::chrono::seconds(10);
constexpr double kGrabErrorExitThreshold = 0.70;
constexpr auto kGetWaitTimeout = std::chrono::milliseconds(100);
constexpr const char *kVideoTopic = "/camera/video";
}  // namespace

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
    encoder_options.input_is_uyvy = false;
    if (!encoder_.start(encoder_options, [this](const std::byte *data, size_t len,
                                                uint64_t log_time_ns, bool /*keyframe*/) {
            foxglove::schemas::CompressedVideo message;
            message.frame_id = "camera";
            message.format = "h264";
            message.data.assign(data, data + len);
            video_channel_->log_message(message, log_time_ns);
        })) {
        spdlog::error("[ZedOneRgbCamera] Video recording requested but no encoder started");
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
    latest_data_.tracking_ok = true;

    if (mcap_recorder_) {
        // VideoPlaybackCamera reads this to replay the file; no config/cameras/ file needed.
        mcap_recorder_->write_metadata(kCameraCalibrationMetadataKey,
                                       camera_calibration_to_toml(calibration_));
    }
    if (mcap_recorder_ && recording_desired_.load()) {
        start_encoder();
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
    // A fresh Mat every frame: the pipeline holds the previous one by reference.
    cv::Mat bgr;
    cv::cvtColor(bgra, bgr, cv::COLOR_BGRA2BGR);

    if (encoder_.running()) {
        encoder_.submit(bgr, wall_time_ns());
    }

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_data_.rgb.image = bgr;
        latest_data_.rgb.header.stamp = static_cast<double>(capture_time_ns) * 1e-9;
        latest_data_.rgb.header.frame_id = FrameId::CAMERA;
        latest_data_.depth.image = cv::Mat();
        latest_data_.tracking_ok = true;
        // A clamped camera does not move: no visual odometry, no camera-world rebase.
        latest_data_.tf_visodom_from_camera.header.stamp = latest_data_.rgb.header.stamp;
        latest_data_.tf_visodom_from_camera.header.frame_id = FrameId::VISUAL_ODOMETRY;
        latest_data_.tf_visodom_from_camera.child_frame_id = FrameId::CAMERA;
        latest_data_.tf_visodom_from_camera.transform.tf = Eigen::Matrix4d::Identity();
        latest_data_.frame_identity.image_stamp_ns = capture_time_ns;
        latest_data_.frame_identity.video_frame_index =
            encoder_.running() ? ++video_frame_index_ : -1;
        frame_counter_++;
    }
    return true;
}

void ZedOneRgbCamera::capture_thread_loop() {
    while (!stop_thread_.load()) {
        if (capture_frame()) {
            data_cv_.notify_all();
        } else if (should_close_.load()) {
            break;
        }
    }
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
    return encoder_.running();
}

bool ZedOneRgbCamera::is_recording_enabled() const { return encoder_.running(); }
}  // namespace auto_battlebot
