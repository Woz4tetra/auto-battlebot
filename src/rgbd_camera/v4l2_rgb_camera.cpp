#include "rgbd_camera/v4l2_rgb_camera.hpp"

#include <fcntl.h>
#include <linux/videodev2.h>
#include <spdlog/spdlog.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <foxglove/schemas.hpp>
#include <utility>

namespace auto_battlebot {
namespace {
constexpr auto kGrabErrorWindow = std::chrono::seconds(10);
constexpr double kGrabErrorExitThreshold = 0.70;
constexpr auto kGetWaitTimeout = std::chrono::milliseconds(100);
constexpr const char *kVideoTopic = "/camera/video";

std::pair<int, int> resolution_size(Resolution resolution) {
    switch (resolution) {
        case Resolution::RES_3856x2180:
            return {3856, 2180};
        case Resolution::RES_3800x1800:
            return {3800, 1800};
        case Resolution::RES_2208x1242:
            return {2208, 1242};
        case Resolution::RES_1920x1536:
            return {1920, 1536};
        case Resolution::RES_1920x1080:
            return {1920, 1080};
        case Resolution::RES_1920x1200:
            return {1920, 1200};
        case Resolution::RES_1280x720:
            return {1280, 720};
        case Resolution::RES_960x600:
            return {960, 600};
        case Resolution::RES_672x376:
            return {672, 376};
    }
    return {1920, 1200};
}

/** ioctl retried through EINTR, which V4L2 hands back routinely. */
int xioctl(int fd, unsigned long request, void *argument) {
    int result = 0;
    do {
        result = ioctl(fd, request, argument);
    } while (result == -1 && errno == EINTR);
    return result;
}

void set_control(int fd, uint32_t id, int32_t value, const char *name) {
    v4l2_control control{};
    control.id = id;
    control.value = value;
    if (xioctl(fd, VIDIOC_S_CTRL, &control) == -1) {
        spdlog::warn("[V4l2RgbCamera] Could not set {} to {}: {}", name, value,
                     std::strerror(errno));
    }
}
}  // namespace

V4l2RgbCamera::V4l2RgbCamera(const V4l2RgbCameraConfiguration &config,
                             std::shared_ptr<McapRecorder> mcap_recorder)
    : config_(config),
      grab_health_(kGrabErrorWindow, kGrabErrorExitThreshold),
      mcap_recorder_(std::move(mcap_recorder)),
      diagnostics_logger_(DiagnosticsLogger::get_logger("v4l2_rgb_camera")) {
    const auto [width, height] = resolution_size(config_.camera_resolution);
    width_ = width;
    height_ = height;
    recording_desired_.store(config_.video_recording);
}

V4l2RgbCamera::~V4l2RgbCamera() {
    stop_thread_.store(true);
    data_cv_.notify_all();
    if (capture_thread_.joinable()) {
        capture_thread_.join();
    }
    encoder_.stop();
    stop_streaming();
    unmap_buffers();
    close_device();
}

bool V4l2RgbCamera::open_device() {
    fd_ = ::open(config_.device.c_str(), O_RDWR | O_NONBLOCK, 0);
    if (fd_ < 0) {
        spdlog::error("[V4l2RgbCamera] Cannot open {}: {}", config_.device, std::strerror(errno));
        return false;
    }
    v4l2_capability capability{};
    if (xioctl(fd_, VIDIOC_QUERYCAP, &capability) == -1) {
        spdlog::error("[V4l2RgbCamera] {} is not a V4L2 device", config_.device);
        return false;
    }
    if ((capability.capabilities & V4L2_CAP_VIDEO_CAPTURE) == 0) {
        spdlog::error("[V4l2RgbCamera] {} does not support video capture", config_.device);
        return false;
    }
    if ((capability.capabilities & V4L2_CAP_STREAMING) == 0) {
        spdlog::error("[V4l2RgbCamera] {} does not support streaming I/O", config_.device);
        return false;
    }
    return true;
}

bool V4l2RgbCamera::negotiate_format() {
    v4l2_format format{};
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = static_cast<uint32_t>(width_);
    format.fmt.pix.height = static_cast<uint32_t>(height_);
    // The sensor emits UYVY and we encode H.264, so both are hard-coded. There is one correct
    // behaviour and one consumer; a second capture format is when this becomes an enum.
    format.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
    format.fmt.pix.field = V4L2_FIELD_NONE;
    if (xioctl(fd_, VIDIOC_S_FMT, &format) == -1) {
        spdlog::error("[V4l2RgbCamera] Cannot set format: {}", std::strerror(errno));
        return false;
    }
    if (format.fmt.pix.pixelformat != V4L2_PIX_FMT_UYVY) {
        spdlog::error("[V4l2RgbCamera] Driver refused UYVY and gave another format instead");
        return false;
    }
    if (static_cast<int>(format.fmt.pix.width) != width_ ||
        static_cast<int>(format.fmt.pix.height) != height_) {
        spdlog::warn("[V4l2RgbCamera] Driver adjusted the capture size to {}x{}",
                     format.fmt.pix.width, format.fmt.pix.height);
        width_ = static_cast<int>(format.fmt.pix.width);
        height_ = static_cast<int>(format.fmt.pix.height);
    }

    v4l2_streamparm parameters{};
    parameters.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    parameters.parm.capture.timeperframe.numerator = 1;
    parameters.parm.capture.timeperframe.denominator = static_cast<uint32_t>(config_.camera_fps);
    if (xioctl(fd_, VIDIOC_S_PARM, &parameters) == -1) {
        spdlog::warn("[V4l2RgbCamera] Could not set {} fps: {}", config_.camera_fps,
                     std::strerror(errno));
    }
    return true;
}

bool V4l2RgbCamera::apply_controls() {
    // Auto-exposure hunting between a lit arena and a dark robot is a plausible source of
    // frame-to-frame detector instability, so pinning these matters. Zero keeps the driver default.
    if (config_.exposure_us > 0) {
        set_control(fd_, V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL, "exposure_auto");
        // V4L2 exposure_absolute is in 100 us units.
        set_control(fd_, V4L2_CID_EXPOSURE_ABSOLUTE, config_.exposure_us / 100,
                    "exposure_absolute");
    }
    if (config_.gain > 0) {
        set_control(fd_, V4L2_CID_GAIN, config_.gain, "gain");
    }
    if (config_.white_balance_temperature > 0) {
        set_control(fd_, V4L2_CID_AUTO_WHITE_BALANCE, 0, "auto_white_balance");
        set_control(fd_, V4L2_CID_WHITE_BALANCE_TEMPERATURE, config_.white_balance_temperature,
                    "white_balance_temperature");
    }
    return true;
}

bool V4l2RgbCamera::map_buffers() {
    v4l2_requestbuffers request{};
    request.count = static_cast<uint32_t>(config_.buffer_count);
    request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    request.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd_, VIDIOC_REQBUFS, &request) == -1) {
        spdlog::error("[V4l2RgbCamera] Cannot request buffers: {}", std::strerror(errno));
        return false;
    }
    if (request.count < 2) {
        spdlog::error("[V4l2RgbCamera] Driver gave only {} buffers", request.count);
        return false;
    }

    buffers_.resize(request.count);
    for (uint32_t i = 0; i < request.count; ++i) {
        v4l2_buffer buffer{};
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;
        buffer.index = i;
        if (xioctl(fd_, VIDIOC_QUERYBUF, &buffer) == -1) {
            spdlog::error("[V4l2RgbCamera] QUERYBUF failed: {}", std::strerror(errno));
            return false;
        }
        buffers_[i].length = buffer.length;
        buffers_[i].start = mmap(nullptr, buffer.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_,
                                 static_cast<off_t>(buffer.m.offset));
        if (buffers_[i].start == MAP_FAILED) {
            spdlog::error("[V4l2RgbCamera] mmap failed: {}", std::strerror(errno));
            buffers_[i].start = nullptr;
            return false;
        }
    }
    return true;
}

bool V4l2RgbCamera::start_streaming() {
    for (size_t i = 0; i < buffers_.size(); ++i) {
        v4l2_buffer buffer{};
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;
        buffer.index = static_cast<uint32_t>(i);
        if (xioctl(fd_, VIDIOC_QBUF, &buffer) == -1) {
            spdlog::error("[V4l2RgbCamera] QBUF failed: {}", std::strerror(errno));
            return false;
        }
    }
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd_, VIDIOC_STREAMON, &type) == -1) {
        spdlog::error("[V4l2RgbCamera] STREAMON failed: {}", std::strerror(errno));
        return false;
    }
    return true;
}

void V4l2RgbCamera::stop_streaming() {
    if (fd_ < 0) {
        return;
    }
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    xioctl(fd_, VIDIOC_STREAMOFF, &type);
}

void V4l2RgbCamera::unmap_buffers() {
    for (auto &buffer : buffers_) {
        if (buffer.start != nullptr) {
            munmap(buffer.start, buffer.length);
            buffer.start = nullptr;
        }
    }
    buffers_.clear();
}

void V4l2RgbCamera::close_device() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool V4l2RgbCamera::initialize() {
    if (is_initialized_.load()) {
        return true;
    }
    if (config_.calibration_file.empty()) {
        spdlog::error(
            "[V4l2RgbCamera] calibration_file is required: a 104 degree M12 lens is not a pinhole "
            "and every consumer downstream assumes one");
        return false;
    }
    calibration_ = load_camera_calibration(config_.calibration_file);

    if (!open_device() || !negotiate_format() || !apply_controls() || !map_buffers() ||
        !start_streaming()) {
        close_device();
        return false;
    }

    rectifier_.build(calibration_, cv::Size(width_, height_));
    latest_data_.camera_info = rectifier_.camera_info();
    latest_data_.tracking_ok = true;

    if (mcap_recorder_) {
        // So a recording can be re-rectified later against a revised calibration.
        mcap_recorder_->write_metadata("calibration_id", calibration_.calibration_id);
    }
    if (mcap_recorder_ && recording_desired_.load()) {
        video_channel_ = std::make_unique<OutputChannel>(
            kVideoTopic, "protobuf",
            VizSchema::from_sdk(foxglove::schemas::CompressedVideo::schema()), false, nullptr,
            mcap_recorder_);
        VideoEncoderOptions encoder_options;
        encoder_options.width = width_;
        encoder_options.height = height_;
        encoder_options.fps = config_.camera_fps;
        encoder_options.bitrate = config_.video_bitrate_kbps * 1000;
        encoder_options.input_is_uyvy = true;
        if (!encoder_.start(encoder_options, [this](const std::byte *data, size_t len,
                                                    uint64_t log_time_ns, bool /*keyframe*/) {
                foxglove::schemas::CompressedVideo message;
                message.frame_id = "camera";
                message.format = "h264";
                message.data.assign(data, data + len);
                video_channel_->log_message(message, log_time_ns);
            })) {
            spdlog::error("[V4l2RgbCamera] Video recording requested but no encoder started");
            video_channel_.reset();
        }
    }

    is_initialized_.store(true);
    stop_thread_.store(false);
    capture_thread_ = std::thread(&V4l2RgbCamera::capture_thread_loop, this);
    spdlog::info("[V4l2RgbCamera] {} at {}x{} {} fps, calibration '{}'", config_.device, width_,
                 height_, config_.camera_fps, calibration_.calibration_id);
    return true;
}

int V4l2RgbCamera::dequeue_newest(uint64_t &capture_time_ns) {
    int newest = -1;
    while (true) {
        v4l2_buffer buffer{};
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;
        if (xioctl(fd_, VIDIOC_DQBUF, &buffer) == -1) {
            if (errno == EAGAIN) {
                break;
            }
            return -1;
        }
        if (newest >= 0) {
            // An older frame the pipeline never reached. Give the buffer straight back.
            v4l2_buffer stale{};
            stale.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            stale.memory = V4L2_MEMORY_MMAP;
            stale.index = static_cast<uint32_t>(newest);
            xioctl(fd_, VIDIOC_QBUF, &stale);
        }
        newest = static_cast<int>(buffer.index);
        // V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC: the kernel's capture instant on CLOCK_MONOTONIC.
        // Not the wall clock, because the replay path depends on the frame stamp being the moment
        // the sensor was read.
        capture_time_ns = static_cast<uint64_t>(buffer.timestamp.tv_sec) * 1000000000ULL +
                          static_cast<uint64_t>(buffer.timestamp.tv_usec) * 1000ULL;
    }
    return newest;
}

bool V4l2RgbCamera::capture_frame() {
    fd_set read_set;
    FD_ZERO(&read_set);
    FD_SET(fd_, &read_set);
    timeval timeout{};
    timeout.tv_sec = 1;
    const int ready = select(fd_ + 1, &read_set, nullptr, nullptr, &timeout);
    if (ready <= 0) {
        grab_health_.record(true, std::chrono::steady_clock::now());
        if (grab_health_.should_shutdown() && !should_close_.load()) {
            should_close_.store(true);
            data_cv_.notify_all();
            spdlog::error(
                "[V4l2RgbCamera] Capture error ratio {:.1f}% over the last 10s exceeded {:.0f}%. "
                "Requesting application shutdown.",
                grab_health_.error_ratio() * 100.0, kGrabErrorExitThreshold * 100.0);
        }
        return false;
    }

    uint64_t capture_time_ns = 0;
    const int index = dequeue_newest(capture_time_ns);
    if (index < 0) {
        grab_health_.record(true, std::chrono::steady_clock::now());
        return false;
    }
    grab_health_.record(false, std::chrono::steady_clock::now());

    const cv::Mat uyvy(height_, width_, CV_8UC2, buffers_[static_cast<size_t>(index)].start);

    // The recording is pre-rectification on purpose: a calibration can be revised later and
    // re-applied to footage already shot.
    if (encoder_.running()) {
        encoder_.submit(uyvy, wall_time_ns());
    }

    cv::Mat bgr;
    cv::cvtColor(uyvy, bgr, cv::COLOR_YUV2BGR_UYVY);
    cv::Mat rectified;
    rectifier_.apply(bgr, rectified);

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_data_.rgb.image = rectified;
        latest_data_.rgb.header.stamp = static_cast<double>(capture_time_ns) * 1e-9;
        latest_data_.rgb.header.frame_id = FrameId::CAMERA;
        latest_data_.depth.image = cv::Mat();
        latest_data_.tracking_ok = true;
        // A clamped camera does not move, so there is no visual odometry and no camera-world
        // rebase to do. The field filter's rebase becomes a no-op, which removes the class of bug
        // that scripts/fix_field_mask_frames.py exists to repair offline.
        latest_data_.tf_visodom_from_camera.header.stamp = latest_data_.rgb.header.stamp;
        latest_data_.tf_visodom_from_camera.header.frame_id = FrameId::VISUAL_ODOMETRY;
        latest_data_.tf_visodom_from_camera.child_frame_id = FrameId::CAMERA;
        latest_data_.tf_visodom_from_camera.transform.tf = Eigen::Matrix4d::Identity();
        latest_data_.frame_identity.image_stamp_ns = capture_time_ns;
        latest_data_.frame_identity.video_frame_index =
            encoder_.running() ? ++video_frame_index_ : -1;
        frame_counter_++;
    }

    v4l2_buffer requeue{};
    requeue.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    requeue.memory = V4L2_MEMORY_MMAP;
    requeue.index = static_cast<uint32_t>(index);
    xioctl(fd_, VIDIOC_QBUF, &requeue);
    return true;
}

void V4l2RgbCamera::capture_thread_loop() {
    while (!stop_thread_.load()) {
        if (capture_frame()) {
            data_cv_.notify_all();
        } else if (should_close_.load()) {
            break;
        }
    }
}

bool V4l2RgbCamera::get(CameraData &data) {
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
        // Never silent. A recording quietly missing frames is discovered months later against
        // ground truth, if at all.
        spdlog::warn("[V4l2RgbCamera] Encoder has dropped {} frame(s); it is behind the capture",
                     drops);
        diagnostics_logger_->log(1, "video_encoder", {{"dropped_frames", std::to_string(drops)}});
        reported_drops_ = drops;
    }
    return true;
}

bool V4l2RgbCamera::set_recording_enabled(bool enabled) {
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

bool V4l2RgbCamera::is_recording_enabled() const { return encoder_.running(); }
}  // namespace auto_battlebot
