#include "rgbd_camera/video_encoder.hpp"

#include <spdlog/spdlog.h>

#include <utility>

#include "rgbd_camera/h264_backend.hpp"

namespace auto_battlebot {
namespace {
// In preference order. FFmpeg's names are tried before GStreamer's because the desktop's NVENC
// lives there; on the Jetsons the GStreamer element is the one that opens.
constexpr const char *kFfmpegHardware[] = {"h264_nvenc", "h264_nvv4l2m2m"};
// Software. On the Orin it cannot hold a recording's 60 fps and takes the cores perception needs,
// so there it is only for the preview (VideoEncoderOptions::software_preview).
constexpr const char *kFfmpegSoftware = "libx264";

bool software_allowed(const VideoEncoderOptions &options) {
#ifdef __aarch64__
    return options.software_preview;
#else
    (void)options;
    return true;
#endif
}
}  // namespace

VideoEncoder::VideoEncoder() = default;
VideoEncoder::~VideoEncoder() { stop(); }

std::string VideoEncoder::candidate_names() {
    std::string names;
    for (const char *name : kFfmpegHardware) {
        names += std::string(names.empty() ? "" : ", ") + "FFmpeg " + name;
    }
#ifdef AUTO_BATTLEBOT_HAVE_GSTREAMER
    names += ", GStreamer nvv4l2h264enc";
#endif
    names += std::string(", FFmpeg ") + kFfmpegSoftware;
#ifdef AUTO_BATTLEBOT_HAVE_GSTREAMER
    names += ", GStreamer x264enc";
#endif
#ifdef __aarch64__
    names += " (software: preview only)";
#endif
    return names;
}

bool VideoEncoder::start(const VideoEncoderOptions &options, PacketSink sink,
                         const BackendFactory &factory) {
    if (running_.load()) {
        return true;
    }
    options_ = options;
    sink_ = std::move(sink);
    next_pts_ = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.clear();
        next_submit_index_ = 0;
        keyframe_requested_.store(false);
        broken_.store(false);
    }
    {
        std::lock_guard<std::mutex> lock(pts_mutex_);
        log_time_by_pts_.clear();
    }

    const H264Backend::PacketSink deliver_packet = [this](const std::byte *data, size_t len,
                                                          int64_t pts, bool keyframe) {
        deliver(data, len, pts, keyframe);
    };
    if (factory) backend_ = factory(options_, deliver_packet);
    for (const char *name : kFfmpegHardware) {
        if (!backend_ && !factory) backend_ = open_ffmpeg_h264(name, options_, deliver_packet);
    }
#ifdef AUTO_BATTLEBOT_HAVE_GSTREAMER
    if (!backend_ && !factory) backend_ = open_gst_nvv4l2_h264(options_, deliver_packet);
#endif
    if (!backend_ && !factory && software_allowed(options_)) {
        backend_ = open_ffmpeg_h264(kFfmpegSoftware, options_, deliver_packet);
    }
#ifdef AUTO_BATTLEBOT_HAVE_GSTREAMER
    // NVIDIA's FFmpeg on JetPack 7 has no libx264; GStreamer's x264enc stands in.
    if (!backend_ && !factory && software_allowed(options_)) {
        backend_ = open_gst_x264(options_, deliver_packet);
    }
#endif
    if (!backend_) {
        spdlog::error("[VideoEncoder] No H.264 encoder opened at {}x{} {} fps (tried {})",
                      options_.width, options_.height, options_.fps, candidate_names());
        return false;
    }
    codec_name_ = backend_->name();

    stopping_.store(false);
    running_.store(true);
    thread_ = std::thread(&VideoEncoder::encode_loop, this);
    spdlog::info("[VideoEncoder] {} at {}x{} {} fps, {} kbps, IDR every {} frames", codec_name_,
                 options_.width, options_.height, options_.fps, options_.bitrate / 1000,
                 options_.keyframe_interval);
    return true;
}

bool VideoEncoder::hardware_encoder_available() {
    for (const char *name : kFfmpegHardware) {
        if (ffmpeg_h264_registered(name)) return true;
    }
#ifdef AUTO_BATTLEBOT_HAVE_GSTREAMER
    if (gst_nvv4l2_h264_installed()) return true;
#endif
    return false;
}

bool VideoEncoder::software_encoder_available() {
#ifdef AUTO_BATTLEBOT_HAVE_GSTREAMER
    if (gst_x264_installed()) return true;
#endif
    return ffmpeg_h264_registered(kFfmpegSoftware);
}

void VideoEncoder::stop() {
    if (running_.load()) {
        {
            // Under the lock, so submit() sees it before the encode thread can drain and exit.
            // Set outside it, a frame submitted in between sat in the queue and opened the next
            // recording as its first packet, two seconds stale, with every index after it shifted.
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_.store(true);
        }
        queue_cv_.notify_all();
        if (thread_.joinable()) {
            thread_.join();
        }
        running_.store(false);
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.clear();
    }
    backend_.reset();
}

int64_t VideoEncoder::submit(const cv::Mat &frame, uint64_t log_time_ns) {
    if (!running_.load()) {
        return -1;
    }
    int64_t index = -1;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (stopping_.load()) {
            return -1;
        }
        if (broken_.load()) {
            dropped_frames_.fetch_add(1);
            return -1;
        }
        if (queue_.size() >= options_.queue_depth) {
            // Counted, not silent. Frame loss discovered months later against ground truth is
            // exactly the failure this counter exists to prevent.
            dropped_frames_.fetch_add(1);
            return -1;
        }
        // The request rides with the frame: the encoder thread may still be working through
        // older frames when it arrives.
        queue_.push_back(
            QueuedFrame{frame.clone(), log_time_ns, keyframe_requested_.exchange(false)});
        index = next_submit_index_++;
    }
    queue_cv_.notify_one();
    return index;
}

void VideoEncoder::encode_loop() {
    while (true) {
        QueuedFrame frame;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            queue_cv_.wait(lock, [this] { return !queue_.empty() || stopping_.load(); });
            if (queue_.empty()) {
                break;
            }
            frame = std::move(queue_.front());
            queue_.pop_front();
        }
        const int64_t pts = next_pts_++;
        {
            std::lock_guard<std::mutex> lock(pts_mutex_);
            log_time_by_pts_[pts] = frame.log_time_ns;
        }
        if (!backend_->encode(std::move(frame.image), pts, frame.force_keyframe)) {
            {
                std::lock_guard<std::mutex> lock(pts_mutex_);
                log_time_by_pts_.erase(pts);
            }
            // This frame already has its index in frame_meta, and encoding the frames behind it
            // would put each one packet before its index. Take nothing more, so every index
            // handed out has its packet or points past the end of the file, and let the runner
            // restart the recording. Only a pipeline stalled for kStallTimeout gets here.
            size_t discarded = 0;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                broken_.store(true);
                discarded = queue_.size();
                queue_.clear();
            }
            dropped_frames_.fetch_add(1 + discarded);
            spdlog::error(
                "[VideoEncoder] {} refused frame {}; video stops here ({} queued frames "
                "discarded) until the recording restarts",
                codec_name_, pts, discarded);
        }
    }
    // Flush, so the frames the encoder was still holding reach the recording.
    backend_->finish();
}

void VideoEncoder::deliver(const std::byte *data, size_t len, int64_t pts, bool keyframe) {
    uint64_t log_time_ns = 0;
    {
        std::lock_guard<std::mutex> lock(pts_mutex_);
        if (auto it = log_time_by_pts_.find(pts); it != log_time_by_pts_.end()) {
            log_time_ns = it->second;
            log_time_by_pts_.erase(it);
        }
    }
    if (sink_ && log_time_ns != 0) {
        sink_(data, len, log_time_ns, keyframe);
        encoded_frames_.fetch_add(1);
    }
}
}  // namespace auto_battlebot
