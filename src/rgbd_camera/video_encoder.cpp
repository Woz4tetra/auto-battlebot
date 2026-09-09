#include "rgbd_camera/video_encoder.hpp"

#include <spdlog/spdlog.h>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}

#include <utility>

namespace auto_battlebot {
namespace {
// In preference order: the desktop's NVENC, the Jetson's V4L2 M2M block, then software.
constexpr const char *kCodecNames[] = {"h264_nvenc", "h264_nvv4l2m2m", "libx264"};
}  // namespace

VideoEncoder::~VideoEncoder() { stop(); }

bool VideoEncoder::start(const VideoEncoderOptions &options, PacketSink sink) {
    if (running_.load()) {
        return true;
    }
    options_ = options;
    sink_ = std::move(sink);

    const AVCodec *codec = nullptr;
    for (const char *name : kCodecNames) {
        codec = avcodec_find_encoder_by_name(name);
        if (codec != nullptr) {
            codec_name_ = name;
            break;
        }
    }
    if (codec == nullptr) {
        spdlog::error("[VideoEncoder] No H.264 encoder available (tried nvenc, nvv4l2m2m, x264)");
        return false;
    }

    context_ = avcodec_alloc_context3(codec);
    if (context_ == nullptr) {
        spdlog::error("[VideoEncoder] Failed to allocate a codec context");
        return false;
    }
    context_->width = options_.width;
    context_->height = options_.height;
    context_->time_base = AVRational{1, options_.fps};
    context_->framerate = AVRational{options_.fps, 1};
    context_->pix_fmt = AV_PIX_FMT_YUV420P;
    context_->bit_rate = options_.bitrate;
    context_->gop_size = options_.keyframe_interval;
    // B-frames reorder PTS. Everything downstream assumes the nth /camera/video message is the nth
    // captured frame: start_frame on playback, reading log_time as the capture instant, and
    // export_camera_transforms.py matching dataset images by position in the stream.
    context_->max_b_frames = 0;
    if (codec_name_ == "libx264") {
        av_opt_set(context_->priv_data, "preset", "veryfast", 0);
        av_opt_set(context_->priv_data, "tune", "zerolatency", 0);
    }
    // No AV_CODEC_FLAG_GLOBAL_HEADER: without it both x264 and nvenc repeat SPS/PPS in band ahead
    // of every IDR, which is what CompressedVideo requires and what makes a mid-file seek decode.

    if (avcodec_open2(context_, codec, nullptr) < 0) {
        spdlog::error("[VideoEncoder] Failed to open {}", codec_name_);
        close_codec();
        return false;
    }

    scaled_ = av_frame_alloc();
    packet_ = av_packet_alloc();
    if (scaled_ == nullptr || packet_ == nullptr) {
        close_codec();
        return false;
    }
    scaled_->format = context_->pix_fmt;
    scaled_->width = context_->width;
    scaled_->height = context_->height;
    if (av_frame_get_buffer(scaled_, 0) < 0) {
        spdlog::error("[VideoEncoder] Failed to allocate the scaled frame buffer");
        close_codec();
        return false;
    }

    const AVPixelFormat source_format =
        options_.input_is_uyvy ? AV_PIX_FMT_UYVY422 : AV_PIX_FMT_BGR24;
    scaler_ =
        sws_getContext(options_.width, options_.height, source_format, options_.width,
                       options_.height, context_->pix_fmt, SWS_BILINEAR, nullptr, nullptr, nullptr);
    if (scaler_ == nullptr) {
        spdlog::error("[VideoEncoder] Failed to build the colour conversion context");
        close_codec();
        return false;
    }

    stopping_.store(false);
    running_.store(true);
    thread_ = std::thread(&VideoEncoder::encode_loop, this);
    spdlog::info("[VideoEncoder] {} at {}x{} {} fps, {} kbps, IDR every {} frames", codec_name_,
                 options_.width, options_.height, options_.fps, options_.bitrate / 1000,
                 options_.keyframe_interval);
    return true;
}

void VideoEncoder::stop() {
    if (!running_.load()) {
        close_codec();
        return;
    }
    stopping_.store(true);
    queue_cv_.notify_all();
    if (thread_.joinable()) {
        thread_.join();
    }
    running_.store(false);
    close_codec();
}

void VideoEncoder::close_codec() {
    if (scaler_ != nullptr) {
        sws_freeContext(scaler_);
        scaler_ = nullptr;
    }
    if (packet_ != nullptr) {
        av_packet_free(&packet_);
    }
    if (scaled_ != nullptr) {
        av_frame_free(&scaled_);
    }
    if (context_ != nullptr) {
        avcodec_free_context(&context_);
    }
    log_time_by_pts_.clear();
    next_pts_ = 0;
}

void VideoEncoder::submit(const cv::Mat &frame, uint64_t log_time_ns) {
    if (!running_.load()) {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.size() >= options_.queue_depth) {
            // Counted, not silent. Frame loss discovered months later against ground truth is
            // exactly the failure this counter exists to prevent.
            dropped_frames_.fetch_add(1);
            return;
        }
        queue_.push_back(QueuedFrame{frame.clone(), log_time_ns});
    }
    queue_cv_.notify_one();
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
        encode_one(frame);
    }
    // Flush, so the frames the encoder was still holding reach the recording.
    if (context_ != nullptr) {
        avcodec_send_frame(context_, nullptr);
        drain();
    }
}

bool VideoEncoder::encode_one(const QueuedFrame &frame) {
    if (context_ == nullptr || frame.image.empty()) {
        return false;
    }
    if (av_frame_make_writable(scaled_) < 0) {
        return false;
    }
    const uint8_t *source_data[4] = {frame.image.data, nullptr, nullptr, nullptr};
    const int source_stride[4] = {static_cast<int>(frame.image.step), 0, 0, 0};
    sws_scale(scaler_, source_data, source_stride, 0, options_.height, scaled_->data,
              scaled_->linesize);

    const int64_t pts = next_pts_++;
    scaled_->pts = pts;
    log_time_by_pts_[pts] = frame.log_time_ns;
    if (avcodec_send_frame(context_, scaled_) < 0) {
        log_time_by_pts_.erase(pts);
        return false;
    }
    drain();
    return true;
}

void VideoEncoder::drain() {
    while (true) {
        const int status = avcodec_receive_packet(context_, packet_);
        if (status == AVERROR(EAGAIN) || status == AVERROR_EOF) {
            return;
        }
        if (status < 0) {
            spdlog::warn("[VideoEncoder] Encoder returned an error while draining");
            return;
        }
        uint64_t log_time_ns = 0;
        if (auto it = log_time_by_pts_.find(packet_->pts); it != log_time_by_pts_.end()) {
            log_time_ns = it->second;
            log_time_by_pts_.erase(it);
        }
        if (sink_ && log_time_ns != 0) {
            sink_(reinterpret_cast<const std::byte *>(packet_->data),
                  static_cast<size_t>(packet_->size), log_time_ns,
                  (packet_->flags & AV_PKT_FLAG_KEY) != 0);
            encoded_frames_.fetch_add(1);
        }
        av_packet_unref(packet_);
    }
}
}  // namespace auto_battlebot
