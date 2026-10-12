#include <spdlog/spdlog.h>

#include "rgbd_camera/h264_backend.hpp"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}

#include <utility>

namespace auto_battlebot {
namespace {
AVPixelFormat source_pixel_format(VideoInputFormat format) {
    switch (format) {
        case VideoInputFormat::BGR:
            return AV_PIX_FMT_BGR24;
        case VideoInputFormat::BGRA:
            return AV_PIX_FMT_BGRA;
        case VideoInputFormat::UYVY:
            return AV_PIX_FMT_UYVY422;
    }
    return AV_PIX_FMT_NONE;
}

class FfmpegH264Backend : public H264Backend {
   public:
    FfmpegH264Backend(std::string name, PacketSink sink)
        : name_(std::move(name)), sink_(std::move(sink)) {}
    ~FfmpegH264Backend() override { close(); }

    bool open(const AVCodec *codec, const VideoEncoderOptions &options) {
        options_ = options;
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
        context_->max_b_frames = 0;
        if (name_ == "libx264") {
            av_opt_set(context_->priv_data, "preset", "veryfast", 0);
            av_opt_set(context_->priv_data, "tune", "zerolatency", 0);
        }
        // A forced keyframe is an IDR, not just an I-frame, so a new viewer can decode from it.
        // Encoders without the option ignore it.
        av_opt_set(context_->priv_data, "forced-idr", "1", 0);
        // No AV_CODEC_FLAG_GLOBAL_HEADER: without it both x264 and nvenc repeat SPS/PPS in band
        // ahead of every IDR, which is what CompressedVideo requires and what makes a mid-file seek
        // decode.
        if (avcodec_open2(context_, codec, nullptr) < 0) {
            return false;
        }

        scaled_ = av_frame_alloc();
        packet_ = av_packet_alloc();
        if (scaled_ == nullptr || packet_ == nullptr) {
            return false;
        }
        scaled_->format = context_->pix_fmt;
        scaled_->width = context_->width;
        scaled_->height = context_->height;
        if (av_frame_get_buffer(scaled_, 0) < 0) {
            spdlog::error("[VideoEncoder] Failed to allocate the scaled frame buffer");
            return false;
        }
        scaler_ = sws_getContext(options_.width, options_.height,
                                 source_pixel_format(options_.input_format), options_.width,
                                 options_.height, context_->pix_fmt, SWS_BILINEAR, nullptr, nullptr,
                                 nullptr);
        if (scaler_ == nullptr) {
            spdlog::error("[VideoEncoder] Failed to build the colour conversion context");
            return false;
        }
        return true;
    }

    bool encode(cv::Mat image, int64_t pts, bool force_keyframe) override {
        if (context_ == nullptr || image.empty() || av_frame_make_writable(scaled_) < 0) {
            return false;
        }
        const uint8_t *source_data[4] = {image.data, nullptr, nullptr, nullptr};
        const int source_stride[4] = {static_cast<int>(image.step), 0, 0, 0};
        sws_scale(scaler_, source_data, source_stride, 0, options_.height, scaled_->data,
                  scaled_->linesize);
        scaled_->pts = pts;
        scaled_->pict_type = force_keyframe ? AV_PICTURE_TYPE_I : AV_PICTURE_TYPE_NONE;
        if (avcodec_send_frame(context_, scaled_) < 0) {
            return false;
        }
        drain();
        return true;
    }

    void finish() override {
        if (context_ != nullptr) {
            avcodec_send_frame(context_, nullptr);
            drain();
        }
        close();
    }

    const std::string &name() const override { return name_; }

   private:
    void drain() {
        while (true) {
            const int status = avcodec_receive_packet(context_, packet_);
            if (status == AVERROR(EAGAIN) || status == AVERROR_EOF) {
                return;
            }
            if (status < 0) {
                spdlog::warn("[VideoEncoder] Encoder returned an error while draining");
                return;
            }
            sink_(reinterpret_cast<const std::byte *>(packet_->data),
                  static_cast<size_t>(packet_->size), packet_->pts,
                  (packet_->flags & AV_PKT_FLAG_KEY) != 0);
            av_packet_unref(packet_);
        }
    }

    void close() {
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
    }

    std::string name_;
    PacketSink sink_;
    VideoEncoderOptions options_;
    AVCodecContext *context_ = nullptr;
    AVFrame *scaled_ = nullptr;
    AVPacket *packet_ = nullptr;
    SwsContext *scaler_ = nullptr;
};
}  // namespace

bool ffmpeg_h264_registered(const std::string &codec) {
    return avcodec_find_encoder_by_name(codec.c_str()) != nullptr;
}

std::unique_ptr<H264Backend> open_ffmpeg_h264(const std::string &codec,
                                              const VideoEncoderOptions &options,
                                              H264Backend::PacketSink sink) {
    const AVCodec *found = avcodec_find_encoder_by_name(codec.c_str());
    if (found == nullptr) {
        return nullptr;
    }
    auto backend = std::make_unique<FfmpegH264Backend>(codec, std::move(sink));
    if (!backend->open(found, options)) {
        spdlog::warn("[VideoEncoder] FFmpeg registers {} but it failed to open", codec);
        return nullptr;
    }
    return backend;
}
}  // namespace auto_battlebot
