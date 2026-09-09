#include "rgbd_camera/video_decoder.hpp"

#include <spdlog/spdlog.h>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
}

namespace auto_battlebot {
VideoDecoder::~VideoDecoder() { close(); }

bool VideoDecoder::open() {
    if (context_ != nullptr) {
        return true;
    }
    const AVCodec *codec = avcodec_find_decoder(AV_CODEC_ID_H264);
    if (codec == nullptr) {
        spdlog::error("[VideoDecoder] No H.264 decoder available");
        return false;
    }
    context_ = avcodec_alloc_context3(codec);
    frame_ = av_frame_alloc();
    packet_ = av_packet_alloc();
    if (context_ == nullptr || frame_ == nullptr || packet_ == nullptr) {
        close();
        return false;
    }
    if (avcodec_open2(context_, codec, nullptr) < 0) {
        spdlog::error("[VideoDecoder] Failed to open the H.264 decoder");
        close();
        return false;
    }
    return true;
}

void VideoDecoder::close() {
    if (scaler_ != nullptr) {
        sws_freeContext(scaler_);
        scaler_ = nullptr;
    }
    if (packet_ != nullptr) {
        av_packet_free(&packet_);
    }
    if (frame_ != nullptr) {
        av_frame_free(&frame_);
    }
    if (context_ != nullptr) {
        avcodec_free_context(&context_);
    }
    scaler_width_ = 0;
    scaler_height_ = 0;
    scaler_format_ = -1;
}

void VideoDecoder::flush() {
    if (context_ != nullptr) {
        avcodec_flush_buffers(context_);
    }
}

bool VideoDecoder::decode(const std::byte *data, size_t len, cv::Mat &bgr) {
    if (context_ == nullptr || data == nullptr || len == 0) {
        return false;
    }
    packet_->data = const_cast<uint8_t *>(reinterpret_cast<const uint8_t *>(data));
    packet_->size = static_cast<int>(len);
    if (avcodec_send_packet(context_, packet_) < 0) {
        return false;
    }
    if (avcodec_receive_frame(context_, frame_) < 0) {
        return false;
    }

    if (scaler_ == nullptr || scaler_width_ != frame_->width || scaler_height_ != frame_->height ||
        scaler_format_ != frame_->format) {
        if (scaler_ != nullptr) {
            sws_freeContext(scaler_);
        }
        scaler_ = sws_getContext(frame_->width, frame_->height,
                                 static_cast<AVPixelFormat>(frame_->format), frame_->width,
                                 frame_->height, AV_PIX_FMT_BGR24, SWS_BILINEAR, nullptr, nullptr,
                                 nullptr);
        scaler_width_ = frame_->width;
        scaler_height_ = frame_->height;
        scaler_format_ = frame_->format;
    }
    if (scaler_ == nullptr) {
        return false;
    }

    bgr.create(frame_->height, frame_->width, CV_8UC3);
    uint8_t *destination_data[4] = {bgr.data, nullptr, nullptr, nullptr};
    const int destination_stride[4] = {static_cast<int>(bgr.step), 0, 0, 0};
    sws_scale(scaler_, frame_->data, frame_->linesize, 0, frame_->height, destination_data,
              destination_stride);
    return true;
}

bool is_keyframe_access_unit(const std::byte *data, size_t len) {
    // Walk the Annex-B start codes looking for NAL type 5 (IDR). Cheaper than decoding, and the
    // seek path needs the answer before it has a decoder running.
    for (size_t i = 0; i + 4 < len; ++i) {
        const auto byte_at = [data](size_t index) { return static_cast<uint8_t>(data[index]); };
        const bool short_code = byte_at(i) == 0 && byte_at(i + 1) == 0 && byte_at(i + 2) == 1;
        const bool long_code = i + 4 < len && byte_at(i) == 0 && byte_at(i + 1) == 0 &&
                               byte_at(i + 2) == 0 && byte_at(i + 3) == 1;
        if (!short_code && !long_code) {
            continue;
        }
        const size_t header = i + (long_code ? 4 : 3);
        if (header >= len) {
            break;
        }
        if ((byte_at(header) & 0x1F) == 5) {
            return true;
        }
    }
    return false;
}
}  // namespace auto_battlebot
