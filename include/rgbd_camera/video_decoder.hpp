#pragma once

#include <cstddef>
#include <opencv2/opencv.hpp>

struct AVCodecContext;
struct AVFrame;
struct AVPacket;
struct SwsContext;

namespace auto_battlebot {
/**
 * @brief H.264 Annex-B decoder for `/camera/video`, one access unit in, one BGR frame out.
 *
 * Stateful where cv::imdecode was not: it holds a codec context for the life of a file and has to
 * be fed in stream order. Seeking means finding the last IDR at or before the target and decoding
 * forward from there, never jumping into the middle of a GOP. `sl::Camera::setSVOPosition` has
 * exactly this problem, landing on non-keyframes so the next grabs return smeared RGB, and it
 * applies to any inter-coded stream.
 */
class VideoDecoder {
   public:
    VideoDecoder() = default;
    ~VideoDecoder();
    VideoDecoder(const VideoDecoder &) = delete;
    VideoDecoder &operator=(const VideoDecoder &) = delete;

    bool open();
    void close();
    bool is_open() const { return context_ != nullptr; }

    /** Decode one access unit. Returns false when the packet produced no frame yet, which is
     *  normal for the first packets of a stream. */
    bool decode(const std::byte *data, size_t len, cv::Mat &bgr);

    /** Drop decoder state at a seek, so the next IDR starts clean. */
    void flush();

   private:
    AVCodecContext *context_ = nullptr;
    AVFrame *frame_ = nullptr;
    AVPacket *packet_ = nullptr;
    SwsContext *scaler_ = nullptr;
    int scaler_width_ = 0;
    int scaler_height_ = 0;
    int scaler_format_ = -1;
};

/** True when an Annex-B access unit contains an IDR NAL, so a decoder can start from it. */
bool is_keyframe_access_unit(const std::byte *data, size_t len);
}  // namespace auto_battlebot
