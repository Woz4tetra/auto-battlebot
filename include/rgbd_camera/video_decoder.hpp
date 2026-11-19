#pragma once

#include <cstddef>
#include <opencv2/opencv.hpp>

struct AVCodecContext;
struct AVFrame;
struct AVPacket;
struct SwsContext;

namespace auto_battlebot {
class VideoDecoder {
   public:
    VideoDecoder() = default;
    ~VideoDecoder();
    VideoDecoder(const VideoDecoder &) = delete;
    VideoDecoder &operator=(const VideoDecoder &) = delete;

    bool open();
    void close();
    bool is_open() const { return context_ != nullptr; }

    bool decode(const std::byte *data, size_t len, cv::Mat &bgr);

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

bool is_keyframe_access_unit(const std::byte *data, size_t len);
}  // namespace auto_battlebot
