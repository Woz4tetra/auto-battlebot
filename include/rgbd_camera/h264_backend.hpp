#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <opencv2/core.hpp>
#include <string>

#include "rgbd_camera/video_encoder.hpp"

namespace auto_battlebot {
class H264Backend {
   public:
    using PacketSink =
        std::function<void(const std::byte *data, size_t len, int64_t pts, bool keyframe)>;

    virtual ~H264Backend() = default;
    virtual bool encode(cv::Mat image, int64_t pts, bool force_keyframe) = 0;
    virtual void finish() = 0;
    virtual const std::string &name() const = 0;
};

std::unique_ptr<H264Backend> open_ffmpeg_h264(const std::string &codec,
                                              const VideoEncoderOptions &options,
                                              H264Backend::PacketSink sink);
bool ffmpeg_h264_registered(const std::string &codec);

#ifdef AUTO_BATTLEBOT_HAVE_GSTREAMER
std::unique_ptr<H264Backend> open_gst_nvv4l2_h264(const VideoEncoderOptions &options,
                                                  H264Backend::PacketSink sink);
bool gst_nvv4l2_h264_installed();
/** x264enc through GStreamer: the preview's software encoder where FFmpeg has no libx264, which is
 *  NVIDIA's FFmpeg build on JetPack 7. */
std::unique_ptr<H264Backend> open_gst_x264(const VideoEncoderOptions &options,
                                           H264Backend::PacketSink sink);
bool gst_x264_installed();
#endif
}  // namespace auto_battlebot
