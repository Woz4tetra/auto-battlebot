#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <opencv2/core.hpp>
#include <string>

#include "rgbd_camera/video_encoder.hpp"

namespace auto_battlebot {
/**
 * @brief One H.264 implementation behind VideoEncoder: FFmpeg or GStreamer.
 *
 * VideoEncoder owns the queue, the encode thread, and the pts -> capture time map. A backend only
 * turns frames into Annex-B access units, with SPS/PPS in band ahead of every IDR and no B-frames.
 */
class H264Backend {
   public:
    /** pts is the frame ordinal VideoEncoder assigned in encode(). */
    using PacketSink =
        std::function<void(const std::byte *data, size_t len, int64_t pts, bool keyframe)>;

    virtual ~H264Backend() = default;
    /** Takes the frame by value so a backend can hand its memory to the encoder without a copy.
     *  Called from the encode thread only. */
    virtual bool encode(cv::Mat image, int64_t pts, bool force_keyframe) = 0;
    /** Flushes: every frame encode() accepted reaches the sink before this returns. */
    virtual void finish() = 0;
    virtual const std::string &name() const = 0;
};

/** nullptr when FFmpeg does not register `codec` or it fails to open. */
std::unique_ptr<H264Backend> open_ffmpeg_h264(const std::string &codec,
                                              const VideoEncoderOptions &options,
                                              H264Backend::PacketSink sink);
bool ffmpeg_h264_registered(const std::string &codec);

#ifdef AUTO_BATTLEBOT_HAVE_GSTREAMER
/** nvv4l2h264enc through GStreamer. nullptr when an element is missing or the pipeline does not
 *  reach PLAYING, which is where nvv4l2h264enc opens the encoder device. */
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
