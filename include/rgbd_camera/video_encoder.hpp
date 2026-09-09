#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <unordered_map>

struct AVCodecContext;
struct AVFrame;
struct AVPacket;
struct SwsContext;

namespace auto_battlebot {
struct VideoEncoderOptions {
    int width = 1920;
    int height = 1200;
    int fps = 60;
    /** 15 Mbps is 1.9 MB/s, about 6.8 GB per hour, and a three-minute match is about 340 MB. */
    int64_t bitrate = 15000000;
    /** IDR every half second at 60 fps. This sets the seek granularity on playback and bounds how
     *  far a decoder has to run forward from a chunk boundary. */
    int keyframe_interval = 30;
    /** Frames the encoder may fall behind before it starts dropping. A submit-and-wait to NVENC
     *  from the perception loop is a blocking call in the latency budget; dropping is the only
     *  other option, and a silent drop is worse than a counted one. */
    size_t queue_depth = 4;
    /** UYVY straight off the V4L2 buffer, rather than the BGR copy the detectors use. libswscale
     *  goes UYVY to NV12 without the detour, and it means the recording is pre-rectification, so
     *  a revised calibration can be applied to footage already shot. */
    bool input_is_uyvy = true;
};

/**
 * @brief H.264 encoder on its own thread, writing one Annex-B access unit per frame.
 *
 * Encoder selection is h264_nvenc, then h264_nvv4l2m2m on the Jetson, then libx264. B-frames are
 * off: they reorder PTS, and both `start_frame` on playback and reading `log_time` as the capture
 * instant depend on the nth message being the nth captured frame.
 */
class VideoEncoder {
   public:
    /** One encoded access unit, with the capture time of the frame it came from. */
    using PacketSink =
        std::function<void(const std::byte *data, size_t len, uint64_t log_time_ns, bool keyframe)>;

    VideoEncoder() = default;
    ~VideoEncoder();
    VideoEncoder(const VideoEncoder &) = delete;
    VideoEncoder &operator=(const VideoEncoder &) = delete;

    bool start(const VideoEncoderOptions &options, PacketSink sink);
    /** Flushes the encoder, so the last frames submitted still reach the sink. */
    void stop();
    bool running() const { return running_.load(); }

    /** Non-blocking. Copies the frame into the queue and returns; drops when the queue is full. */
    void submit(const cv::Mat &frame, uint64_t log_time_ns);

    uint64_t encoded_frames() const { return encoded_frames_.load(); }
    uint64_t dropped_frames() const { return dropped_frames_.load(); }
    const std::string &codec_name() const { return codec_name_; }

   private:
    struct QueuedFrame {
        cv::Mat image;
        uint64_t log_time_ns = 0;
    };

    void encode_loop();
    bool encode_one(const QueuedFrame &frame);
    void drain();
    void close_codec();

    VideoEncoderOptions options_;
    PacketSink sink_;
    std::string codec_name_;

    AVCodecContext *context_ = nullptr;
    AVFrame *scaled_ = nullptr;
    AVPacket *packet_ = nullptr;
    SwsContext *scaler_ = nullptr;
    int64_t next_pts_ = 0;
    /** pts is the frame ordinal, so the capture time survives whatever latency the encoder holds
     *  frames for. */
    std::unordered_map<int64_t, uint64_t> log_time_by_pts_;

    std::deque<QueuedFrame> queue_;
    mutable std::mutex mutex_;
    std::condition_variable queue_cv_;
    std::thread thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> stopping_{false};
    std::atomic<uint64_t> encoded_frames_{0};
    std::atomic<uint64_t> dropped_frames_{0};
};
}  // namespace auto_battlebot
