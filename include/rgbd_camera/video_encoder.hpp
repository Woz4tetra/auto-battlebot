#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <unordered_map>

namespace auto_battlebot {
class H264Backend;

enum class VideoInputFormat { BGR, BGRA, UYVY };

struct VideoEncoderOptions {
    int width = 1920;
    int height = 1200;
    int fps = 60;
    int64_t bitrate = 15000000;
    int keyframe_interval = 30;
    /** Frames the encoder may fall behind before it starts dropping. A submit-and-wait to NVENC
     *  from the perception loop is a blocking call in the latency budget; dropping is the only
     *  other option, and a silent drop is worse than a counted one. */
    size_t queue_depth = 4;
    VideoInputFormat input_format = VideoInputFormat::UYVY;
    bool software_preview = false;
};

/**
 * @brief H.264 encoder on its own thread, writing one Annex-B access unit per frame.
 *
 * Encoder selection is FFmpeg's h264_nvenc (desktop), FFmpeg's h264_nvv4l2m2m, GStreamer's
 * nvv4l2h264enc (Jetson hardware encoder), then libx264 (FFmpeg, else GStreamer x264enc). A
 * candidate counts only if it opens: the ZED Box's stock FFmpeg registers h264_nvenc, which cannot
 * open without libnvidia-encode. On aarch64 libx264 is only for `software_preview`: at 1920x1200 it
 * managed 39-43 fps on all eight Orin cores and would starve perception.
 *
 * B-frames are off: they reorder PTS, and both `start_frame` on playback and reading `log_time` as
 * the capture instant depend on the nth message being the nth captured frame.
 */
class VideoEncoder {
   public:
    using PacketSink =
        std::function<void(const std::byte *data, size_t len, uint64_t log_time_ns, bool keyframe)>;

    VideoEncoder();
    ~VideoEncoder();
    VideoEncoder(const VideoEncoder &) = delete;
    VideoEncoder &operator=(const VideoEncoder &) = delete;

    using BackendSink =
        std::function<void(const std::byte *data, size_t len, int64_t pts, bool keyframe)>;
    using BackendFactory = std::function<std::unique_ptr<H264Backend>(
        const VideoEncoderOptions &options, BackendSink sink)>;

    bool start(const VideoEncoderOptions &options, PacketSink sink,
               const BackendFactory &factory = nullptr);
    /** Flushes the encoder, so the last frames submitted still reach the sink. */
    void stop();
    bool running() const { return running_.load(); }

    /** Non-blocking. Copies the frame into the queue and returns; drops when the queue is full. */
    /** The frame's index in this encode session, which is the nth /camera/video message of the
     *  recording it lands in; -1 when the frame was not taken (stopped, or the queue is full).
     *  Cameras write this into frame_meta: the encoder alone decides which frames become
     *  packets, so a count kept anywhere else drifts the first time one is dropped. */
    int64_t submit(const cv::Mat &frame, uint64_t log_time_ns);

    void request_keyframe() { keyframe_requested_.store(true); }

    static bool hardware_encoder_installed();

    static bool software_encoder_available();

    static std::string candidate_names();

    /** The backend refused a frame that was already numbered. From then on submit() takes
     *  nothing, so every index already handed out either has its packet or points past the
     *  last one. Only a stop() and start() clears it: the caller restarts the recording. */
    bool broken() const { return broken_.load(); }

    uint64_t encoded_frames() const { return encoded_frames_.load(); }
    uint64_t dropped_frames() const { return dropped_frames_.load(); }
    const std::string &codec_name() const { return codec_name_; }

   private:
    struct QueuedFrame {
        cv::Mat image;
        uint64_t log_time_ns = 0;
        bool force_keyframe = false;
    };

    void encode_loop();
    void deliver(const std::byte *data, size_t len, int64_t pts, bool keyframe);

    VideoEncoderOptions options_;
    PacketSink sink_;
    std::string codec_name_;
    std::unique_ptr<H264Backend> backend_;

    int64_t next_pts_ = 0;
    /** Guarded by mutex_. Equals next_pts_ once the queue drains. */
    int64_t next_submit_index_ = 0;
    std::unordered_map<int64_t, uint64_t> log_time_by_frame_ordinal_;
    std::mutex pts_mutex_;

    std::deque<QueuedFrame> queue_;
    mutable std::mutex mutex_;
    std::condition_variable queue_cv_;
    std::thread thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> stopping_{false};
    std::atomic<bool> keyframe_requested_{false};
    std::atomic<bool> broken_{false};
    std::atomic<uint64_t> encoded_frames_{0};
    std::atomic<uint64_t> dropped_frames_{0};
};
}  // namespace auto_battlebot
