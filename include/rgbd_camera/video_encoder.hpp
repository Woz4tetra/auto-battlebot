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
    size_t queue_depth = 4;
    VideoInputFormat input_format = VideoInputFormat::UYVY;
    bool software_preview = false;
};

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
    void stop();
    bool running() const { return running_.load(); }

    int64_t submit(const cv::Mat &frame, uint64_t log_time_ns);

    void request_keyframe() { keyframe_requested_.store(true); }

    static bool hardware_encoder_installed();

    static bool software_encoder_available();

    static std::string candidate_names();

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
