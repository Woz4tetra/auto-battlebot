#include <gtest/gtest.h>

#include <chrono>
#include <map>
#include <mutex>
#include <thread>
#include <vector>

#include "rgbd_camera/h264_backend.hpp"
#include "rgbd_camera/video_encoder.hpp"

namespace auto_battlebot {
namespace {

// Emits one byte per frame, and refuses the frame with pts `refuse_pts`.
class RefusingBackend : public H264Backend {
   public:
    RefusingBackend(VideoEncoder::BackendSink sink, int64_t refuse_pts)
        : sink_(std::move(sink)), refuse_pts_(refuse_pts) {}
    bool encode(cv::Mat, int64_t pts, bool) override {
        // Slow enough that frames queue up behind the refused one.
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        if (pts == refuse_pts_) return false;
        const std::byte byte{0};
        sink_(&byte, 1, pts, pts == 0);
        return true;
    }
    void finish() override {}
    const std::string &name() const override { return name_; }

   private:
    VideoEncoder::BackendSink sink_;
    int64_t refuse_pts_;
    std::string name_ = "refusing";
};

TEST(VideoEncoderTest, RequestKeyframeMakesTheNextFrameAKeyframe) {
    VideoEncoder encoder;
    VideoEncoderOptions options;
    options.width = 320;
    options.height = 240;
    options.fps = 30;
    options.bitrate = 500000;
    options.keyframe_interval = 1000;  // Only the first frame and the requested one are keyframes.
    options.queue_depth = 64;
    options.input_format = VideoInputFormat::BGR;

    std::mutex mutex;
    std::map<uint64_t, bool> keyframe_by_log_time;
    const bool started =
        encoder.start(options, [&](const std::byte *, size_t, uint64_t log_time_ns, bool keyframe) {
            std::lock_guard<std::mutex> lock(mutex);
            keyframe_by_log_time[log_time_ns] = keyframe;
        });
    if (!started) GTEST_SKIP() << "No H.264 encoder available";

    for (uint64_t t = 1; t <= 12; ++t) {
        cv::Mat frame(options.height, options.width, CV_8UC3,
                      cv::Scalar(static_cast<double>(t * 10), 64, 128));
        if (t == 8) encoder.request_keyframe();
        encoder.submit(frame, t);
    }
    encoder.stop();

    std::lock_guard<std::mutex> lock(mutex);
    ASSERT_EQ(keyframe_by_log_time.size(), 12u) << encoder.codec_name();
    EXPECT_TRUE(keyframe_by_log_time[1]);
    std::vector<uint64_t> requested_keyframes;
    for (uint64_t t = 2; t <= 12; ++t) {
        if (keyframe_by_log_time[t]) requested_keyframes.push_back(t);
    }
    ASSERT_EQ(requested_keyframes.size(), 1u) << encoder.codec_name();
    // nvv4l2h264enc's force-IDR applies to the next frame the hardware encodes. When the hardware
    // is a frame behind, that is the one before the requested frame. One frame early still lets a
    // new preview viewer start decoding at once, which is all request_keyframe() is for.
    if (encoder.codec_name() == "nvv4l2h264enc") {
        EXPECT_GE(requested_keyframes[0], 7u);
        EXPECT_LE(requested_keyframes[0], 8u);
    } else {
        EXPECT_EQ(requested_keyframes[0], 8u) << encoder.codec_name();
    }
}

// frame_meta carries submit()'s return value as video_frame_index, and playback reads it as the
// position of the packet in the file. So within one session: indices 0, 1, 2... for exactly the
// frames that become packets, -1 for a refused frame, and nothing from an earlier session.
TEST(VideoEncoderTest, SubmitIndexIsThePacketPositionInEachSession) {
    VideoEncoderOptions options;
    options.width = 320;
    options.height = 240;
    options.fps = 30;
    options.bitrate = 500000;
    options.queue_depth = 64;
    options.input_format = VideoInputFormat::BGR;

    VideoEncoder encoder;
    for (int session = 0; session < 3; ++session) {
        std::mutex mutex;
        std::vector<uint64_t> packet_times;
        const bool started =
            encoder.start(options, [&](const std::byte *, size_t, uint64_t log_time_ns, bool) {
                std::lock_guard<std::mutex> lock(mutex);
                packet_times.push_back(log_time_ns);
            });
        if (!started) GTEST_SKIP() << "No H.264 encoder available";

        const uint64_t base = 1000 * static_cast<uint64_t>(session + 1);
        std::vector<int64_t> indices;
        for (uint64_t t = 0; t < 10; ++t) {
            cv::Mat frame(options.height, options.width, CV_8UC3,
                          cv::Scalar(static_cast<double>(t * 20), 64, 128));
            indices.push_back(encoder.submit(frame, base + t));
        }
        encoder.stop();
        cv::Mat late(options.height, options.width, CV_8UC3, cv::Scalar::all(0));
        EXPECT_EQ(encoder.submit(late, base + 99), -1) << "a stopped encoder took a frame";

        std::lock_guard<std::mutex> lock(mutex);
        ASSERT_EQ(packet_times.size(), 10u) << "session " << session;
        for (int64_t i = 0; i < 10; ++i) {
            EXPECT_EQ(indices[static_cast<size_t>(i)], i) << "session " << session;
            EXPECT_EQ(packet_times[static_cast<size_t>(i)], base + static_cast<uint64_t>(i))
                << "session " << session << ": packet " << i << " is not the frame numbered " << i;
        }
    }
}

// After the backend refuses a numbered frame, no index may point at the wrong packet: every index
// handed out is either its packet's position or past the last packet.
TEST(VideoEncoderTest, RefusedFrameStopsTheSessionInsteadOfShiftingIndices) {
    VideoEncoderOptions options;
    options.width = 32;
    options.height = 24;
    options.queue_depth = 64;
    std::mutex mutex;
    std::vector<uint64_t> packet_times;
    const auto sink = [&](const std::byte *, size_t, uint64_t log_time_ns, bool) {
        std::lock_guard<std::mutex> lock(mutex);
        packet_times.push_back(log_time_ns);
    };
    const auto refuse_at = [](int64_t pts) {
        return [pts](const VideoEncoderOptions &, VideoEncoder::BackendSink backend_sink) {
            return std::make_unique<RefusingBackend>(std::move(backend_sink), pts);
        };
    };

    VideoEncoder encoder;
    ASSERT_TRUE(encoder.start(options, sink, refuse_at(3)));
    std::vector<int64_t> indices;
    for (uint64_t t = 1; t <= 20; ++t) {
        indices.push_back(encoder.submit(cv::Mat(24, 32, CV_8UC3, cv::Scalar::all(0)), t));
    }
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!encoder.broken() && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    ASSERT_TRUE(encoder.broken());
    EXPECT_EQ(encoder.submit(cv::Mat(24, 32, CV_8UC3, cv::Scalar::all(0)), 99), -1);
    encoder.stop();

    {
        std::lock_guard<std::mutex> lock(mutex);
        ASSERT_EQ(packet_times, (std::vector<uint64_t>{1, 2, 3}));
        for (size_t i = 0; i < indices.size(); ++i) {
            const int64_t index = indices[i];
            if (index < 0 || index >= static_cast<int64_t>(packet_times.size())) continue;
            EXPECT_EQ(packet_times[static_cast<size_t>(index)], i + 1)
                << "frame " << i + 1 << " was given index " << index << ", another frame's packet";
        }
        packet_times.clear();
    }

    // A restart clears it, and numbering starts over.
    ASSERT_TRUE(encoder.start(options, sink, refuse_at(-1)));
    EXPECT_FALSE(encoder.broken());
    EXPECT_EQ(encoder.submit(cv::Mat(24, 32, CV_8UC3, cv::Scalar::all(0)), 100), 0);
    encoder.stop();
    std::lock_guard<std::mutex> lock(mutex);
    EXPECT_EQ(packet_times, (std::vector<uint64_t>{100}));
}

}  // namespace
}  // namespace auto_battlebot
