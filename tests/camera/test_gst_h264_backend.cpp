#include <gtest/gtest.h>

#ifdef AUTO_BATTLEBOT_HAVE_GSTREAMER
#include <chrono>
#include <map>
#include <mutex>
#include <opencv2/core.hpp>
#include <thread>

#include "rgbd_camera/h264_backend.hpp"

namespace auto_battlebot {
namespace {

struct Packet {
    bool keyframe = false;
    size_t bytes = 0;
};

class GstH264BackendTest : public ::testing::TestWithParam<VideoInputFormat> {};

// The Jetson's encoder through GStreamer: every frame comes back once, in order, with its pts, the
// first and a forced frame as IDRs and nothing else, and each IDR carrying SPS ahead of it.
TEST_P(GstH264BackendTest, EncodesEveryFrameWithItsPtsAndForcedKeyframe) {
    if (!gst_nvv4l2_h264_installed()) GTEST_SKIP() << "nvv4l2h264enc not installed";
    VideoEncoderOptions options;
    options.width = 640;
    options.height = 480;
    options.fps = 60;
    options.bitrate = 4000000;
    options.keyframe_interval = 1000;  // Only the first frame and the forced one are keyframes.
    options.input_format = GetParam();

    std::mutex mutex;
    std::map<int64_t, Packet> by_pts;
    std::map<int64_t, bool> has_sps;
    auto backend = open_gst_nvv4l2_h264(options, [&](const std::byte *data, size_t len, int64_t pts,
                                                     bool keyframe) {
        std::lock_guard<std::mutex> lock(mutex);
        by_pts[pts] = Packet{keyframe, len};
        // Annex-B NAL type 7 (SPS) after a 00 00 01 start code.
        bool sps = false;
        for (size_t i = 0; i + 3 < len; ++i) {
            if (data[i] == std::byte{0} && data[i + 1] == std::byte{0} &&
                data[i + 2] == std::byte{1} && (std::to_integer<int>(data[i + 3]) & 0x1f) == 7) {
                sps = true;
                break;
            }
        }
        has_sps[pts] = sps;
    });
    ASSERT_NE(backend, nullptr) << "nvv4l2h264enc is installed but the pipeline did not open";

    const int type = GetParam() == VideoInputFormat::BGR    ? CV_8UC3
                     : GetParam() == VideoInputFormat::BGRA ? CV_8UC4
                                                            : CV_8UC2;
    constexpr int kFrames = 60;
    constexpr int kForced = 40;
    for (int i = 0; i < kFrames; ++i) {
        cv::Mat frame(options.height, options.width, type, cv::Scalar::all((i * 4) % 255));
        // Paced like a camera, so the drop-when-queued guard never fires.
        while (!backend->encode(frame.clone(), i, i == kForced)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    backend->finish();

    std::lock_guard<std::mutex> lock(mutex);
    ASSERT_EQ(by_pts.size(), static_cast<size_t>(kFrames));
    EXPECT_EQ(by_pts.begin()->first, 0);
    EXPECT_EQ(by_pts.rbegin()->first, kFrames - 1);
    for (const auto &[pts, packet] : by_pts) {
        EXPECT_GT(packet.bytes, 0u) << pts;
        EXPECT_EQ(packet.keyframe, pts == 0 || pts == kForced) << "pts " << pts;
        if (packet.keyframe) EXPECT_TRUE(has_sps[pts]) << "IDR without SPS at pts " << pts;
    }
}

INSTANTIATE_TEST_SUITE_P(InputFormats, GstH264BackendTest,
                         ::testing::Values(VideoInputFormat::BGR, VideoInputFormat::BGRA,
                                           VideoInputFormat::UYVY));

}  // namespace
}  // namespace auto_battlebot
#endif
