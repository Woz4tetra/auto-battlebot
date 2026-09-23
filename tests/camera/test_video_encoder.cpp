#include <gtest/gtest.h>

#include <map>
#include <mutex>

#include "rgbd_camera/video_encoder.hpp"

namespace auto_battlebot {
namespace {

TEST(VideoEncoderTest, RequestKeyframeMakesTheNextFrameAKeyframe) {
    VideoEncoder encoder;
    VideoEncoderOptions options;
    options.width = 320;
    options.height = 240;
    options.fps = 30;
    options.bitrate = 500000;
    options.keyframe_interval = 1000;  // Only the first frame and the requested one are keyframes.
    options.queue_depth = 64;
    options.input_is_uyvy = false;

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
    EXPECT_FALSE(keyframe_by_log_time[7]) << encoder.codec_name();
    EXPECT_TRUE(keyframe_by_log_time[8]) << encoder.codec_name();
    EXPECT_FALSE(keyframe_by_log_time[9]) << encoder.codec_name();
}

}  // namespace
}  // namespace auto_battlebot
