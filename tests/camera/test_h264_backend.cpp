#include <gtest/gtest.h>

#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

#include "rgbd_camera/h264_backend.hpp"

namespace auto_battlebot {
namespace {

constexpr int kFrames = 30;
constexpr int kKeyframeInterval = 10;
constexpr int kNalSps = 7;
constexpr int kNalPps = 8;

struct Packet {
    int64_t pts = 0;
    bool keyframe = false;
    bool has_sps = false;
    bool has_pps = false;
};

bool has_nal(const std::byte *data, size_t len, int nal_type) {
    for (size_t i = 0; i + 3 < len; ++i) {
        if (data[i] == std::byte{0} && data[i + 1] == std::byte{0} && data[i + 2] == std::byte{1} &&
            (std::to_integer<int>(data[i + 3]) & 0x1f) == nal_type) {
            return true;
        }
    }
    return false;
}

using BackendOpener = std::function<std::unique_ptr<H264Backend>(const VideoEncoderOptions &,
                                                                 H264Backend::PacketSink)>;

struct NamedOpener {
    std::string name;
    std::function<bool()> available;
    BackendOpener open;
};

std::vector<NamedOpener> openers() {
    std::vector<NamedOpener> result;
    result.push_back({"ffmpeg_libx264", [] { return ffmpeg_h264_registered("libx264"); },
                      [](const VideoEncoderOptions &options, H264Backend::PacketSink sink) {
                          return open_ffmpeg_h264("libx264", options, std::move(sink));
                      }});
#ifdef AUTO_BATTLEBOT_HAVE_GSTREAMER
    result.push_back({"gst_x264", [] { return gst_x264_installed(); }, open_gst_x264});
#endif
    return result;
}

class H264BackendTest : public ::testing::TestWithParam<size_t> {
   protected:
    std::vector<Packet> encode_all() {
        const NamedOpener &opener = openers()[GetParam()];
        VideoEncoderOptions options;
        options.width = 320;
        options.height = 240;
        options.fps = 30;
        options.bitrate = 1000000;
        options.keyframe_interval = kKeyframeInterval;
        options.input_format = VideoInputFormat::BGR;
        options.software_preview = true;

        std::mutex mutex;
        std::vector<Packet> packets;
        auto backend = opener.open(
            options, [&](const std::byte *data, size_t len, int64_t pts, bool keyframe) {
                std::lock_guard<std::mutex> lock(mutex);
                packets.push_back(Packet{pts, keyframe, has_nal(data, len, kNalSps),
                                         has_nal(data, len, kNalPps)});
            });
        if (!backend) {
            return {};
        }
        for (int i = 0; i < kFrames; ++i) {
            cv::Mat frame(options.height, options.width, CV_8UC3,
                          cv::Scalar((i * 7) % 255, (i * 13) % 255, (i * 29) % 255));
            while (!backend->encode(frame.clone(), i, false)) {
            }
        }
        backend->finish();
        std::lock_guard<std::mutex> lock(mutex);
        return packets;
    }
};

TEST_P(H264BackendTest, FlushDeliversAllFrames) {
    if (!openers()[GetParam()].available()) GTEST_SKIP() << "encoder not installed";
    const std::vector<Packet> packets = encode_all();
    ASSERT_EQ(packets.size(), static_cast<size_t>(kFrames));
}

TEST_P(H264BackendTest, EveryIdrCarriesSpsPps) {
    if (!openers()[GetParam()].available()) GTEST_SKIP() << "encoder not installed";
    const std::vector<Packet> packets = encode_all();
    ASSERT_EQ(packets.size(), static_cast<size_t>(kFrames));
    int keyframes = 0;
    for (const Packet &packet : packets) {
        if (!packet.keyframe) continue;
        ++keyframes;
        EXPECT_TRUE(packet.has_sps) << "keyframe at pts " << packet.pts << " has no SPS";
        EXPECT_TRUE(packet.has_pps) << "keyframe at pts " << packet.pts << " has no PPS";
    }
    EXPECT_GE(keyframes, kFrames / kKeyframeInterval);
}

TEST_P(H264BackendTest, PacketsLeaveInCaptureOrderBecauseThereAreNoBFrames) {
    if (!openers()[GetParam()].available()) GTEST_SKIP() << "encoder not installed";
    const std::vector<Packet> packets = encode_all();
    ASSERT_EQ(packets.size(), static_cast<size_t>(kFrames));
    for (size_t i = 0; i < packets.size(); ++i) {
        EXPECT_EQ(packets[i].pts, static_cast<int64_t>(i)) << "packet " << i << " reordered";
    }
}

INSTANTIATE_TEST_SUITE_P(Backends, H264BackendTest, ::testing::Range<size_t>(0, openers().size()),
                         [](const ::testing::TestParamInfo<size_t> &info) {
                             return openers()[info.param].name;
                         });

}  // namespace
}  // namespace auto_battlebot
