#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <foxglove/schemas.hpp>
#include <memory>
#include <thread>

#include "config/config_parser.hpp"
#include "data_structures.hpp"
#include "foxglove_adapters/camera_info.hpp"
#include "foxglove_adapters/json_schemas.hpp"
#include "mcap_recorder/mcap_recorder.hpp"
#include "publisher/output_channel.hpp"
#include "rgbd_camera/camera_calibration.hpp"
#include "rgbd_camera/config.hpp"
#include "rgbd_camera/video_encoder.hpp"
#include "rgbd_camera/video_playback_camera.hpp"

namespace auto_battlebot {
namespace {
constexpr int kWidth = 320;
constexpr int kHeight = 240;
constexpr int kFrames = 45;
constexpr const char *kCalibrationId = "video_playback_test";

/** A recognisable UYVY frame whose content changes with the index, so a decoded frame can be
 *  matched back to the one that produced it. */
cv::Mat make_uyvy_frame(int index) {
    cv::Mat frame(kHeight, kWidth, CV_8UC2);
    for (int row = 0; row < kHeight; ++row) {
        for (int col = 0; col < kWidth; ++col) {
            const auto luma = static_cast<uint8_t>((col + row + index * 4) % 256);
            frame.at<cv::Vec2b>(row, col) = cv::Vec2b(128, luma);
        }
    }
    return frame;
}

/** Writes a short recording the way the RGB camera does: H.264 on /camera/video, FrameMeta
 *  alongside it, and a calibration_id in the file metadata. */
class RecordingFixture {
   public:
    RecordingFixture() {
        calibration_path_ = (std::filesystem::temp_directory_path() /
                             "auto_battlebot_video_playback_calibration.toml")
                                .string();
        CameraCalibration calibration;
        calibration.calibration_id = kCalibrationId;
        calibration.width = kWidth;
        calibration.height = kHeight;
        calibration.fx = 200.0;
        calibration.fy = 200.0;
        calibration.cx = kWidth / 2.0;
        calibration.cy = kHeight / 2.0;
        save_camera_calibration(calibration_path_, calibration);

        recorder_ = std::make_shared<McapRecorder>("video_playback_test");
        recorder_->write_metadata("calibration_id", kCalibrationId);
        path_ = recorder_->file_path();

        OutputChannel video("/camera/video", "protobuf",
                            VizSchema::from_sdk(foxglove::schemas::CompressedVideo::schema()),
                            false, nullptr, recorder_);
        VizSchema meta_schema;
        meta_schema.name = foxglove_adapters::kFrameMetaSchemaName;
        meta_schema.encoding = "jsonschema";
        meta_schema.data = foxglove_adapters::kFrameMetaSchema;
        OutputChannel meta("/camera/frame_meta", "json", meta_schema, false, nullptr, recorder_);

        VideoEncoder encoder;
        VideoEncoderOptions options;
        options.width = kWidth;
        options.height = kHeight;
        options.fps = 30;
        options.keyframe_interval = 15;
        options.bitrate = 2000000;
        started_ = encoder.start(
            options, [&video](const std::byte *data, size_t len, uint64_t log_time_ns, bool) {
                foxglove::schemas::CompressedVideo message;
                message.frame_id = "camera";
                message.format = "h264";
                message.data.assign(data, data + len);
                video.log_message(message, log_time_ns);
            });
        if (!started_) {
            return;
        }
        for (int i = 0; i < kFrames; ++i) {
            const uint64_t stamp_ns = kBaseStampNs + static_cast<uint64_t>(i) * 33333333ULL;
            FrameIdentity identity;
            identity.image_stamp_ns = stamp_ns;
            identity.video_frame_index = i;
            meta.log(foxglove_adapters::to_frame_meta_json(identity), stamp_ns);
            // Wait for the encoder to keep up rather than overrunning its bounded queue. A live
            // camera submits at frame rate; this loop would otherwise submit as fast as memcpy
            // allows and the fixture would come out with holes in it.
            const uint64_t before = encoder.dropped_frames();
            encoder.submit(make_uyvy_frame(i), stamp_ns);
            while (encoder.dropped_frames() != before) {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                const uint64_t retry = encoder.dropped_frames();
                encoder.submit(make_uyvy_frame(i), stamp_ns);
                if (encoder.dropped_frames() == retry) {
                    break;
                }
            }
        }
        encoder.stop();
        recorder_->close();
    }

    ~RecordingFixture() {
        std::error_code ec;
        std::filesystem::remove(path_, ec);
        std::filesystem::remove(calibration_path_, ec);
    }

    bool started() const { return started_; }
    const std::filesystem::path &path() const { return path_; }
    const std::string &calibration_path() const { return calibration_path_; }

    static constexpr uint64_t kBaseStampNs = 1788011445339499712ULL;

   private:
    std::shared_ptr<McapRecorder> recorder_;
    std::filesystem::path path_;
    std::string calibration_path_;
    bool started_ = false;
};

VideoPlaybackCameraConfiguration make_config(const RecordingFixture &fixture) {
    VideoPlaybackCameraConfiguration config;
    config.video_file_path = fixture.path().string();
    config.calibration_file = fixture.calibration_path();
    // Off, so the test does not sleep through the recording's own frame spacing.
    config.real_time_mode = false;
    config.rebase_stamps = false;
    return config;
}
}  // namespace

class VideoPlaybackCameraTest : public ::testing::Test {
   protected:
    static void SetUpTestSuite() { fixture_ = std::make_unique<RecordingFixture>(); }
    static void TearDownTestSuite() { fixture_.reset(); }

    void SetUp() override {
        if (!fixture_ || !fixture_->started()) {
            GTEST_SKIP() << "No H.264 encoder available to build the test recording";
        }
    }

    static std::unique_ptr<RecordingFixture> fixture_;
};

std::unique_ptr<RecordingFixture> VideoPlaybackCameraTest::fixture_;

TEST_F(VideoPlaybackCameraTest, FullDataPipeline) {
    VideoPlaybackCamera camera(make_config(*fixture_));
    ASSERT_TRUE(camera.initialize());

    CameraData data;
    ASSERT_TRUE(camera.get(data));

    EXPECT_EQ(data.camera_info.width, kWidth);
    EXPECT_EQ(data.camera_info.height, kHeight);
    EXPECT_FALSE(data.camera_info.intrinsics.empty());
    EXPECT_FALSE(data.camera_info.distortion.empty());

    EXPECT_FALSE(data.rgb.image.empty());
    EXPECT_EQ(data.rgb.image.type(), CV_8UC3);
    EXPECT_EQ(data.rgb.image.cols, data.camera_info.width);
    EXPECT_EQ(data.rgb.image.rows, data.camera_info.height);

    // The depth assertions from the SVO test invert. There is no depth, and playback must not
    // invent one: KeypointHeightGate abstains on an empty image and to_field_point_cloud returns
    // nullopt, both of which the pipeline already handles.
    EXPECT_TRUE(data.depth.image.empty());

    EXPECT_GT(data.tf_visodom_from_camera.header.stamp, 0.0);
    EXPECT_EQ(data.tf_visodom_from_camera.header.frame_id, FrameId::VISUAL_ODOMETRY);
    EXPECT_EQ(data.tf_visodom_from_camera.child_frame_id, FrameId::CAMERA);
    EXPECT_EQ(data.tf_visodom_from_camera.transform.tf.rows(), 4);
    // A clamped camera does not move.
    EXPECT_TRUE(data.tf_visodom_from_camera.transform.tf.isIdentity(1e-12));
    EXPECT_TRUE(data.tracking_ok);
}

TEST_F(VideoPlaybackCameraTest, MultipleFrameProcessing) {
    VideoPlaybackCamera camera(make_config(*fixture_));
    ASSERT_TRUE(camera.initialize());

    CameraData first;
    CameraData second;
    ASSERT_TRUE(camera.get(first));
    ASSERT_TRUE(camera.get(second));

    EXPECT_EQ(first.camera_info.width, second.camera_info.width);
    EXPECT_EQ(cv::norm(first.camera_info.intrinsics - second.camera_info.intrinsics), 0.0);
    EXPECT_NE(first.rgb.header.stamp, second.rgb.header.stamp);
    EXPECT_EQ(second.frame_identity.video_frame_index, first.frame_identity.video_frame_index + 1);
}

TEST_F(VideoPlaybackCameraTest, DataIndependence) {
    VideoPlaybackCamera camera(make_config(*fixture_));
    ASSERT_TRUE(camera.initialize());

    CameraData reference;
    ASSERT_TRUE(camera.get(reference));
    CameraData copy;
    ASSERT_TRUE(camera.get(copy));
    copy.rgb.image = reference.rgb.image.clone();

    const cv::Vec3b original = reference.rgb.image.at<cv::Vec3b>(0, 0);
    copy.rgb.image.at<cv::Vec3b>(0, 0) = cv::Vec3b(255, 255, 255);
    EXPECT_EQ(reference.rgb.image.at<cv::Vec3b>(0, 0), original);
}

TEST_F(VideoPlaybackCameraTest, ReachesTheEndOfTheRecording) {
    VideoPlaybackCamera camera(make_config(*fixture_));
    ASSERT_TRUE(camera.initialize());
    EXPECT_FALSE(camera.should_close());

    CameraData data;
    int frames = 0;
    while (!camera.should_close() && frames < kFrames * 4) {
        if (!camera.get(data)) {
            break;
        }
        ++frames;
    }
    EXPECT_TRUE(camera.should_close());
    // The decoder primes on the first packets, so a frame or two may not come out.
    EXPECT_GE(frames, kFrames - 2);
    EXPECT_LE(frames, kFrames);
    EXPECT_EQ(camera.recorded_calibration_id(), kCalibrationId);
}

TEST_F(VideoPlaybackCameraTest, StartsAtTheRequestedFrame) {
    VideoPlaybackCameraConfiguration config = make_config(*fixture_);
    config.start_frame = 20;
    VideoPlaybackCamera camera(config);
    ASSERT_TRUE(camera.initialize());

    CameraData data;
    ASSERT_TRUE(camera.get(data));
    // Seeking runs forward from the last IDR at or before the target and discards until it, so the
    // first frame out is the one asked for and not the keyframe it decoded from.
    EXPECT_EQ(data.frame_identity.video_frame_index, 20);
}

TEST_F(VideoPlaybackCameraTest, RejectsAStartFrameBeyondTheEndOfTheFile) {
    // A scratch overlay that sets the file path but inherits start_frame from the config it
    // extends used to seek past the end and exit before the first heartbeat, which reads as a
    // corrupt recording rather than a config merge.
    VideoPlaybackCameraConfiguration config = make_config(*fixture_);
    config.start_frame = 1000000;
    VideoPlaybackCamera camera(config);
    EXPECT_THROW(camera.initialize(), ConfigValidationError);
}

TEST_F(VideoPlaybackCameraTest, IgnoresThePreviousRunsOutput) {
    // Everything in the file except the three camera channels is the previous run's output, which
    // replay exists to regenerate. Reading it would be wrong rather than merely wasteful.
    VideoPlaybackCamera camera(make_config(*fixture_));
    ASSERT_TRUE(camera.initialize());
    CameraData data;
    ASSERT_TRUE(camera.get(data));
    EXPECT_TRUE(data.ground_truth_poses.empty());
    EXPECT_EQ(data.rgb.header.frame_id, FrameId::CAMERA);
}

TEST_F(VideoPlaybackCameraTest, NamesTheMissingVideoChannel) {
    // A recording that carries only /camera/image starts fine and then produces nothing, which
    // reads as a broken camera rather than the wrong recording.
    auto recorder = std::make_shared<McapRecorder>("video_playback_no_video");
    const std::filesystem::path path = recorder->file_path();
    {
        OutputChannel image("/camera/image", "protobuf",
                            VizSchema::from_sdk(foxglove::schemas::CompressedImage::schema()),
                            false, nullptr, recorder);
        foxglove::schemas::CompressedImage message;
        message.frame_id = "camera";
        message.format = "jpeg";
        message.data.assign(4, std::byte{0});
        image.log_message(message, RecordingFixture::kBaseStampNs);
    }
    recorder->close();

    VideoPlaybackCameraConfiguration config;
    config.video_file_path = path.string();
    VideoPlaybackCamera camera(config);
    EXPECT_FALSE(camera.initialize());

    std::error_code ec;
    std::filesystem::remove(path, ec);
}

TEST_F(VideoPlaybackCameraTest, ReadsTheCalibrationIdOutOfTheRecording) {
    VideoPlaybackCameraConfiguration config = make_config(*fixture_);
    config.calibration_file = "";
    VideoPlaybackCamera camera(config);
    // No config/cameras/<id>.toml exists for the test id, so initialize fails on the lookup. What
    // matters is that the recording named the calibration it was rectified with.
    EXPECT_ANY_THROW(camera.initialize());
    EXPECT_EQ(camera.recorded_calibration_id(), kCalibrationId);
}
}  // namespace auto_battlebot
