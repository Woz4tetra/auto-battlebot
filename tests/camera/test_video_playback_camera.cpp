#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <filesystem>
#include <foxglove/schemas.hpp>
#include <memory>
#include <thread>

#include "../scratch_recordings.hpp"
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
constexpr double kEmbeddedFocal = 200.0;
constexpr double kFileFocal = 250.0;
constexpr std::array<double, 5> kZeroDistortion{};

cv::Mat make_index_tagged_uyvy_frame(int index) {
    cv::Mat frame(kHeight, kWidth, CV_8UC2);
    for (int row = 0; row < kHeight; ++row) {
        for (int col = 0; col < kWidth; ++col) {
            const auto luma = static_cast<uint8_t>((col + row + index * 4) % 256);
            frame.at<cv::Vec2b>(row, col) = cv::Vec2b(128, luma);
        }
    }
    return frame;
}

void submit_waiting_for_encoder(VideoEncoder &encoder, int index, uint64_t log_time_ns) {
    const uint64_t before = encoder.dropped_frames();
    encoder.submit(make_index_tagged_uyvy_frame(index), log_time_ns);
    while (encoder.dropped_frames() != before) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        const uint64_t retry = encoder.dropped_frames();
        encoder.submit(make_index_tagged_uyvy_frame(index), log_time_ns);
        if (encoder.dropped_frames() == retry) {
            break;
        }
    }
}

class RecordingWithEmbeddedAndFileCalibration {
   public:
    explicit RecordingWithEmbeddedAndFileCalibration(uint64_t video_log_offset_ns = 0) {
        calibration_path_ = (std::filesystem::temp_directory_path() /
                             "auto_battlebot_video_playback_calibration.toml")
                                .string();
        CameraCalibration calibration;
        calibration.calibration_id = kCalibrationId;
        calibration.width = kWidth;
        calibration.height = kHeight;
        calibration.fx = kEmbeddedFocal;
        calibration.fy = kEmbeddedFocal;
        calibration.distortion_k1_k2_p1_p2_k3 = kZeroDistortion;
        calibration.cx = kWidth / 2.0;
        calibration.cy = kHeight / 2.0;

        dir_ = make_scratch_recordings_dir("video_playback");
        recorder_ = std::make_shared<McapRecorder>("video_playback_test", dir_);
        recorder_->write_metadata(kCameraCalibrationMetadataKey,
                                  camera_calibration_to_toml(calibration));

        CameraCalibration revised = calibration;
        revised.fx = kFileFocal;
        revised.fy = kFileFocal;
        save_camera_calibration(calibration_path_, revised);
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
            const uint64_t stamp_ns = kBaseStampNs + static_cast<uint64_t>(i) * kFrameIntervalNs;
            FrameIdentity identity;
            identity.image_stamp_ns = stamp_ns;
            identity.video_frame_index = i;
            meta.log(foxglove_adapters::to_frame_meta_json(identity), stamp_ns);
            submit_waiting_for_encoder(encoder, i, stamp_ns + video_log_offset_ns);
        }
        encoder.stop();
        recorder_->close();
    }

    ~RecordingWithEmbeddedAndFileCalibration() {
        std::error_code ec;
        std::filesystem::remove(path_, ec);
        std::filesystem::remove(calibration_path_, ec);
        std::filesystem::remove_all(dir_, ec);
    }

    bool started() const { return started_; }
    const std::filesystem::path &path() const { return path_; }
    const std::string &calibration_path() const { return calibration_path_; }

    static constexpr uint64_t kBaseStampNs = 1788011445339499712ULL;
    static constexpr uint64_t kFrameIntervalNs = 33333333ULL;

   private:
    std::shared_ptr<McapRecorder> recorder_;
    std::filesystem::path path_;
    std::string calibration_path_;
    std::filesystem::path dir_;
    bool started_ = false;
};

VideoPlaybackCameraConfiguration make_config(
    const RecordingWithEmbeddedAndFileCalibration &fixture) {
    VideoPlaybackCameraConfiguration config;
    config.video_file_path = fixture.path().string();
    config.calibration_file = fixture.calibration_path();
    config.real_time_mode = false;
    config.rebase_stamps = false;
    return config;
}
}  // namespace

class VideoPlaybackCameraTest : public ::testing::Test {
   protected:
    static void SetUpTestSuite() {
        fixture_ = std::make_unique<RecordingWithEmbeddedAndFileCalibration>();
    }
    static void TearDownTestSuite() { fixture_.reset(); }

    void SetUp() override {
        if (!fixture_ || !fixture_->started()) {
            GTEST_SKIP() << "No H.264 encoder available to build the test recording";
        }
    }

    static std::unique_ptr<RecordingWithEmbeddedAndFileCalibration> fixture_;
};

std::unique_ptr<RecordingWithEmbeddedAndFileCalibration> VideoPlaybackCameraTest::fixture_;

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

    EXPECT_GT(data.tf_visodom_from_camera.header.stamp, 0.0);
    EXPECT_EQ(data.tf_visodom_from_camera.header.frame_id, FrameId::VISUAL_ODOMETRY);
    EXPECT_EQ(data.tf_visodom_from_camera.child_frame_id, FrameId::CAMERA);
    EXPECT_EQ(data.tf_visodom_from_camera.transform.tf.rows(), 4);
    EXPECT_TRUE(data.tf_visodom_from_camera.transform.tf.isIdentity(1e-12))
        << "video playback has no odometry and the camera is fixed";
    EXPECT_TRUE(data.pose_tracking_ready);
}

TEST_F(VideoPlaybackCameraTest, ProducesNoDepth) {
    VideoPlaybackCamera camera(make_config(*fixture_));
    ASSERT_TRUE(camera.initialize());
    CameraData data;
    ASSERT_TRUE(camera.get(data));
    EXPECT_TRUE(data.depth.image.empty());
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

TEST_F(VideoPlaybackCameraTest, SuccessiveGrabsDoNotShareBuffers) {
    VideoPlaybackCamera camera(make_config(*fixture_));
    ASSERT_TRUE(camera.initialize());

    CameraData first;
    ASSERT_TRUE(camera.get(first));
    const cv::Mat first_snapshot = first.rgb.image.clone();
    CameraData second;
    ASSERT_TRUE(camera.get(second));

    EXPECT_NE(first.rgb.image.data, second.rgb.image.data);
    EXPECT_EQ(cv::norm(first.rgb.image, first_snapshot, cv::NORM_INF), 0.0);
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
    EXPECT_EQ(frames, kFrames);
    EXPECT_EQ(camera.recorded_calibration_id(), kCalibrationId);
}

TEST_F(VideoPlaybackCameraTest, RectifiesWithTheEmbeddedCalibration) {
    VideoPlaybackCameraConfiguration config = make_config(*fixture_);
    config.calibration_file = "";
    VideoPlaybackCamera camera(config);
    ASSERT_TRUE(camera.initialize());

    CameraData data;
    ASSERT_TRUE(camera.get(data));
    EXPECT_EQ(camera.recorded_calibration_id(), kCalibrationId);
    EXPECT_NEAR(data.camera_info.intrinsics.at<double>(0, 0), kEmbeddedFocal, 1.0)
        << "zero distortion keeps the recorded focal length";
}

TEST_F(VideoPlaybackCameraTest, ConfiguredCalibrationFileOverridesTheEmbeddedOne) {
    VideoPlaybackCamera camera(make_config(*fixture_));
    ASSERT_TRUE(camera.initialize());

    CameraData data;
    ASSERT_TRUE(camera.get(data));
    EXPECT_NEAR(data.camera_info.intrinsics.at<double>(0, 0), kFileFocal, 1.0);
}

TEST(CameraCalibrationTest, TomlTextRoundTrips) {
    CameraCalibration written;
    written.calibration_id = "round_trip";
    written.width = 1920;
    written.height = 1200;
    written.fx = 1006.123456789;
    written.fy = 1005.5;
    written.cx = 961.25;
    written.cy = 599.75;
    written.distortion_k1_k2_p1_p2_k3 = {-0.31, 0.12, 0.0004, -0.0002, -0.021};

    const CameraCalibration read =
        parse_camera_calibration(camera_calibration_to_toml(written), "round trip");
    EXPECT_EQ(read.calibration_id, written.calibration_id);
    EXPECT_EQ(read.width, written.width);
    EXPECT_EQ(read.height, written.height);
    EXPECT_NEAR(read.fx, written.fx, 1e-8);
    EXPECT_NEAR(read.fy, written.fy, 1e-8);
    EXPECT_NEAR(read.cx, written.cx, 1e-8);
    EXPECT_NEAR(read.cy, written.cy, 1e-8);
    for (size_t i = 0; i < written.distortion_k1_k2_p1_p2_k3.size(); ++i) {
        EXPECT_NEAR(read.distortion_k1_k2_p1_p2_k3[i], written.distortion_k1_k2_p1_p2_k3[i], 1e-8);
    }
}

TEST(CameraCalibrationTest, ParseNamesTheSourceOfAMissingField) {
    EXPECT_THROW(
        {
            try {
                parse_camera_calibration("calibration_id = \"x\"\nwidth = 10\nheight = 10\n",
                                         "some.mcap metadata");
            } catch (const ConfigValidationError &error) {
                EXPECT_NE(std::string(error.what()).find("some.mcap metadata"), std::string::npos);
                throw;
            }
        },
        ConfigValidationError);
}

TEST(VideoPlaybackCameraJoinTest, ImageStampJoinsFrameMetaOnOrdinalNotLogTime) {
    constexpr uint64_t kVideoLogOffsetNs = 5000000000ULL;
    RecordingWithEmbeddedAndFileCalibration fixture(kVideoLogOffsetNs);
    if (!fixture.started()) GTEST_SKIP() << "No H.264 encoder available to build the recording";
    VideoPlaybackCamera camera(make_config(fixture));
    ASSERT_TRUE(camera.initialize());

    CameraData data;
    int frames = 0;
    while (frames < 10 && camera.get(data)) {
        const int64_t ordinal = data.frame_identity.video_frame_index;
        EXPECT_EQ(data.frame_identity.image_stamp_ns,
                  RecordingWithEmbeddedAndFileCalibration::kBaseStampNs +
                      static_cast<uint64_t>(ordinal) *
                          RecordingWithEmbeddedAndFileCalibration::kFrameIntervalNs)
            << "frame " << ordinal << " took the video log time instead of its frame_meta stamp";
        ++frames;
    }
    EXPECT_EQ(frames, 10);
}

TEST_F(VideoPlaybackCameraTest, PacingReturnsEachFrameOnTheRecordedScheduleNotAPeriodAfterTheWork) {
    constexpr int kPlayed = 15;
    constexpr double kPeriodMs = 33.333333;
    VideoPlaybackCameraConfiguration config = make_config(*fixture_);
    config.real_time_mode = true;
    VideoPlaybackCamera camera(config);
    ASSERT_TRUE(camera.initialize());

    CameraData data;
    ASSERT_TRUE(camera.get(data));
    const auto begin = std::chrono::steady_clock::now();
    for (int i = 1; i < kPlayed; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(i % 2 == 0 ? 5 : 20));
        ASSERT_TRUE(camera.get(data));
        const double returned_ms =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin)
                .count();
        EXPECT_NEAR(returned_ms, i * kPeriodMs, 6.0) << "frame " << i;
    }
}

TEST_F(VideoPlaybackCameraTest,
       PacingKeepsTheAverageRateWhenPipelineWorkVariesAndSometimesOverruns) {
    constexpr int kPlayed = 15;
    constexpr double kPeriodMs = 33.333333;
    VideoPlaybackCameraConfiguration config = make_config(*fixture_);
    config.real_time_mode = true;
    VideoPlaybackCamera camera(config);
    ASSERT_TRUE(camera.initialize());

    CameraData data;
    ASSERT_TRUE(camera.get(data));
    const auto begin = std::chrono::steady_clock::now();
    double halfway_ms = 0.0;
    for (int i = 1; i < kPlayed; ++i) {
        const bool overrun = i % 4 == 1;
        std::this_thread::sleep_for(std::chrono::milliseconds(overrun ? 45 : 5));
        ASSERT_TRUE(camera.get(data));
        if (i == 7) {
            halfway_ms =
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin)
                    .count();
        }
    }
    const double elapsed_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();
    EXPECT_NEAR(elapsed_ms, (kPlayed - 1) * kPeriodMs, 70.0);
    EXPECT_NEAR(halfway_ms, 7 * kPeriodMs, 70.0);
}

TEST_F(VideoPlaybackCameraTest, PacingHoldsTheRecordedRateWhilePipelineWorkRuns) {
    constexpr int kPlayed = 15;
    constexpr auto kPipelineWork = std::chrono::milliseconds(25);
    VideoPlaybackCameraConfiguration config = make_config(*fixture_);
    config.real_time_mode = true;
    VideoPlaybackCamera camera(config);
    ASSERT_TRUE(camera.initialize());

    CameraData data;
    ASSERT_TRUE(camera.get(data));
    const auto begin = std::chrono::steady_clock::now();
    for (int i = 1; i < kPlayed; ++i) {
        std::this_thread::sleep_for(kPipelineWork);
        ASSERT_TRUE(camera.get(data));
    }
    const double elapsed_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();
    const double recorded_ms =
        (kPlayed - 1) *
        static_cast<double>(RecordingWithEmbeddedAndFileCalibration::kFrameIntervalNs) * 1e-6;
    EXPECT_GT(elapsed_ms, recorded_ms * 0.95);
    EXPECT_LT(elapsed_ms, recorded_ms * 1.2)
        << "pipeline work of " << kPipelineWork.count() << " ms per frame stretched the replay";
}

TEST_F(VideoPlaybackCameraTest, StartsAtTheRequestedFrameNotThePrecedingKeyframe) {
    VideoPlaybackCameraConfiguration config = make_config(*fixture_);
    config.start_frame = 20;
    VideoPlaybackCamera camera(config);
    ASSERT_TRUE(camera.initialize());

    CameraData data;
    ASSERT_TRUE(camera.get(data));
    EXPECT_EQ(data.frame_identity.video_frame_index, 20);
}

TEST_F(VideoPlaybackCameraTest, SeekStartsFromPrecedingKeyframe) {
    constexpr int kStartFrame = 20;
    VideoPlaybackCameraConfiguration seeked_config = make_config(*fixture_);
    seeked_config.start_frame = kStartFrame;
    VideoPlaybackCamera seeked(seeked_config);
    ASSERT_TRUE(seeked.initialize());
    CameraData from_seek;
    ASSERT_TRUE(seeked.get(from_seek));
    ASSERT_EQ(from_seek.frame_identity.video_frame_index, kStartFrame);

    VideoPlaybackCamera sequential(make_config(*fixture_));
    ASSERT_TRUE(sequential.initialize());
    CameraData from_start;
    do {
        ASSERT_TRUE(sequential.get(from_start));
    } while (from_start.frame_identity.video_frame_index < kStartFrame);
    ASSERT_EQ(from_start.frame_identity.video_frame_index, kStartFrame);

    EXPECT_EQ(cv::norm(from_seek.rgb.image, from_start.rgb.image, cv::NORM_INF), 0.0)
        << "a seek into the middle of a GOP decodes smeared frames";
}

TEST_F(VideoPlaybackCameraTest, RejectsAStartFrameBeyondTheEndOfTheFile) {
    VideoPlaybackCameraConfiguration config = make_config(*fixture_);
    config.start_frame = 1000000;
    VideoPlaybackCamera camera(config);
    EXPECT_THROW(camera.initialize(), ConfigValidationError);
}

TEST_F(VideoPlaybackCameraTest, IgnoresThePreviousRunsOutput) {
    VideoPlaybackCamera camera(make_config(*fixture_));
    ASSERT_TRUE(camera.initialize());
    CameraData data;
    ASSERT_TRUE(camera.get(data));
    EXPECT_FALSE(data.ground_truth.has_value());
    EXPECT_EQ(data.rgb.header.frame_id, FrameId::CAMERA);
}

TEST_F(VideoPlaybackCameraTest, NamesTheMissingVideoChannel) {
    const std::filesystem::path dir = make_scratch_recordings_dir("video_playback_no_video");
    auto recorder = std::make_shared<McapRecorder>("video_playback_no_video", dir);
    const std::filesystem::path path = recorder->file_path();
    {
        OutputChannel image("/camera/image", "protobuf",
                            VizSchema::from_sdk(foxglove::schemas::CompressedImage::schema()),
                            false, nullptr, recorder);
        foxglove::schemas::CompressedImage message;
        message.frame_id = "camera";
        message.format = "jpeg";
        message.data.assign(4, std::byte{0});
        image.log_message(message, RecordingWithEmbeddedAndFileCalibration::kBaseStampNs);
    }
    recorder->close();

    VideoPlaybackCameraConfiguration config;
    config.video_file_path = path.string();
    VideoPlaybackCamera camera(config);
    EXPECT_FALSE(camera.initialize());

    std::error_code ec;
    std::filesystem::remove_all(dir, ec);
}
}  // namespace auto_battlebot
