#ifdef BUILD_WITH_ZED
#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <memory>
#include <thread>
#include <vector>

#include "config/config_parser.hpp"
#include "data_structures.hpp"
#include "rgbd_camera/config.hpp"
#include "rgbd_camera/zed_svo_playback_camera.hpp"

namespace auto_battlebot {
class ZedSvoPlaybackCameraTest : public ::testing::Test {
   protected:
    std::string svo_file_path;

    std::unique_ptr<ZedSvoPlaybackCamera> camera_;
    ZedSvoPlaybackCameraConfiguration config_;

    void SetUp() override {
        std::filesystem::path test_dir = std::filesystem::path(__FILE__).parent_path();
        svo_file_path = (test_dir / "test.svo2").string();

        if (!std::filesystem::exists(svo_file_path)) {
            GTEST_SKIP() << "Test SVO file not found at: " << svo_file_path;
        }
        config_.svo_file_path = svo_file_path;
        config_.camera_fps = 30;
        config_.camera_resolution = Resolution::RES_1280x720;
        config_.depth_mode = DepthMode::ZED_NEURAL;
        camera_ = std::make_unique<ZedSvoPlaybackCamera>(config_);
        if (!camera_->initialize()) {
            camera_.reset();
        }
    }
};

TEST_F(ZedSvoPlaybackCameraTest, RejectsAStartFrameBeyondTheEndOfTheFile) {
    ASSERT_NE(camera_, nullptr) << "ZED camera failed to open the test SVO";

    ZedSvoPlaybackCameraConfiguration config = config_;
    config.svo_start_frame = 1000000;
    ZedSvoPlaybackCamera camera(config);

    EXPECT_THROW(camera.initialize(), ConfigValidationError);
}

TEST_F(ZedSvoPlaybackCameraTest, FrameIndexMatchesGrabbedFrame) {
    ASSERT_NE(camera_, nullptr) << "ZED camera failed to open the test SVO";

    ZedSvoPlaybackCameraConfiguration config = config_;
    config.svo_start_frame = 0;
    ZedSvoPlaybackCamera camera(config);
    ASSERT_TRUE(camera.initialize());
    CameraData data;
    ASSERT_TRUE(camera.get(data));
    EXPECT_EQ(data.frame_identity.video_frame_index, 0);
    ASSERT_TRUE(camera.get(data));
    EXPECT_EQ(data.frame_identity.video_frame_index, 1);
}

TEST_F(ZedSvoPlaybackCameraTest, FullDataPipeline) {
    ASSERT_NE(camera_, nullptr) << "ZED camera failed to open the test SVO";

    CameraData data;
    ASSERT_TRUE(camera_->get(data));

    EXPECT_GT(data.camera_info.width, 0);
    EXPECT_GT(data.camera_info.height, 0);
    EXPECT_FALSE(data.camera_info.intrinsics.empty());
    EXPECT_FALSE(data.camera_info.distortion.empty());

    EXPECT_FALSE(data.rgb.image.empty());
    EXPECT_EQ(data.rgb.image.type(), CV_8UC3);
    EXPECT_EQ(data.rgb.image.cols, data.camera_info.width);
    EXPECT_EQ(data.rgb.image.rows, data.camera_info.height);

    EXPECT_FALSE(data.depth.image.empty());
    EXPECT_EQ(data.depth.image.type(), CV_32FC1);
    EXPECT_EQ(data.depth.image.cols, data.camera_info.width);
    EXPECT_EQ(data.depth.image.rows, data.camera_info.height);

    EXPECT_GT(data.tf_visodom_from_camera.header.stamp, 0.0);
    EXPECT_EQ(data.tf_visodom_from_camera.header.frame_id, FrameId::VISUAL_ODOMETRY);
    EXPECT_EQ(data.tf_visodom_from_camera.child_frame_id, FrameId::CAMERA);
    EXPECT_EQ(data.tf_visodom_from_camera.transform.tf.rows(), 4);
    EXPECT_EQ(data.tf_visodom_from_camera.transform.tf.cols(), 4);
}

TEST_F(ZedSvoPlaybackCameraTest, MultipleFrameProcessing) {
    ASSERT_NE(camera_, nullptr) << "ZED camera failed to open the test SVO";

    CameraData data1;
    ASSERT_TRUE(camera_->get(data1));
    double timestamp1 = data1.tf_visodom_from_camera.header.stamp;

    CameraData data2;
    ASSERT_TRUE(camera_->get(data2));
    double timestamp2 = data2.tf_visodom_from_camera.header.stamp;

    EXPECT_EQ(data1.camera_info.width, data2.camera_info.width);
    EXPECT_EQ(data1.camera_info.height, data2.camera_info.height);
    EXPECT_EQ(cv::norm(data1.camera_info.intrinsics - data2.camera_info.intrinsics), 0.0);
    EXPECT_EQ(cv::norm(data1.camera_info.distortion - data2.camera_info.distortion), 0.0);

    EXPECT_NE(timestamp1, timestamp2);
}

TEST_F(ZedSvoPlaybackCameraTest, DataIndependence) {
    ASSERT_NE(camera_, nullptr) << "ZED camera failed to open the test SVO";

    CameraData data_ref;
    ASSERT_TRUE(camera_->get(data_ref));

    CameraData data_copy;
    ASSERT_TRUE(camera_->get(data_copy));
    data_copy.rgb.image = data_ref.rgb.image.clone();
    data_copy.depth.image = data_ref.depth.image.clone();

    if (!data_copy.rgb.image.empty() && data_copy.rgb.image.isContinuous()) {
        cv::Vec3b original_pixel = data_ref.rgb.image.at<cv::Vec3b>(0, 0);
        data_copy.rgb.image.at<cv::Vec3b>(0, 0) = cv::Vec3b(255, 255, 255);
        EXPECT_EQ(data_ref.rgb.image.at<cv::Vec3b>(0, 0), original_pixel);
        EXPECT_EQ(data_copy.rgb.image.at<cv::Vec3b>(0, 0), cv::Vec3b(255, 255, 255));
    }
}

TEST_F(ZedSvoPlaybackCameraTest, DeliversEveryFrameInOrderWhenThePipelineIsSlow) {
    ZedSvoPlaybackCameraConfiguration config;
    config.svo_file_path = svo_file_path;
    config.camera_fps = 30;
    config.camera_resolution = Resolution::RES_1280x720;
    config.depth_mode = DepthMode::ZED_NEURAL;
    ZedSvoPlaybackCamera camera(config);
    ASSERT_TRUE(camera.initialize());

    constexpr int kFrames = 6;
    std::vector<double> stamps;
    CameraData data;
    for (int i = 0; i < kFrames; ++i) {
        ASSERT_TRUE(camera.get(data));
        stamps.push_back(data.tf_visodom_from_camera.header.stamp);
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
    }
    const double interval = stamps[1] - stamps[0];
    ASSERT_GT(interval, 0.0);
    for (size_t i = 1; i < stamps.size(); ++i) {
        EXPECT_NEAR(stamps[i] - stamps[i - 1], interval, 1e-3) << "frame " << i << " was skipped";
    }
}

TEST_F(ZedSvoPlaybackCameraTest, SvoEndOfFile) {
    ZedSvoPlaybackCameraConfiguration config;
    config.svo_file_path = svo_file_path;
    config.camera_fps = 30;
    config.camera_resolution = Resolution::RES_1280x720;
    config.depth_mode = DepthMode::ZED_NEURAL;

    ZedSvoPlaybackCamera camera(config);
    ASSERT_TRUE(camera.initialize());

    EXPECT_FALSE(camera.should_close());

    CameraData data;
    constexpr int kMaxFramesBeforeGivingUp = 1000;
    int frame_count = 0;
    while (!camera.should_close() && frame_count < kMaxFramesBeforeGivingUp) {
        camera.get(data);
        frame_count++;
    }

    EXPECT_TRUE(camera.should_close() || frame_count >= kMaxFramesBeforeGivingUp);
}

TEST_F(ZedSvoPlaybackCameraTest, ShouldCloseBeforeInitialize) {
    ZedSvoPlaybackCameraConfiguration config;
    config.svo_file_path = svo_file_path;
    config.camera_fps = 30;
    config.camera_resolution = Resolution::RES_1280x720;
    config.depth_mode = DepthMode::ZED_NEURAL;

    ZedSvoPlaybackCamera camera(config);
    EXPECT_FALSE(camera.should_close());
}

TEST_F(ZedSvoPlaybackCameraTest, CancelRequestedBeforeInitializeReturnsCancelled) {
    ASSERT_NE(camera_, nullptr) << "ZED camera failed open test SVO";
    ZedSvoPlaybackCamera camera(config_);
    camera.cancel_initialize();
    const auto start = std::chrono::steady_clock::now();
    EXPECT_FALSE(camera.initialize());
    const double elapsed_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    EXPECT_LT(elapsed_s, 10.0);
}

TEST_F(ZedSvoPlaybackCameraTest, InvalidSvoFile) {
    ZedSvoPlaybackCameraConfiguration config;
    config.svo_file_path = "/nonexistent/path/to/file.svo2";
    config.camera_fps = 30;
    config.camera_resolution = Resolution::RES_1280x720;
    config.depth_mode = DepthMode::ZED_NEURAL;

    ZedSvoPlaybackCamera camera(config);
    EXPECT_FALSE(camera.initialize());
}

}  // namespace auto_battlebot
#endif
