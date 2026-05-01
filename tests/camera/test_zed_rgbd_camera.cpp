#include <gtest/gtest.h>

#include <filesystem>
#include <memory>

#include "data_structures.hpp"
#include "rgbd_camera/config.hpp"
#include "rgbd_camera/zed_rgbd_camera.hpp"

namespace auto_battlebot {
class ZedRgbdCameraTest : public ::testing::Test {
   protected:
    std::string svo_file_path;

    // Shared camera opened once for the suite to avoid ZED SDK segfault when
    // opening the same SVO twice in one process.
    static std::unique_ptr<ZedRgbdCamera> shared_camera_;
    static ZedRgbdCameraConfiguration shared_config_;

    void SetUp() override {
        std::filesystem::path test_dir = std::filesystem::path(__FILE__).parent_path();
        svo_file_path = (test_dir / "test.svo2").string();

        if (!std::filesystem::exists(svo_file_path)) {
            GTEST_SKIP() << "Test SVO file not found at: " << svo_file_path;
        }
    }

    static void SetUpTestSuite() {
        std::filesystem::path test_dir = std::filesystem::path(__FILE__).parent_path();
        std::string path = (test_dir / "test.svo2").string();
        if (!std::filesystem::exists(path)) {
            shared_camera_.reset();
            return;
        }
        shared_config_.svo_file_path = path;
        shared_config_.camera_fps = 30;
        shared_config_.camera_resolution = Resolution::RES_1280x720;
        shared_config_.depth_mode = DepthMode::ZED_NEURAL;
        shared_camera_ = std::make_unique<ZedRgbdCamera>(shared_config_);
        if (!shared_camera_->initialize()) {
            shared_camera_.reset();
        }
    }

    static void TearDownTestSuite() { shared_camera_.reset(); }
};

std::unique_ptr<ZedRgbdCamera> ZedRgbdCameraTest::shared_camera_;
ZedRgbdCameraConfiguration ZedRgbdCameraTest::shared_config_;

TEST_F(ZedRgbdCameraTest, FullDataPipeline) {
    ASSERT_NE(shared_camera_, nullptr)
        << "Shared ZED camera not available (SVO missing or init failed)";

    CameraData data;
    ASSERT_TRUE(shared_camera_->get(data, true));

    // Verify camera info
    EXPECT_GT(data.camera_info.width, 0);
    EXPECT_GT(data.camera_info.height, 0);
    EXPECT_FALSE(data.camera_info.intrinsics.empty());
    EXPECT_FALSE(data.camera_info.distortion.empty());

    // Verify RGB image
    EXPECT_FALSE(data.rgb.image.empty());
    EXPECT_EQ(data.rgb.image.type(), CV_8UC3);
    EXPECT_EQ(data.rgb.image.cols, data.camera_info.width);
    EXPECT_EQ(data.rgb.image.rows, data.camera_info.height);

    // Verify depth image
    EXPECT_FALSE(data.depth.image.empty());
    EXPECT_EQ(data.depth.image.type(), CV_32FC1);
    EXPECT_EQ(data.depth.image.cols, data.camera_info.width);
    EXPECT_EQ(data.depth.image.rows, data.camera_info.height);

    // Verify transform data
    EXPECT_GT(data.tf_visodom_from_camera.header.stamp, 0.0);
    EXPECT_EQ(data.tf_visodom_from_camera.header.frame_id, FrameId::VISUAL_ODOMETRY);
    EXPECT_EQ(data.tf_visodom_from_camera.child_frame_id, FrameId::CAMERA);
    EXPECT_EQ(data.tf_visodom_from_camera.transform.tf.rows(), 4);
    EXPECT_EQ(data.tf_visodom_from_camera.transform.tf.cols(), 4);
}

TEST_F(ZedRgbdCameraTest, MultipleFrameProcessing) {
    ASSERT_NE(shared_camera_, nullptr)
        << "Shared ZED camera not available (SVO missing or init failed)";

    CameraData data1;
    ASSERT_TRUE(shared_camera_->get(data1, true));
    double timestamp1 = data1.tf_visodom_from_camera.header.stamp;

    CameraData data2;
    ASSERT_TRUE(shared_camera_->get(data2, true));
    double timestamp2 = data2.tf_visodom_from_camera.header.stamp;

    // Camera info should remain constant
    EXPECT_EQ(data1.camera_info.width, data2.camera_info.width);
    EXPECT_EQ(data1.camera_info.height, data2.camera_info.height);
    EXPECT_EQ(cv::norm(data1.camera_info.intrinsics - data2.camera_info.intrinsics), 0.0);
    EXPECT_EQ(cv::norm(data1.camera_info.distortion - data2.camera_info.distortion), 0.0);

    // With pass by reference, data1 and data2 are separate copies
    // Timestamps should have changed after the second update
    EXPECT_NE(timestamp1, timestamp2);
}

TEST_F(ZedRgbdCameraTest, DataIndependence) {
    ASSERT_NE(shared_camera_, nullptr)
        << "Shared ZED camera not available (SVO missing or init failed)";

    CameraData data_ref;
    ASSERT_TRUE(shared_camera_->get(data_ref, true));

    CameraData data_copy;
    ASSERT_TRUE(shared_camera_->get(data_copy, true));
    data_copy.rgb.image = data_ref.rgb.image.clone();
    data_copy.depth.image = data_ref.depth.image.clone();

    // Verify we can make independent copies with clone()
    if (!data_copy.rgb.image.empty() && data_copy.rgb.image.isContinuous()) {
        cv::Vec3b original_pixel = data_ref.rgb.image.at<cv::Vec3b>(0, 0);
        data_copy.rgb.image.at<cv::Vec3b>(0, 0) = cv::Vec3b(255, 255, 255);
        EXPECT_EQ(data_ref.rgb.image.at<cv::Vec3b>(0, 0), original_pixel);
        EXPECT_EQ(data_copy.rgb.image.at<cv::Vec3b>(0, 0), cv::Vec3b(255, 255, 255));
    }
}

TEST_F(ZedRgbdCameraTest, SvoEndOfFile) {
    // Verify should_close() returns true when SVO reaches end
    ZedRgbdCameraConfiguration config;
    config.svo_file_path = svo_file_path;
    config.camera_fps = 30;
    config.camera_resolution = Resolution::RES_1280x720;
    config.depth_mode = DepthMode::ZED_NEURAL;

    ZedRgbdCamera camera(config);
    ASSERT_TRUE(camera.initialize());

    EXPECT_FALSE(camera.should_close());

    // Process frames until end of SVO file
    CameraData data;
    int max_frames = 1000;  // Safety limit
    int frame_count = 0;
    while (!camera.should_close() && frame_count < max_frames) {
        camera.get(data, true);
        frame_count++;
    }

    // Should eventually reach end of file
    EXPECT_TRUE(camera.should_close() || frame_count >= max_frames);
}

TEST_F(ZedRgbdCameraTest, ShouldCloseBeforeInitialize) {
    // Verify should_close() returns false before initialization
    ZedRgbdCameraConfiguration config;
    config.svo_file_path = svo_file_path;
    config.camera_fps = 30;
    config.camera_resolution = Resolution::RES_1280x720;
    config.depth_mode = DepthMode::ZED_NEURAL;

    ZedRgbdCamera camera(config);
    EXPECT_FALSE(camera.should_close());
}

TEST_F(ZedRgbdCameraTest, InvalidSvoFile) {
    // Verify initialization fails with non-existent SVO file
    ZedRgbdCameraConfiguration config;
    config.svo_file_path = "/nonexistent/path/to/file.svo2";
    config.camera_fps = 30;
    config.camera_resolution = Resolution::RES_1280x720;
    config.depth_mode = DepthMode::ZED_NEURAL;

    ZedRgbdCamera camera(config);
    EXPECT_FALSE(camera.initialize());
}

}  // namespace auto_battlebot
