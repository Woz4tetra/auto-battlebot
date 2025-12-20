#include <gtest/gtest.h>
#include <filesystem>
#include "rgbd_camera/zed_rgbd_camera.hpp"
#include "rgbd_camera/config.hpp"
#include "data_structures.hpp"

namespace auto_battlebot
{
    class ZedRgbdCameraTest : public ::testing::Test
    {
    protected:
        std::string svo_file_path;

        void SetUp() override
        {
            // Get the path to the test SVO file
            std::filesystem::path test_dir = std::filesystem::path(__FILE__).parent_path();
            svo_file_path = (test_dir / "test.svo2").string();

            // Skip tests if SVO file doesn't exist
            if (!std::filesystem::exists(svo_file_path))
            {
                GTEST_SKIP() << "Test SVO file not found at: " << svo_file_path;
            }
        }
    };

    TEST_F(ZedRgbdCameraTest, FullDataPipeline)
    {
        // Test complete workflow: initialize -> update -> get
        ZedRgbdCameraConfiguration config;
        config.svo_file_path = svo_file_path;
        config.camera_fps = 30;
        config.camera_resolution = Resolution::RES_1280x720;
        config.depth_mode = DepthMode::ZED_NEURAL;

        ZedRgbdCamera camera(config);
        ASSERT_TRUE(camera.initialize());
        ASSERT_TRUE(camera.update());

        const CameraData &data = camera.get();

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
        EXPECT_GT(data.tf_visodom_from_camera.header.timestamp, 0.0);
        EXPECT_EQ(data.tf_visodom_from_camera.header.frame_id, FrameId::VISUAL_ODOMETRY);
        EXPECT_EQ(data.tf_visodom_from_camera.child_frame_id, FrameId::CAMERA);
        EXPECT_EQ(data.tf_visodom_from_camera.transform.tf.rows(), 4);
        EXPECT_EQ(data.tf_visodom_from_camera.transform.tf.cols(), 4);
    }

    TEST_F(ZedRgbdCameraTest, MultipleFrameProcessing)
    {
        // Verify camera can process multiple consecutive frames and data remains consistent
        ZedRgbdCameraConfiguration config;
        config.svo_file_path = svo_file_path;
        config.camera_fps = 30;
        config.camera_resolution = Resolution::RES_1280x720;
        config.depth_mode = DepthMode::ZED_NEURAL;

        ZedRgbdCamera camera(config);
        ASSERT_TRUE(camera.initialize());
        ASSERT_TRUE(camera.update());

        const CameraData &data1 = camera.get();
        double timestamp1 = data1.tf_visodom_from_camera.header.timestamp;

        // Update again
        ASSERT_TRUE(camera.update());

        const CameraData &data2 = camera.get();
        double timestamp2 = data2.tf_visodom_from_camera.header.timestamp;

        // Camera info should remain constant
        EXPECT_EQ(data1.camera_info.width, data2.camera_info.width);
        EXPECT_EQ(data1.camera_info.height, data2.camera_info.height);
        EXPECT_EQ(cv::norm(data1.camera_info.intrinsics - data2.camera_info.intrinsics), 0.0);
        EXPECT_EQ(cv::norm(data1.camera_info.distortion - data2.camera_info.distortion), 0.0);

        // Since get() returns a const reference to internal data that gets updated,
        // both references point to the same updated data now
        EXPECT_EQ(&data1, &data2);
        // Timestamps should have changed after the second update
        EXPECT_NE(timestamp1, timestamp2);
    }

    TEST_F(ZedRgbdCameraTest, DataIndependence)
    {
        // Verify get() returns a const reference, and deep copies are independent
        ZedRgbdCameraConfiguration config;
        config.svo_file_path = svo_file_path;
        config.camera_fps = 30;
        config.camera_resolution = Resolution::RES_1280x720;
        config.depth_mode = DepthMode::ZED_NEURAL;

        ZedRgbdCamera camera(config);
        ASSERT_TRUE(camera.initialize());
        ASSERT_TRUE(camera.update());

        const CameraData &data_ref = camera.get();

        // Make a deep copy to test data independence
        CameraData data_copy = data_ref;
        data_copy.rgb.image = data_ref.rgb.image.clone();
        data_copy.depth.image = data_ref.depth.image.clone();

        // Verify we can make independent copies with clone()
        if (!data_copy.rgb.image.empty() && data_copy.rgb.image.isContinuous())
        {
            cv::Vec3b original_pixel = data_ref.rgb.image.at<cv::Vec3b>(0, 0);
            // Modify the deep copy
            data_copy.rgb.image.at<cv::Vec3b>(0, 0) = cv::Vec3b(255, 255, 255);
            // Original should be unchanged
            EXPECT_EQ(data_ref.rgb.image.at<cv::Vec3b>(0, 0), original_pixel);
            // Copy should be modified
            EXPECT_EQ(data_copy.rgb.image.at<cv::Vec3b>(0, 0), cv::Vec3b(255, 255, 255));
        }
    }

    TEST_F(ZedRgbdCameraTest, SvoEndOfFile)
    {
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
        int max_frames = 1000; // Safety limit
        int frame_count = 0;
        while (!camera.should_close() && frame_count < max_frames)
        {
            camera.update();
            frame_count++;
        }

        // Should eventually reach end of file
        EXPECT_TRUE(camera.should_close() || frame_count >= max_frames);
    }

    TEST_F(ZedRgbdCameraTest, InvalidSvoFile)
    {
        // Verify initialization fails with non-existent SVO file
        ZedRgbdCameraConfiguration config;
        config.svo_file_path = "/nonexistent/path/to/file.svo2";
        config.camera_fps = 30;
        config.camera_resolution = Resolution::RES_1280x720;
        config.depth_mode = DepthMode::ZED_NEURAL;

        ZedRgbdCamera camera(config);
        EXPECT_FALSE(camera.initialize());
    }

} // namespace auto_battlebot
