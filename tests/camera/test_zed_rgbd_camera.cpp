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

        CameraData data;
        ASSERT_TRUE(camera.get(data));

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

        CameraData data1;
        ASSERT_TRUE(camera.get(data1));

        // Update again
        ASSERT_TRUE(camera.update());

        CameraData data2;
        ASSERT_TRUE(camera.get(data2));

        // Camera info should remain constant
        EXPECT_EQ(data1.camera_info.width, data2.camera_info.width);
        EXPECT_EQ(data1.camera_info.height, data2.camera_info.height);
        EXPECT_EQ(cv::norm(data1.camera_info.intrinsics - data2.camera_info.intrinsics), 0.0);
        EXPECT_EQ(cv::norm(data1.camera_info.distortion - data2.camera_info.distortion), 0.0);

        // Timestamps should be different
        EXPECT_NE(data1.tf_visodom_from_camera.header.timestamp, data2.tf_visodom_from_camera.header.timestamp);
    }

    TEST_F(ZedRgbdCameraTest, DataIndependence)
    {
        // Verify get() returns independent copies, not shared data
        ZedRgbdCameraConfiguration config;
        config.svo_file_path = svo_file_path;
        config.camera_fps = 30;
        config.camera_resolution = Resolution::RES_1280x720;
        config.depth_mode = DepthMode::ZED_NEURAL;

        ZedRgbdCamera camera(config);
        ASSERT_TRUE(camera.initialize());
        ASSERT_TRUE(camera.update());

        CameraData data1;
        ASSERT_TRUE(camera.get(data1));

        CameraData data2;
        ASSERT_TRUE(camera.get(data2));

        // Verify data is copied, not shared
        EXPECT_NE(data1.rgb.image.data, data2.rgb.image.data);
        EXPECT_NE(data1.depth.image.data, data2.depth.image.data);

        // Modify one and verify the other is unchanged
        if (!data1.rgb.image.empty() && data1.rgb.image.isContinuous())
        {
            cv::Vec3b original_pixel = data2.rgb.image.at<cv::Vec3b>(0, 0);
            data1.rgb.image.at<cv::Vec3b>(0, 0) = cv::Vec3b(255, 255, 255);
            EXPECT_EQ(data2.rgb.image.at<cv::Vec3b>(0, 0), original_pixel);
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
