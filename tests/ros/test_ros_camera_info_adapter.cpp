#include <gtest/gtest.h>
#include "ros/ros_message_adapters/ros_camera_info.hpp"

namespace auto_battlebot
{
    class RosCameraInfoAdapterTest : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            test_camera_info.header.stamp = 123.456;
            test_camera_info.header.frame_id = FrameId::CAMERA;

            // Create 3x3 intrinsics matrix
            test_camera_info.width = 640;
            test_camera_info.height = 480;
            test_camera_info.intrinsics = cv::Mat::eye(3, 3, CV_64F);
            test_camera_info.intrinsics.at<double>(0, 0) = 500.0; // fx
            test_camera_info.intrinsics.at<double>(1, 1) = 500.0; // fy
            test_camera_info.intrinsics.at<double>(0, 2) = 320.0; // cx
            test_camera_info.intrinsics.at<double>(1, 2) = 240.0; // cy

            // Create distortion coefficients (5 coefficients)
            test_camera_info.distortion = cv::Mat(5, 1, CV_64F);
            test_camera_info.distortion.at<double>(0) = 0.1;
            test_camera_info.distortion.at<double>(1) = -0.2;
            test_camera_info.distortion.at<double>(2) = 0.01;
            test_camera_info.distortion.at<double>(3) = -0.01;
            test_camera_info.distortion.at<double>(4) = 0.05;
        }

        CameraInfo test_camera_info;
    };

    // Test basic conversion
    TEST_F(RosCameraInfoAdapterTest, BasicConversion)
    {
        auto ros_camera_info = ros_adapters::to_ros_camera_info(test_camera_info);

        EXPECT_EQ(ros_camera_info.height, 480);
        EXPECT_EQ(ros_camera_info.width, 640);
        EXPECT_EQ(ros_camera_info.distortion_model, "plumb_bob");
    }

    // Test intrinsics matrix (K)
    TEST_F(RosCameraInfoAdapterTest, IntrinsicsMatrix)
    {
        auto ros_camera_info = ros_adapters::to_ros_camera_info(test_camera_info);

        EXPECT_DOUBLE_EQ(ros_camera_info.K[0], 500.0); // fx
        EXPECT_DOUBLE_EQ(ros_camera_info.K[1], 0.0);
        EXPECT_DOUBLE_EQ(ros_camera_info.K[2], 320.0); // cx
        EXPECT_DOUBLE_EQ(ros_camera_info.K[3], 0.0);
        EXPECT_DOUBLE_EQ(ros_camera_info.K[4], 500.0); // fy
        EXPECT_DOUBLE_EQ(ros_camera_info.K[5], 240.0); // cy
        EXPECT_DOUBLE_EQ(ros_camera_info.K[6], 0.0);
        EXPECT_DOUBLE_EQ(ros_camera_info.K[7], 0.0);
        EXPECT_DOUBLE_EQ(ros_camera_info.K[8], 1.0);
    }

    // Test distortion coefficients
    TEST_F(RosCameraInfoAdapterTest, DistortionCoefficients)
    {
        auto ros_camera_info = ros_adapters::to_ros_camera_info(test_camera_info);

        EXPECT_EQ(ros_camera_info.D.size(), 5);
        EXPECT_DOUBLE_EQ(ros_camera_info.D[0], 0.1);
        EXPECT_DOUBLE_EQ(ros_camera_info.D[1], -0.2);
        EXPECT_DOUBLE_EQ(ros_camera_info.D[2], 0.01);
        EXPECT_DOUBLE_EQ(ros_camera_info.D[3], -0.01);
        EXPECT_DOUBLE_EQ(ros_camera_info.D[4], 0.05);
    }

    // Test rectification matrix (R) - should be identity
    TEST_F(RosCameraInfoAdapterTest, RectificationMatrix)
    {
        auto ros_camera_info = ros_adapters::to_ros_camera_info(test_camera_info);

        // R should be identity
        EXPECT_DOUBLE_EQ(ros_camera_info.R[0], 1.0);
        EXPECT_DOUBLE_EQ(ros_camera_info.R[1], 0.0);
        EXPECT_DOUBLE_EQ(ros_camera_info.R[2], 0.0);
        EXPECT_DOUBLE_EQ(ros_camera_info.R[3], 0.0);
        EXPECT_DOUBLE_EQ(ros_camera_info.R[4], 1.0);
        EXPECT_DOUBLE_EQ(ros_camera_info.R[5], 0.0);
        EXPECT_DOUBLE_EQ(ros_camera_info.R[6], 0.0);
        EXPECT_DOUBLE_EQ(ros_camera_info.R[7], 0.0);
        EXPECT_DOUBLE_EQ(ros_camera_info.R[8], 1.0);
    }

    // Test projection matrix (P) - should be [K | 0]
    TEST_F(RosCameraInfoAdapterTest, ProjectionMatrix)
    {
        auto ros_camera_info = ros_adapters::to_ros_camera_info(test_camera_info);

        // First 3x3 should match K
        EXPECT_DOUBLE_EQ(ros_camera_info.P[0], 500.0);
        EXPECT_DOUBLE_EQ(ros_camera_info.P[1], 0.0);
        EXPECT_DOUBLE_EQ(ros_camera_info.P[2], 320.0);
        EXPECT_DOUBLE_EQ(ros_camera_info.P[3], 0.0); // Fourth column
        EXPECT_DOUBLE_EQ(ros_camera_info.P[4], 0.0);
        EXPECT_DOUBLE_EQ(ros_camera_info.P[5], 500.0);
        EXPECT_DOUBLE_EQ(ros_camera_info.P[6], 240.0);
        EXPECT_DOUBLE_EQ(ros_camera_info.P[7], 0.0); // Fourth column
        EXPECT_DOUBLE_EQ(ros_camera_info.P[8], 0.0);
        EXPECT_DOUBLE_EQ(ros_camera_info.P[9], 0.0);
        EXPECT_DOUBLE_EQ(ros_camera_info.P[10], 1.0);
        EXPECT_DOUBLE_EQ(ros_camera_info.P[11], 0.0); // Fourth column
    }

    // Test with empty distortion
    TEST_F(RosCameraInfoAdapterTest, EmptyDistortion)
    {
        test_camera_info.distortion = cv::Mat();

        auto ros_camera_info = ros_adapters::to_ros_camera_info(test_camera_info);

        EXPECT_EQ(ros_camera_info.D.size(), 0);
    }

    // Test header
    TEST_F(RosCameraInfoAdapterTest, HeaderInfo)
    {
        auto ros_camera_info = ros_adapters::to_ros_camera_info(test_camera_info);

        EXPECT_DOUBLE_EQ(ros_camera_info.header.stamp.toSec(), 123.456);
        EXPECT_EQ(ros_camera_info.header.frame_id, "camera");
    }

} // namespace auto_battlebot
