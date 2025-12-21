#include <gtest/gtest.h>
#include "ros/ros_message_adapters/ros_image.hpp"
#include <sensor_msgs/image_encodings.h>

namespace auto_battlebot
{
    class RosImageAdapterTest : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            // Create test RGB image (3x3 BGR image)
            test_rgb.header.stamp = 123.456;
            test_rgb.header.frame_id = FrameId::CAMERA;
            test_rgb.image = cv::Mat(3, 3, CV_8UC3);
            for (int i = 0; i < 9; ++i)
            {
                test_rgb.image.at<cv::Vec3b>(i / 3, i % 3) = cv::Vec3b(i * 10, i * 20, i * 30);
            }

            // Create test depth image (3x3 float image)
            test_depth.header.stamp = 123.456;
            test_depth.header.frame_id = FrameId::CAMERA;
            test_depth.image = cv::Mat(3, 3, CV_32FC1);
            for (int i = 0; i < 9; ++i)
            {
                test_depth.image.at<float>(i / 3, i % 3) = i * 0.5f;
            }
        }

        RgbImage test_rgb;
        DepthImage test_depth;
    };

    // Test header conversion
    TEST_F(RosImageAdapterTest, HeaderConversion)
    {
        auto ros_header = ros_adapters::to_ros_header(test_rgb.header);

        EXPECT_DOUBLE_EQ(ros_header.stamp.toSec(), 123.456);
        EXPECT_EQ(ros_header.frame_id, "camera");
    }

    // Test RGB image conversion
    TEST_F(RosImageAdapterTest, RgbImageConversion)
    {
        auto ros_image = ros_adapters::to_ros_image(test_rgb);

        EXPECT_EQ(ros_image.height, 3);
        EXPECT_EQ(ros_image.width, 3);
        EXPECT_EQ(ros_image.encoding, sensor_msgs::image_encodings::BGR8);
        EXPECT_EQ(ros_image.is_bigendian, false);
        EXPECT_EQ(ros_image.step, 9);         // 3 cols * 3 bytes per pixel
        EXPECT_EQ(ros_image.data.size(), 27); // 3x3x3

        // Verify some pixel data
        EXPECT_EQ(ros_image.data[0], 0); // First pixel B
        EXPECT_EQ(ros_image.data[1], 0); // First pixel G
        EXPECT_EQ(ros_image.data[2], 0); // First pixel R
    }

    // Test depth image conversion
    TEST_F(RosImageAdapterTest, DepthImageConversion)
    {
        auto ros_image = ros_adapters::to_ros_image(test_depth);

        EXPECT_EQ(ros_image.height, 3);
        EXPECT_EQ(ros_image.width, 3);
        EXPECT_EQ(ros_image.encoding, sensor_msgs::image_encodings::TYPE_32FC1);
        EXPECT_EQ(ros_image.is_bigendian, false);
        EXPECT_EQ(ros_image.step, 12);        // 3 cols * 4 bytes per float
        EXPECT_EQ(ros_image.data.size(), 36); // 3x3x4 bytes
    }

    // Test empty RGB image
    TEST_F(RosImageAdapterTest, EmptyRgbImage)
    {
        RgbImage empty_rgb;
        empty_rgb.image = cv::Mat();

        auto ros_image = ros_adapters::to_ros_image(empty_rgb);

        EXPECT_EQ(ros_image.height, 0);
        EXPECT_EQ(ros_image.width, 0);
        EXPECT_EQ(ros_image.data.size(), 0);
    }

    // Test different frame IDs
    TEST_F(RosImageAdapterTest, DifferentFrameIds)
    {
        test_rgb.header.frame_id = FrameId::VISUAL_ODOMETRY;
        auto ros_header = ros_adapters::to_ros_header(test_rgb.header);
        EXPECT_EQ(ros_header.frame_id, "visual_odometry");

        test_rgb.header.frame_id = FrameId::FIELD;
        ros_header = ros_adapters::to_ros_header(test_rgb.header);
        EXPECT_EQ(ros_header.frame_id, "field");
    }

    // Test large image
    TEST_F(RosImageAdapterTest, LargeImage)
    {
        RgbImage large_rgb;
        large_rgb.image = cv::Mat(100, 200, CV_8UC3, cv::Scalar(50, 100, 150));

        auto ros_image = ros_adapters::to_ros_image(large_rgb);

        EXPECT_EQ(ros_image.height, 100);
        EXPECT_EQ(ros_image.width, 200);
        EXPECT_EQ(ros_image.step, 600);          // 200 * 3
        EXPECT_EQ(ros_image.data.size(), 60000); // 100 * 600
    }

} // namespace auto_battlebot
