#include <gtest/gtest.h>
#include "ros/ros_message_adapters/ros_point_cloud2.hpp"
#include <sensor_msgs/PointField.hxx>

namespace auto_battlebot
{
    class RosPointCloud2AdapterTest : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            test_header.timestamp = 123.456;
            test_header.frame_id = FrameId::CAMERA;

            // Create simple 2x2 depth and RGB images
            test_depth.image = cv::Mat(2, 2, CV_32FC1);
            test_depth.image.at<float>(0, 0) = 1.0f;
            test_depth.image.at<float>(0, 1) = 2.0f;
            test_depth.image.at<float>(1, 0) = 3.0f;
            test_depth.image.at<float>(1, 1) = 4.0f;

            test_rgb.image = cv::Mat(2, 2, CV_8UC3);
            test_rgb.image.at<cv::Vec3b>(0, 0) = cv::Vec3b(255, 0, 0);     // Blue
            test_rgb.image.at<cv::Vec3b>(0, 1) = cv::Vec3b(0, 255, 0);     // Green
            test_rgb.image.at<cv::Vec3b>(1, 0) = cv::Vec3b(0, 0, 255);     // Red
            test_rgb.image.at<cv::Vec3b>(1, 1) = cv::Vec3b(255, 255, 255); // White

            // Create camera info
            test_camera_info.width = 2;
            test_camera_info.height = 2;
            test_camera_info.intrinsics = cv::Mat::eye(3, 3, CV_64F);
            test_camera_info.intrinsics.at<double>(0, 0) = 1.0; // fx
            test_camera_info.intrinsics.at<double>(1, 1) = 1.0; // fy
            test_camera_info.intrinsics.at<double>(0, 2) = 1.0; // cx
            test_camera_info.intrinsics.at<double>(1, 2) = 1.0; // cy
        }

        Header test_header;
        DepthImage test_depth;
        RgbImage test_rgb;
        CameraInfo test_camera_info;
    };

    // Test basic point cloud structure
    TEST_F(RosPointCloud2AdapterTest, BasicStructure)
    {
        auto cloud = ros_adapters::to_ros_point_cloud2(test_depth, test_rgb, test_camera_info, test_header);

        EXPECT_EQ(cloud.height, 2);
        EXPECT_EQ(cloud.width, 2);
        EXPECT_EQ(cloud.point_step, 16); // 4 floats * 4 bytes
        EXPECT_EQ(cloud.row_step, 32);   // 2 points * 16 bytes
        EXPECT_EQ(cloud.is_bigendian, false);
        EXPECT_EQ(cloud.is_dense, false);
        EXPECT_EQ(cloud.data.size(), 64); // 2 rows * 32 bytes
    }

    // Test field definitions
    TEST_F(RosPointCloud2AdapterTest, FieldDefinitions)
    {
        auto cloud = ros_adapters::to_ros_point_cloud2(test_depth, test_rgb, test_camera_info, test_header);

        EXPECT_EQ(cloud.fields.size(), 4);

        EXPECT_EQ(cloud.fields[0].name, "x");
        EXPECT_EQ(cloud.fields[0].offset, 0);
        EXPECT_EQ(cloud.fields[0].datatype, sensor_msgs::PointField::FLOAT32);
        EXPECT_EQ(cloud.fields[0].count, 1);

        EXPECT_EQ(cloud.fields[1].name, "y");
        EXPECT_EQ(cloud.fields[1].offset, 4);
        EXPECT_EQ(cloud.fields[1].datatype, sensor_msgs::PointField::FLOAT32);
        EXPECT_EQ(cloud.fields[1].count, 1);

        EXPECT_EQ(cloud.fields[2].name, "z");
        EXPECT_EQ(cloud.fields[2].offset, 8);
        EXPECT_EQ(cloud.fields[2].datatype, sensor_msgs::PointField::FLOAT32);
        EXPECT_EQ(cloud.fields[2].count, 1);

        EXPECT_EQ(cloud.fields[3].name, "rgb");
        EXPECT_EQ(cloud.fields[3].offset, 12);
        EXPECT_EQ(cloud.fields[3].datatype, sensor_msgs::PointField::FLOAT32);
        EXPECT_EQ(cloud.fields[3].count, 1);
    }

    // Test header
    TEST_F(RosPointCloud2AdapterTest, HeaderInfo)
    {
        auto cloud = ros_adapters::to_ros_point_cloud2(test_depth, test_rgb, test_camera_info, test_header);

        EXPECT_DOUBLE_EQ(cloud.header.stamp.toSec(), 123.456);
        EXPECT_EQ(cloud.header.frame_id, "camera");
    }

    // Test point data size
    TEST_F(RosPointCloud2AdapterTest, PointDataSize)
    {
        auto cloud = ros_adapters::to_ros_point_cloud2(test_depth, test_rgb, test_camera_info, test_header);

        // Should have 4 points (2x2) with 16 bytes each
        EXPECT_EQ(cloud.data.size(), 64);
    }

    // Test with empty RGB image
    TEST_F(RosPointCloud2AdapterTest, EmptyRgbImage)
    {
        RgbImage empty_rgb;
        empty_rgb.image = cv::Mat();

        auto cloud = ros_adapters::to_ros_point_cloud2(test_depth, empty_rgb, test_camera_info, test_header);

        // Should still generate point cloud with default gray color
        EXPECT_EQ(cloud.height, 2);
        EXPECT_EQ(cloud.width, 2);
        EXPECT_GT(cloud.data.size(), 0);
    }

    // Test large point cloud
    TEST_F(RosPointCloud2AdapterTest, LargePointCloud)
    {
        DepthImage large_depth;
        large_depth.image = cv::Mat(100, 100, CV_32FC1, cv::Scalar(1.0f));

        RgbImage large_rgb;
        large_rgb.image = cv::Mat(100, 100, CV_8UC3, cv::Scalar(128, 128, 128));

        CameraInfo large_camera_info;
        large_camera_info.width = 100;
        large_camera_info.height = 100;
        large_camera_info.intrinsics = cv::Mat::eye(3, 3, CV_64F);
        large_camera_info.intrinsics.at<double>(0, 0) = 50.0;
        large_camera_info.intrinsics.at<double>(1, 1) = 50.0;
        large_camera_info.intrinsics.at<double>(0, 2) = 50.0;
        large_camera_info.intrinsics.at<double>(1, 2) = 50.0;

        auto cloud = ros_adapters::to_ros_point_cloud2(large_depth, large_rgb, large_camera_info, test_header);

        EXPECT_EQ(cloud.height, 100);
        EXPECT_EQ(cloud.width, 100);
        EXPECT_EQ(cloud.data.size(), 100 * 100 * 16);
    }

} // namespace auto_battlebot
