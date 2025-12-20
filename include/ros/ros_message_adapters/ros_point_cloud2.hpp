#pragma once

#include <sensor_msgs/PointCloud2.hxx>
#include <sensor_msgs/PointField.hxx>
#include "data_structures/camera.hpp"
#include "data_structures/header.hpp"

namespace auto_battlebot
{
    namespace ros_adapters
    {
        /**
         * @brief Convert depth image and camera info to ROS PointCloud2 message
         * 
         * @param depth_image Depth image with z-values
         * @param rgb_image RGB image for color information
         * @param camera_info Camera intrinsics for projection
         * @param header Header information
         * @return sensor_msgs::PointCloud2
         */
        sensor_msgs::PointCloud2 to_ros_point_cloud2(
            const DepthImage& depth_image,
            const RgbImage& rgb_image,
            const CameraInfo& camera_info,
            const Header& header
        );

    } // namespace ros_adapters
} // namespace auto_battlebot
