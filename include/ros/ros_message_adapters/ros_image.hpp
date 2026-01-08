#pragma once

#include <sensor_msgs/Image.hxx>
#include <sensor_msgs/CompressedImage.hxx>
#include <std_msgs/Header.hxx>
#include <sensor_msgs/image_encodings.h>

#include "data_structures/image.hpp"
#include "data_structures/header.hpp"
#include "enum_to_string_lower.hpp"
#include <opencv2/imgcodecs.hpp>

namespace auto_battlebot
{
    namespace ros_adapters
    {
        /**
         * @brief Convert Header to ROS Header message
         */
        std_msgs::Header to_ros_header(const Header &header);

        /**
         * @brief Convert RgbImage to ROS Image message
         */
        sensor_msgs::Image to_ros_image(const RgbImage &rgb_image);

        /**
         * @brief Convert DepthImage to ROS Image message
         */
        sensor_msgs::Image to_ros_image(const DepthImage &depth_image);

        /**
         * @brief Convert RgbImage to ROS CompressedImage (jpeg)
         */
        sensor_msgs::CompressedImage to_ros_image_compressed(const RgbImage &rgb_image);

        /**
         * @brief Convert DepthImage to ROS CompressedImage (jpeg)
         */
        sensor_msgs::CompressedImage to_ros_image_compressed(const DepthImage &depth_image);

    } // namespace ros_adapters
} // namespace auto_battlebot
