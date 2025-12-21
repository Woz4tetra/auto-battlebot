#pragma once

#include <sensor_msgs/Image.hxx>
#include <std_msgs/Header.hxx>
#include <sensor_msgs/image_encodings.h>

#include "data_structures/image.hpp"
#include "data_structures/header.hpp"

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
        sensor_msgs::Image to_ros_image(const RgbImage &rgb_image, const Header &header);

        /**
         * @brief Convert DepthImage to ROS Image message
         */
        sensor_msgs::Image to_ros_image(const DepthImage &depth_image, const Header &header);

    } // namespace ros_adapters
} // namespace auto_battlebot
