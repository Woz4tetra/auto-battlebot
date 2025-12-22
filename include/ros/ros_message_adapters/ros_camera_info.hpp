#pragma once

#include <sensor_msgs/CameraInfo.hxx>
#include "data_structures/camera.hpp"
#include "data_structures/header.hpp"
#include "ros/ros_message_adapters/ros_image.hpp"

namespace auto_battlebot
{
    namespace ros_adapters
    {
        /**
         * @brief Convert CameraInfo to ROS CameraInfo message
         */
        sensor_msgs::CameraInfo to_ros_camera_info(const CameraInfo &camera_info);

    } // namespace ros_adapters
} // namespace auto_battlebot
