#pragma once

#include <tf2_msgs/TFMessage.hxx>
#include <geometry_msgs/TransformStamped.hxx>
#include "data_structures/transform.hpp"

namespace auto_battlebot
{
    namespace ros_adapters
    {
        /**
         * @brief Convert TransformStamped to ROS TransformStamped message
         */
        geometry_msgs::TransformStamped to_ros_transform_stamped(const TransformStamped &transform);

        /**
         * @brief Convert TransformStamped to ROS TFMessage
         */
        tf2_msgs::TFMessage to_ros_tf_message(const TransformStamped &transform);

    } // namespace ros_adapters
} // namespace auto_battlebot
