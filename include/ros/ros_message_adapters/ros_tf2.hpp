#pragma once

#include <Eigen/Geometry>
#include <geometry_msgs/TransformStamped.hxx>
#include <magic_enum.hpp>
#include <tf2_msgs/TFMessage.hxx>

#include "data_structures/transform.hpp"
#include "enum_to_string_lower.hpp"
#include "ros/ros_message_adapters/ros_image.hpp"

namespace auto_battlebot {
namespace ros_adapters {
/**
 * @brief Convert TransformStamped to ROS TransformStamped message
 */
geometry_msgs::TransformStamped to_ros_transform_stamped(const TransformStamped &transform);

/**
 * @brief Convert TransformStamped to ROS TFMessage
 */
tf2_msgs::TFMessage to_ros_tf_message(const TransformStamped &transform);

}  // namespace ros_adapters
}  // namespace auto_battlebot
