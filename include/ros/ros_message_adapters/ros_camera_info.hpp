#pragma once

#include <sensor_msgs/CameraInfo.hxx>
#include <std_msgs/String.hxx>

#include "data_structures/camera.hpp"
#include "data_structures/header.hpp"
#include "ros/ros_message_adapters/ros_image.hpp"

namespace auto_battlebot {
namespace ros_adapters {
/**
 * @brief Convert CameraInfo to ROS CameraInfo message
 */
sensor_msgs::CameraInfo to_ros_camera_info(const CameraInfo &camera_info);

/**
 * @brief Serialize a FrameIdentity as a JSON string message.
 *
 * Matches how detections are published: a std_msgs/String carrying JSON, so no new schema is
 * needed to record it.
 */
std_msgs::String to_ros_frame_identity(const FrameIdentity &identity);

}  // namespace ros_adapters
}  // namespace auto_battlebot
