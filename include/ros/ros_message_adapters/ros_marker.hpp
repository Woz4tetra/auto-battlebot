#pragma once

#include <visualization_msgs/Marker.hxx>
#include <visualization_msgs/MarkerArray.hxx>
#include "data_structures/field.hpp"
#include "data_structures/keypoint.hpp"
#include "data_structures/robot.hpp"
#include "ros/ros_message_adapters/ros_image.hpp"
#include <geometry_msgs/Point.hxx>
#include <geometry_msgs/Vector3.hxx>
#include <std_msgs/ColorRGBA.hxx>

namespace auto_battlebot
{
    namespace ros_adapters
    {
        /**
         * @brief Convert FieldDescription to ROS Marker messages (field cuboid and inlier points)
         */
        std::vector<visualization_msgs::Marker> to_ros_field_marker(const FieldDescriptionWithInlierPoints &field);

        /**
         * @brief Convert RobotDescriptionsStamped to ROS Marker messages
         */
        std::vector<visualization_msgs::Marker> to_ros_robot_markers(const RobotDescriptionsStamped &robots);

    } // namespace ros_adapters
} // namespace auto_battlebot
