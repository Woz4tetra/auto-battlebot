#pragma once

#include <visualization_msgs/Marker.hxx>
#include <visualization_msgs/MarkerArray.hxx>
#include "data_structures/field.hpp"
#include "data_structures/keypoint.hpp"
#include "data_structures/robot.hpp"

namespace auto_battlebot
{
    namespace ros_adapters
    {
        /**
         * @brief Convert FieldDescription to ROS Marker message
         */
        visualization_msgs::Marker to_ros_field_marker(const FieldDescription& field);

        /**
         * @brief Convert KeypointsStamped to ROS Marker messages
         */
        std::vector<visualization_msgs::Marker> to_ros_keypoint_markers(const KeypointsStamped& keypoints);

        /**
         * @brief Convert RobotDescriptionsStamped to ROS Marker messages
         */
        std::vector<visualization_msgs::Marker> to_ros_robot_markers(const RobotDescriptionsStamped& robots);

    } // namespace ros_adapters
} // namespace auto_battlebot
