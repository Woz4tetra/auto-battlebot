#include "ros/ros_message_adapters/ros_marker.hpp"
#include "ros/ros_message_adapters/ros_image.hpp"
#include <geometry_msgs/Point.hxx>
#include <geometry_msgs/Vector3.hxx>
#include <std_msgs/ColorRGBA.hxx>

namespace auto_battlebot
{
    namespace ros_adapters
    {
        visualization_msgs::Marker to_ros_field_marker(const FieldDescription &field)
        {
            visualization_msgs::Marker marker;
            marker.header = to_ros_header(field.header);
            marker.ns = "field";
            marker.id = 0;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;

            // Set pose from transform
            const auto &tf = field.tf_camera_from_fieldcenter.tf;
            if (tf.rows() >= 3 && tf.cols() >= 4)
            {
                marker.pose.position.x = tf(0, 3);
                marker.pose.position.y = tf(1, 3);
                marker.pose.position.z = tf(2, 3);

                // Extract rotation (simplified - assumes no rotation for now)
                marker.pose.orientation.w = 1.0;
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
            }

            // Set scale from size
            marker.scale.x = field.size.size.x;
            marker.scale.y = field.size.size.y;
            marker.scale.z = 0.01; // Thin rectangle for field

            // Set color (green field)
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 0.5f; // Semi-transparent

            marker.frame_locked = false;

            return marker;
        }

        std::vector<visualization_msgs::Marker> to_ros_keypoint_markers(const KeypointsStamped &keypoints)
        {
            std::vector<visualization_msgs::Marker> markers;

            for (size_t i = 0; i < keypoints.keypoints.size(); ++i)
            {
                const auto &kp = keypoints.keypoints[i];

                visualization_msgs::Marker marker;
                marker.header = to_ros_header(keypoints.header);
                marker.ns = "keypoints";
                marker.id = i;
                marker.type = visualization_msgs::Marker::SPHERE;
                marker.action = visualization_msgs::Marker::ADD;

                // Project 2D keypoint to 3D (simplified - assumes z=0 or use camera info)
                marker.pose.position.x = kp.x;
                marker.pose.position.y = kp.y;
                marker.pose.position.z = 0.0;

                marker.pose.orientation.w = 1.0;
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;

                // Small sphere for keypoint
                marker.scale.x = 0.05;
                marker.scale.y = 0.05;
                marker.scale.z = 0.05;

                // Color based on label
                marker.color.r = 1.0f;
                marker.color.g = 0.0f;
                marker.color.b = 0.0f;
                marker.color.a = 1.0f;

                marker.frame_locked = false;

                markers.push_back(marker);
            }

            return markers;
        }

        std::vector<visualization_msgs::Marker> to_ros_robot_markers(const RobotDescriptionsStamped &robots)
        {
            std::vector<visualization_msgs::Marker> markers;

            for (size_t i = 0; i < robots.descriptions.size(); ++i)
            {
                const auto &robot = robots.descriptions[i];

                visualization_msgs::Marker marker;
                marker.header = to_ros_header(robots.header);
                marker.ns = "robots";
                marker.id = i;
                marker.type = visualization_msgs::Marker::CUBE;
                marker.action = visualization_msgs::Marker::ADD;

                // Set pose
                marker.pose.position.x = robot.pose.position.x;
                marker.pose.position.y = robot.pose.position.y;
                marker.pose.position.z = robot.pose.position.z;

                marker.pose.orientation.w = robot.pose.rotation.w;
                marker.pose.orientation.x = robot.pose.rotation.x;
                marker.pose.orientation.y = robot.pose.rotation.y;
                marker.pose.orientation.z = robot.pose.rotation.z;

                // Set scale from size
                marker.scale.x = robot.size.x;
                marker.scale.y = robot.size.y;
                marker.scale.z = robot.size.z;

                // Color based on group
                if (robot.group == Group::OURS)
                {
                    marker.color.r = 0.0f;
                    marker.color.g = 0.0f;
                    marker.color.b = 1.0f; // Blue for our team
                }
                else
                {
                    marker.color.r = 1.0f;
                    marker.color.g = 0.0f;
                    marker.color.b = 0.0f; // Red for opponent
                }
                marker.color.a = 0.8f;

                marker.frame_locked = false;

                markers.push_back(marker);
            }

            return markers;
        }

    } // namespace ros_adapters
} // namespace auto_battlebot
