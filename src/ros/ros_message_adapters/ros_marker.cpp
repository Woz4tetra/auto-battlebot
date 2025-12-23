#include "ros/ros_message_adapters/ros_marker.hpp"
#include "transform_utils.hpp"

namespace auto_battlebot
{
    namespace ros_adapters
    {
        std::vector<visualization_msgs::Marker> to_ros_field_marker(const FieldDescriptionWithInlierPoints &field)
        {
            std::vector<visualization_msgs::Marker> markers;

            // Create field cuboid marker
            visualization_msgs::Marker field_marker;
            field_marker.header = to_ros_header(field.header);
            field_marker.ns = "field";
            field_marker.id = 0;
            field_marker.type = visualization_msgs::Marker::CUBE;
            field_marker.action = visualization_msgs::Marker::ADD;

            // Set pose from transform
            const auto &tf = field.tf_camera_from_fieldcenter.tf;
            if (tf.rows() >= 3 && tf.cols() >= 4)
            {
                Position position;
                Rotation quaternion;
                matrix_to_position_quaternion(tf, position, quaternion);

                field_marker.pose.position.x = position.x;
                field_marker.pose.position.y = position.y;
                field_marker.pose.position.z = position.z;

                field_marker.pose.orientation.w = quaternion.w;
                field_marker.pose.orientation.x = quaternion.x;
                field_marker.pose.orientation.y = quaternion.y;
                field_marker.pose.orientation.z = quaternion.z;
            }

            // Set scale from size
            field_marker.scale.x = field.size.size.x;
            field_marker.scale.y = field.size.size.y;
            field_marker.scale.z = 0.01; // Thin rectangle for field

            // Set color (green field)
            field_marker.color.r = 0.0f;
            field_marker.color.g = 1.0f;
            field_marker.color.b = 0.0f;
            field_marker.color.a = 0.5f; // Semi-transparent

            field_marker.frame_locked = false;

            markers.push_back(field_marker);

            // Create inlier points marker
            if (field.inlier_points.cloud && !field.inlier_points.cloud->points.empty())
            {
                visualization_msgs::Marker points_marker;
                points_marker.header = to_ros_header(field.header);
                points_marker.ns = "field_inliers";
                points_marker.id = 1;
                points_marker.type = visualization_msgs::Marker::POINTS;
                points_marker.action = visualization_msgs::Marker::ADD;

                points_marker.pose.orientation.w = 1.0;
                points_marker.pose.orientation.x = 0.0;
                points_marker.pose.orientation.y = 0.0;
                points_marker.pose.orientation.z = 0.0;

                // Set point size
                points_marker.scale.x = 0.01; // Point width
                points_marker.scale.y = 0.01; // Point height

                // Find min/max z values for colorization
                float min_z = std::numeric_limits<float>::max();
                float max_z = std::numeric_limits<float>::min();
                for (const auto &point : field.inlier_points.cloud->points)
                {
                    min_z = std::min(min_z, point.z);
                    max_z = std::max(max_z, point.z);
                }
                float z_range = max_z - min_z;
                if (z_range < 1e-6f)
                    z_range = 1.0f; // Avoid division by zero

                // Add all inlier points with per-point colors
                for (const auto &point : field.inlier_points.cloud->points)
                {
                    geometry_msgs::Point p;
                    p.x = point.x;
                    p.y = point.y;
                    p.z = point.z;
                    points_marker.points.push_back(p);

                    // Color gradient from blue (low) to red (high) based on z-coordinate
                    float normalized_z = (point.z - min_z) / z_range;
                    std_msgs::ColorRGBA color;
                    color.r = normalized_z;
                    color.g = 0.5f * (1.0f - std::abs(normalized_z - 0.5f) * 2.0f);
                    color.b = 1.0f - normalized_z;
                    color.a = 1.0f;
                    points_marker.colors.push_back(color);
                }

                points_marker.frame_locked = false;

                markers.push_back(points_marker);
            }

            return markers;
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
