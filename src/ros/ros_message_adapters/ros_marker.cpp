#include "ros/ros_message_adapters/ros_marker.hpp"
#include "transform_utils.hpp"
#include "enum_to_string_lower.hpp"
#include "label_utils.hpp"
#include <cmath>

namespace auto_battlebot
{
    namespace ros_adapters
    {
        namespace
        {
            /**
             * @brief Convert ColorRGBf to std_msgs::ColorRGBA
             */
            std_msgs::ColorRGBA to_ros_color(Label label, float alpha = 1.0f)
            {
                auto color = get_color_for_label(label);
                std_msgs::ColorRGBA ros_color;
                ros_color.r = color.r;
                ros_color.g = color.g;
                ros_color.b = color.b;
                ros_color.a = alpha;
                return ros_color;
            }
        } // anonymous namespace

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

        std::vector<visualization_msgs::Marker> to_ros_robot_markers(const RobotDescriptionsStamped &robots)
        {
            std::vector<visualization_msgs::Marker> markers;
            int marker_id = 0;

            for (size_t i = 0; i < robots.descriptions.size(); ++i)
            {
                const auto &robot = robots.descriptions[i];

                // Get color based on robot label for consistency
                std_msgs::ColorRGBA robot_color = to_ros_color(robot.label, 0.7f);
                std_msgs::ColorRGBA solid_color = to_ros_color(robot.label, 1.0f);

                // Cube marker for robot body
                visualization_msgs::Marker cube_marker;
                cube_marker.header = to_ros_header(robots.header);
                cube_marker.ns = "robot_bounds";
                cube_marker.id = marker_id++;
                cube_marker.type = visualization_msgs::Marker::CUBE;
                cube_marker.action = visualization_msgs::Marker::ADD;

                // Set pose
                cube_marker.pose.position.x = robot.pose.position.x;
                cube_marker.pose.position.y = robot.pose.position.y;
                cube_marker.pose.position.z = robot.pose.position.z;

                cube_marker.pose.orientation.w = robot.pose.rotation.w;
                cube_marker.pose.orientation.x = robot.pose.rotation.x;
                cube_marker.pose.orientation.y = robot.pose.rotation.y;
                cube_marker.pose.orientation.z = robot.pose.rotation.z;

                // Set scale from size
                cube_marker.scale.x = robot.size.x;
                cube_marker.scale.y = robot.size.y;
                cube_marker.scale.z = robot.size.z;

                cube_marker.color = robot_color;
                cube_marker.frame_locked = false;

                markers.push_back(cube_marker);

                // Arrow marker for robot pose/heading
                visualization_msgs::Marker arrow_marker;
                arrow_marker.header = to_ros_header(robots.header);
                arrow_marker.ns = "robot_poses";
                arrow_marker.id = marker_id++;
                arrow_marker.type = visualization_msgs::Marker::ARROW;
                arrow_marker.action = visualization_msgs::Marker::ADD;

                // Set pose
                arrow_marker.pose.position.x = robot.pose.position.x;
                arrow_marker.pose.position.y = robot.pose.position.y;
                arrow_marker.pose.position.z = robot.pose.position.z;

                arrow_marker.pose.orientation.w = robot.pose.rotation.w;
                arrow_marker.pose.orientation.x = robot.pose.rotation.x;
                arrow_marker.pose.orientation.y = robot.pose.rotation.y;
                arrow_marker.pose.orientation.z = robot.pose.rotation.z;

                // Set arrow scale: length based on robot x-size, fixed diameter
                arrow_marker.scale.x = robot.size.x * 1.5; // arrow length (along robot x-axis)
                arrow_marker.scale.y = 0.015;              // shaft diameter (fixed)
                arrow_marker.scale.z = 0.015;              // head diameter (fixed)

                arrow_marker.color = solid_color;
                arrow_marker.frame_locked = false;

                markers.push_back(arrow_marker);

                // Text marker for robot label
                visualization_msgs::Marker text_marker;
                text_marker.header = to_ros_header(robots.header);
                text_marker.ns = "robot_labels";
                text_marker.id = marker_id++;
                text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                text_marker.action = visualization_msgs::Marker::ADD;

                // Position text above the robot
                text_marker.pose.position.x = robot.pose.position.x;
                text_marker.pose.position.y = robot.pose.position.y;
                text_marker.pose.position.z = robot.pose.position.z + robot.size.z * 0.6;

                text_marker.pose.orientation.w = 1.0;
                text_marker.pose.orientation.x = 0.0;
                text_marker.pose.orientation.y = 0.0;
                text_marker.pose.orientation.z = 0.0;

                text_marker.scale.z = 0.1; // Text height
                text_marker.text = enum_to_string_lower(robot.label);
                text_marker.color = solid_color;
                text_marker.frame_locked = false;

                markers.push_back(text_marker);

                // Keypoint markers
                for (size_t kp_idx = 0; kp_idx < robot.keypoints.size(); ++kp_idx)
                {
                    const Position &keypoint = robot.keypoints[kp_idx];

                    // Sphere marker for robot keypoint
                    visualization_msgs::Marker keypoint_marker;
                    keypoint_marker.header = to_ros_header(robots.header);
                    keypoint_marker.ns = "robot_keypoints";
                    keypoint_marker.id = marker_id++;
                    keypoint_marker.type = visualization_msgs::Marker::SPHERE;
                    keypoint_marker.action = visualization_msgs::Marker::ADD;

                    // Set pose
                    keypoint_marker.pose.position.x = keypoint.x;
                    keypoint_marker.pose.position.y = keypoint.y;
                    keypoint_marker.pose.position.z = keypoint.z;

                    keypoint_marker.pose.orientation.w = 1.0;
                    keypoint_marker.pose.orientation.x = 0.0;
                    keypoint_marker.pose.orientation.y = 0.0;
                    keypoint_marker.pose.orientation.z = 0.0;

                    keypoint_marker.scale.x = 0.02;
                    keypoint_marker.scale.y = 0.02;
                    keypoint_marker.scale.z = 0.02;

                    keypoint_marker.color = solid_color;
                    keypoint_marker.frame_locked = false;

                    markers.push_back(keypoint_marker);
                }

                // Connect keypoints with line strips if there are multiple
                if (robot.keypoints.size() > 1)
                {
                    visualization_msgs::Marker line_marker;
                    line_marker.header = to_ros_header(robots.header);
                    line_marker.ns = "robot_keypoint_lines";
                    line_marker.id = marker_id++;
                    line_marker.type = visualization_msgs::Marker::LINE_STRIP;
                    line_marker.action = visualization_msgs::Marker::ADD;

                    line_marker.pose.orientation.w = 1.0;
                    line_marker.scale.x = 0.005; // Line width

                    line_marker.color = solid_color;
                    line_marker.frame_locked = false;

                    for (const Position &keypoint : robot.keypoints)
                    {
                        geometry_msgs::Point p;
                        p.x = keypoint.x;
                        p.y = keypoint.y;
                        p.z = keypoint.z;
                        line_marker.points.push_back(p);
                    }

                    markers.push_back(line_marker);
                }
            }

            return markers;
        }

    } // namespace ros_adapters
} // namespace auto_battlebot
