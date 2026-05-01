#include "ros/ros_message_adapters/ros_marker.hpp"

#include <cmath>
#include <magic_enum.hpp>

#include "enum_to_string_lower.hpp"
#include "label_utils.hpp"
#include "transform_utils.hpp"

namespace auto_battlebot {
namespace ros_adapters {
namespace {
/**
 * @brief Convert ColorRGBf to std_msgs::ColorRGBA
 */
std_msgs::ColorRGBA to_ros_color(Group group, float alpha = 1.0f) {
    auto color = get_color_for_index(group);
    std_msgs::ColorRGBA ros_color;
    ros_color.r = color.r;
    ros_color.g = color.g;
    ros_color.b = color.b;
    ros_color.a = alpha;
    return ros_color;
}
}  // anonymous namespace

std::vector<visualization_msgs::Marker> to_ros_field_marker(
    const FieldDescriptionWithInlierPoints &field) {
    std::vector<visualization_msgs::Marker> markers;

    // Create field border line strip marker
    visualization_msgs::Marker field_marker;
    field_marker.header = to_ros_header(field.header);
    field_marker.ns = "field";
    field_marker.id = 0;
    field_marker.type = visualization_msgs::Marker::LINE_STRIP;
    field_marker.action = visualization_msgs::Marker::ADD;

    // Identity pose - corners are expressed in the field center frame via the transform
    field_marker.pose.orientation.w = 1.0;

    // Build the four corners of the rectangle in field-center-local coordinates,
    // then transform them into the camera frame using tf_camera_from_fieldcenter.
    const auto &tf = field.tf_camera_from_fieldcenter.tf;
    if (tf.rows() >= 3 && tf.cols() >= 4) {
        const float hx = field.size.size.x / 2.0f;
        const float hy = field.size.size.y / 2.0f;

        // Corners in field-center frame (z = 0)
        const std::array<Eigen::Vector4d, 4> local_corners = {{
            {-hx, -hy, 0.0, 1.0},
            {hx, -hy, 0.0, 1.0},
            {hx, hy, 0.0, 1.0},
            {-hx, hy, 0.0, 1.0},
        }};

        for (const auto &c : local_corners) {
            Eigen::Vector3d world = tf.block<3, 4>(0, 0) * c;
            geometry_msgs::Point p;
            p.x = world.x();
            p.y = world.y();
            p.z = world.z();
            field_marker.points.push_back(p);
        }
        // Close the loop
        field_marker.points.push_back(field_marker.points.front());
    }

    field_marker.scale.x = 0.01;  // Line width

    // Set color (green border)
    field_marker.color.r = 0.0f;
    field_marker.color.g = 1.0f;
    field_marker.color.b = 0.0f;
    field_marker.color.a = 1.0f;

    field_marker.frame_locked = false;

    markers.push_back(field_marker);

    // Create inlier points marker
    if (field.inlier_points.cloud && !field.inlier_points.cloud->points.empty()) {
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
        points_marker.scale.x = 0.01;  // Point width
        points_marker.scale.y = 0.01;  // Point height

        // Find min/max z values for colorization
        float min_z = std::numeric_limits<float>::max();
        float max_z = std::numeric_limits<float>::min();
        for (const auto &point : field.inlier_points.cloud->points) {
            min_z = std::min(min_z, point.z);
            max_z = std::max(max_z, point.z);
        }
        float z_range = max_z - min_z;
        if (z_range < 1e-6f) z_range = 1.0f;  // Avoid division by zero

        // Add all inlier points with per-point colors
        for (const auto &point : field.inlier_points.cloud->points) {
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

std::vector<visualization_msgs::Marker> to_ros_robot_markers(
    const RobotDescriptionsStamped &robots) {
    std::vector<visualization_msgs::Marker> markers;

    int keypoint_counter = 0;
    for (size_t i = 0; i < robots.descriptions.size(); ++i) {
        const auto &robot = robots.descriptions[i];
        const auto robot_frame_id_index = magic_enum::enum_index(robot.frame_id);

        // Get color based on robot label for consistency
        std_msgs::ColorRGBA robot_color = to_ros_color(robot.group, 0.7f);
        std_msgs::ColorRGBA solid_color = to_ros_color(robot.group, 1.0f);

        // Cube marker for robot body
        visualization_msgs::Marker cube_marker;
        cube_marker.header = to_ros_header(robots.header);
        cube_marker.ns = "robot_bounds";
        cube_marker.id = robot_frame_id_index.value_or(i);
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
        arrow_marker.id = robot_frame_id_index.value_or(i);
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
        arrow_marker.scale.x = robot.size.x * 1.5;  // arrow length (along robot x-axis)
        arrow_marker.scale.y = 0.015;               // shaft diameter (fixed)
        arrow_marker.scale.z = 0.015;               // head diameter (fixed)

        arrow_marker.color = solid_color;
        arrow_marker.frame_locked = false;

        markers.push_back(arrow_marker);

        // Text marker for robot label
        visualization_msgs::Marker text_marker;
        text_marker.header = to_ros_header(robots.header);
        text_marker.ns = "robot_labels";
        text_marker.id = robot_frame_id_index.value_or(i);
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

        text_marker.scale.z = 0.1;  // Text height
        text_marker.text = enum_to_string_lower(robot.frame_id);
        text_marker.color = solid_color;
        text_marker.frame_locked = false;

        markers.push_back(text_marker);

        // Keypoint markers
        for (size_t kp_idx = 0; kp_idx < robot.keypoints.size(); ++kp_idx) {
            const Position &keypoint = robot.keypoints[kp_idx];

            // Sphere marker for robot keypoint
            visualization_msgs::Marker keypoint_marker;
            keypoint_marker.header = to_ros_header(robots.header);
            keypoint_marker.ns = "robot_keypoints";
            keypoint_marker.id = keypoint_counter++;
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
        if (robot.keypoints.size() > 1) {
            visualization_msgs::Marker line_marker;
            line_marker.header = to_ros_header(robots.header);
            line_marker.ns = "robot_keypoint_lines";
            line_marker.id = robot_frame_id_index.value_or(i);
            line_marker.type = visualization_msgs::Marker::LINE_STRIP;
            line_marker.action = visualization_msgs::Marker::ADD;

            line_marker.pose.orientation.w = 1.0;
            line_marker.scale.x = 0.005;  // Line width

            line_marker.color = solid_color;
            line_marker.frame_locked = false;

            for (const Position &keypoint : robot.keypoints) {
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

std::vector<visualization_msgs::Marker> to_ros_navigation_markers(
    const NavigationVisualization &nav) {
    std::vector<visualization_msgs::Marker> markers;
    auto ros_header = to_ros_header(nav.header);
    int id = 0;

    // Find our robot in the descriptions for position/heading
    const RobotDescription *our_robot = nullptr;
    for (const auto &r : nav.robots.descriptions) {
        if (r.frame_id == FrameId::OUR_ROBOT_1) {
            our_robot = &r;
            break;
        }
    }

    // --- Pursuit line: our robot → target ---
    if (nav.path.has_value()) {
        const auto &path = nav.path.value();

        visualization_msgs::Marker line;
        line.header = ros_header;
        line.ns = "nav_pursuit_line";
        line.id = id++;
        line.type = visualization_msgs::Marker::LINE_STRIP;
        line.action = visualization_msgs::Marker::ADD;
        line.pose.orientation.w = 1.0;
        line.scale.x = 0.005;

        line.color.r = 1.0f;
        line.color.g = 0.6f;
        line.color.b = 0.0f;
        line.color.a = 0.8f;

        geometry_msgs::Point p_our, p_target;
        p_our.x = path.our_x;
        p_our.y = path.our_y;
        p_our.z = 0.01;
        p_target.x = path.target_x;
        p_target.y = path.target_y;
        p_target.z = 0.01;
        line.points.push_back(p_our);
        line.points.push_back(p_target);

        markers.push_back(line);

        // --- Target crosshair (two short perpendicular lines) ---
        constexpr double cross_size = 0.04;
        visualization_msgs::Marker crosshair;
        crosshair.header = ros_header;
        crosshair.ns = "nav_target";
        crosshair.id = id++;
        crosshair.type = visualization_msgs::Marker::LINE_LIST;
        crosshair.action = visualization_msgs::Marker::ADD;
        crosshair.pose.orientation.w = 1.0;
        crosshair.scale.x = 0.006;

        crosshair.color.r = 1.0f;
        crosshair.color.g = 0.2f;
        crosshair.color.b = 0.2f;
        crosshair.color.a = 1.0f;

        auto make_pt = [](double x, double y, double z) {
            geometry_msgs::Point p;
            p.x = x;
            p.y = y;
            p.z = z;
            return p;
        };

        double tx = path.target_x, ty = path.target_y, tz = 0.01;
        crosshair.points.push_back(make_pt(tx - cross_size, ty, tz));
        crosshair.points.push_back(make_pt(tx + cross_size, ty, tz));
        crosshair.points.push_back(make_pt(tx, ty - cross_size, tz));
        crosshair.points.push_back(make_pt(tx, ty + cross_size, tz));

        markers.push_back(crosshair);
    }

    if (our_robot) {
        Pose2D pose = pose_to_pose2d(our_robot->pose);
        double cos_yaw = std::cos(pose.yaw);
        double sin_yaw = std::sin(pose.yaw);

        // --- Velocity arrow: shows commanded linear velocity direction & magnitude ---
        {
            double vx_field = nav.command.linear_x * cos_yaw - nav.command.linear_y * sin_yaw;
            double vy_field = nav.command.linear_x * sin_yaw + nav.command.linear_y * cos_yaw;
            double v_mag = std::sqrt(vx_field * vx_field + vy_field * vy_field);
            constexpr double arrow_scale = 0.15;

            if (v_mag > 0.01) {
                visualization_msgs::Marker vel_arrow;
                vel_arrow.header = ros_header;
                vel_arrow.ns = "nav_velocity";
                vel_arrow.id = id++;
                vel_arrow.type = visualization_msgs::Marker::ARROW;
                vel_arrow.action = visualization_msgs::Marker::ADD;
                vel_arrow.pose.orientation.w = 1.0;
                vel_arrow.lifetime = miniros::Duration(0.1);

                vel_arrow.scale.x = 0.008;  // shaft diameter
                vel_arrow.scale.y = 0.014;  // head diameter
                vel_arrow.scale.z = 0.012;  // head length

                vel_arrow.color.r = 0.2f;
                vel_arrow.color.g = 1.0f;
                vel_arrow.color.b = 0.2f;
                vel_arrow.color.a = 0.9f;

                geometry_msgs::Point start, end;
                start.x = pose.x;
                start.y = pose.y;
                start.z = 0.02;
                end.x = pose.x + vx_field * arrow_scale;
                end.y = pose.y + vy_field * arrow_scale;
                end.z = 0.02;
                vel_arrow.points.push_back(start);
                vel_arrow.points.push_back(end);

                markers.push_back(vel_arrow);
            }
        }

        // --- Angular velocity arc: shows commanded turning direction ---
        {
            double omega = nav.command.angular_z;
            if (std::abs(omega) > 0.05) {
                visualization_msgs::Marker arc;
                arc.header = ros_header;
                arc.ns = "nav_angular";
                arc.id = id++;
                arc.type = visualization_msgs::Marker::LINE_STRIP;
                arc.action = visualization_msgs::Marker::ADD;
                arc.pose.orientation.w = 1.0;
                arc.scale.x = 0.004;
                arc.lifetime = miniros::Duration(0.1);

                arc.color.r = 0.4f;
                arc.color.g = 0.6f;
                arc.color.b = 1.0f;
                arc.color.a = 0.9f;

                constexpr double arc_radius = 0.06;
                double sweep = omega * M_PI * 0.5;
                sweep = std::clamp(sweep, -M_PI, M_PI);
                constexpr int segments = 16;

                for (int s = 0; s <= segments; ++s) {
                    double frac = static_cast<double>(s) / segments;
                    double a = pose.yaw + M_PI_2 + frac * sweep;
                    geometry_msgs::Point p;
                    p.x = pose.x + arc_radius * std::cos(a);
                    p.y = pose.y + arc_radius * std::sin(a);
                    p.z = 0.02;
                    arc.points.push_back(p);
                }

                // Arrowhead: small triangle at the arc tip
                double tip_angle = pose.yaw + M_PI_2 + sweep;
                double tangent = tip_angle + (omega > 0 ? M_PI_2 : -M_PI_2);
                constexpr double head_len = 0.012;

                visualization_msgs::Marker arc_head;
                arc_head.header = ros_header;
                arc_head.ns = "nav_angular_head";
                arc_head.id = id++;
                arc_head.type = visualization_msgs::Marker::LINE_LIST;
                arc_head.action = visualization_msgs::Marker::ADD;
                arc_head.pose.orientation.w = 1.0;
                arc_head.scale.x = 0.004;
                arc_head.color = arc.color;
                arc_head.lifetime = miniros::Duration(0.1);

                geometry_msgs::Point tip;
                tip.x = pose.x + arc_radius * std::cos(tip_angle);
                tip.y = pose.y + arc_radius * std::sin(tip_angle);
                tip.z = 0.02;

                geometry_msgs::Point h1, h2;
                h1.x = tip.x + head_len * std::cos(tangent + 2.5);
                h1.y = tip.y + head_len * std::sin(tangent + 2.5);
                h1.z = 0.02;
                h2.x = tip.x + head_len * std::cos(tangent - 2.5);
                h2.y = tip.y + head_len * std::sin(tangent - 2.5);
                h2.z = 0.02;

                arc_head.points.push_back(tip);
                arc_head.points.push_back(h1);
                arc_head.points.push_back(tip);
                arc_head.points.push_back(h2);

                markers.push_back(arc);
                markers.push_back(arc_head);
            }
        }
    }

    // --- DELETE markers for unused IDs to clean up stale visuals ---
    const std::vector<std::string> all_ns = {"nav_pursuit_line", "nav_target", "nav_velocity",
                                             "nav_angular", "nav_angular_head"};
    for (const auto &ns : all_ns) {
        bool already_present = false;
        for (const auto &m : markers) {
            if (m.ns == ns) {
                already_present = true;
                break;
            }
        }
        if (!already_present) {
            visualization_msgs::Marker del;
            del.header = ros_header;
            del.ns = ns;
            del.id = 0;
            del.action = visualization_msgs::Marker::DELETE;
            markers.push_back(del);
        }
    }

    return markers;
}

}  // namespace ros_adapters
}  // namespace auto_battlebot
