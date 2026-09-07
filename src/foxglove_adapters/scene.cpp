#include "foxglove_adapters/scene.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <magic_enum.hpp>
#include <string>

#include "foxglove_adapters/common.hpp"
#include "transform_utils.hpp"

namespace auto_battlebot {
namespace foxglove_adapters {
namespace {

namespace fg = foxglove::schemas;

fg::SceneEntity make_entity(const Header &header, const std::string &ns, int id) {
    fg::SceneEntity entity;
    entity.timestamp = to_timestamp(header.stamp);
    entity.frame_id = frame_id_string(header.frame_id);
    entity.id = ns + "/" + std::to_string(id);
    entity.frame_locked = false;
    return entity;
}

fg::SceneEntity make_entity(const Header &header, const std::string &ns, int id,
                            double lifetime_s) {
    fg::SceneEntity entity = make_entity(header, ns, id);
    entity.lifetime = to_duration(lifetime_s);
    return entity;
}

fg::LinePrimitive make_line(fg::LinePrimitive::LineType type, double thickness,
                            const fg::Color &color) {
    fg::LinePrimitive line;
    line.type = type;
    line.pose = identity_pose();
    line.thickness = thickness;
    line.scale_invariant = false;
    line.color = color;
    return line;
}

fg::SceneEntityDeletion make_delete_all(const Header &header) {
    fg::SceneEntityDeletion deletion;
    deletion.timestamp = to_timestamp(header.stamp);
    deletion.type = fg::SceneEntityDeletion::SceneEntityDeletionType::ALL;
    return deletion;
}

fg::SceneEntityDeletion make_deletion(const Header &header, const std::string &ns, int id) {
    fg::SceneEntityDeletion deletion;
    deletion.timestamp = to_timestamp(header.stamp);
    deletion.type = fg::SceneEntityDeletion::SceneEntityDeletionType::MATCHING_ID;
    deletion.id = ns + "/" + std::to_string(id);
    return deletion;
}

// Pose-form arrow: sits at `pose`, points along its +x, `length` long. Same split of shaft and
// head as the converter uses for legacy pose-form markers.
fg::ArrowPrimitive make_pose_arrow(const fg::Pose &pose, double length, double shaft_diameter,
                                   double head_diameter, const fg::Color &color) {
    fg::ArrowPrimitive arrow;
    arrow.pose = pose;
    arrow.head_length = 0.23 * length;
    arrow.shaft_length = 0.77 * length;
    arrow.shaft_diameter = shaft_diameter;
    arrow.head_diameter = head_diameter;
    arrow.color = color;
    return arrow;
}

// Two-point arrow from `start` to `end`: the pose rotates +x onto the direction vector.
fg::ArrowPrimitive make_segment_arrow(const Eigen::Vector3d &start, const Eigen::Vector3d &end,
                                      double shaft_diameter, double head_diameter,
                                      double head_length, const fg::Color &color) {
    const Eigen::Vector3d delta = end - start;
    const double length = delta.norm();
    // Rotation taking +x onto the direction. Written out by hand: Eigen's FromTwoVectors trips
    // -Werror=maybe-uninitialized through its SVD fallback.
    Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
    if (length > 1e-9) {
        const Eigen::Vector3d dir = delta / length;
        const double dot = dir.x();  // UnitX . dir
        if (dot < -1.0 + 1e-9) {
            rotation = Eigen::Quaterniond(0.0, 0.0, 0.0, 1.0);  // 180 degrees about z
        } else {
            const Eigen::Vector3d axis = Eigen::Vector3d::UnitX().cross(dir);
            rotation = Eigen::Quaterniond(1.0 + dot, axis.x(), axis.y(), axis.z()).normalized();
        }
    }
    fg::ArrowPrimitive arrow;
    fg::Pose pose;
    pose.position = to_vector3(start.x(), start.y(), start.z());
    pose.orientation = to_quaternion(rotation.w(), rotation.x(), rotation.y(), rotation.z());
    arrow.pose = pose;
    arrow.head_length = head_length > 0.0 ? head_length : 0.23 * length;
    arrow.shaft_length = std::max(0.0, length - arrow.head_length);
    arrow.shaft_diameter = shaft_diameter;
    arrow.head_diameter = head_diameter;
    arrow.color = color;
    return arrow;
}

}  // namespace

fg::SceneUpdate to_field_scene(const FieldDescriptionWithInlierPoints &field) {
    fg::SceneUpdate update;
    fg::SceneEntity border = make_entity(field.header, "field", 0);
    fg::LinePrimitive line =
        make_line(fg::LinePrimitive::LineType::LINE_STRIP, 0.01, to_color(0.0f, 1.0f, 0.0f, 1.0f));

    // Four corners in field-center-local coordinates, transformed into the camera frame with
    // tf_camera_from_fieldcenter, then closed back onto the first corner.
    const auto &tf = field.tf_camera_from_fieldcenter.tf;
    if (tf.rows() >= 3 && tf.cols() >= 4) {
        const double hx = field.size.size.x / 2.0;
        const double hy = field.size.size.y / 2.0;
        const std::array<Eigen::Vector4d, 4> local_corners = {{
            {-hx, -hy, 0.0, 1.0},
            {hx, -hy, 0.0, 1.0},
            {hx, hy, 0.0, 1.0},
            {-hx, hy, 0.0, 1.0},
        }};
        for (const auto &c : local_corners) {
            Eigen::Vector3d world = tf.block<3, 4>(0, 0) * c;
            line.points.push_back(to_point3(world.x(), world.y(), world.z()));
        }
        line.points.push_back(line.points.front());
    }
    border.lines.push_back(std::move(line));
    update.entities.push_back(std::move(border));
    return update;
}

std::optional<fg::PointCloud> to_field_point_cloud(const FieldDescriptionWithInlierPoints &field) {
    if (!field.inlier_points.cloud || field.inlier_points.cloud->points.empty()) {
        return std::nullopt;
    }
    fg::PointCloud cloud;
    cloud.timestamp = to_timestamp(field.header.stamp);
    cloud.frame_id = frame_id_string(field.header.frame_id);
    cloud.pose = identity_pose();
    cloud.point_stride = 12;
    const char *names[3] = {"x", "y", "z"};
    for (uint32_t i = 0; i < 3; ++i) {
        fg::PackedElementField f;
        f.name = names[i];
        f.offset = i * 4;
        f.type = fg::PackedElementField::NumericType::FLOAT32;
        cloud.fields.push_back(std::move(f));
    }
    // The PCL points are float32 already; this is a straight copy at 12 bytes per point.
    const auto &points = field.inlier_points.cloud->points;
    cloud.data.resize(points.size() * 12);
    std::byte *out = cloud.data.data();
    for (const auto &p : points) {
        const float xyz[3] = {p.x, p.y, p.z};
        std::memcpy(out, xyz, 12);
        out += 12;
    }
    return cloud;
}

fg::SceneUpdate to_hazard_scene(const FieldDescription &field) {
    constexpr int kRingSegments = 48;
    fg::SceneUpdate update;
    // Entities persist in the 3D panel until deleted, so each update starts by clearing the
    // previous cycle's rings: a hazard that aged out must not linger on the overlay as
    // something the controller is still avoiding. Also keeps the message non-empty when there
    // are no hazards, which the SDK would otherwise refuse to log.
    update.deletions.push_back(make_delete_all(field.header));
    const auto &tf = field.tf_camera_from_fieldcenter.tf;
    if (tf.rows() < 3 || tf.cols() < 4) return update;

    // One ring per hazard, at the hazard's own size. The keep-out it inflates to sits our robot's
    // half-diagonal plus the margin further out; drawing that too was clutter, and the robot
    // markers already give the eye the scale to judge it.
    for (size_t i = 0; i < field.hazards.size(); ++i) {
        const FieldHazard &hazard = field.hazards[i];
        const bool is_static = hazard.source == HazardSource::STATIC;
        fg::SceneEntity entity = make_entity(field.header, "hazards", static_cast<int>(i));
        fg::LinePrimitive ring =
            make_line(fg::LinePrimitive::LineType::LINE_STRIP, 0.012,
                      to_color(1.0f, is_static ? 0.65f : 0.0f, is_static ? 0.0f : 1.0f, 0.9f));
        for (int step = 0; step <= kRingSegments; ++step) {
            const double angle = 2.0 * M_PI * step / kRingSegments;
            Eigen::Vector4d local{hazard.center.x + hazard.object_radius * std::cos(angle),
                                  hazard.center.y + hazard.object_radius * std::sin(angle), 0.0,
                                  1.0};
            Eigen::Vector3d world = tf.block<3, 4>(0, 0) * local;
            ring.points.push_back(to_point3(world.x(), world.y(), world.z()));
        }
        entity.lines.push_back(std::move(ring));
        update.entities.push_back(std::move(entity));
    }
    return update;
}

fg::SceneUpdate to_robot_scene(const RobotDescriptionsStamped &robots) {
    fg::SceneUpdate update;
    // Same clear-then-draw rule as the hazards: a robot that dropped out of the filter must
    // not keep its last box on screen.
    update.deletions.push_back(make_delete_all(robots.header));
    int keypoint_counter = 0;
    for (size_t i = 0; i < robots.descriptions.size(); ++i) {
        const auto &robot = robots.descriptions[i];
        const int robot_id = static_cast<int>(magic_enum::enum_index(robot.frame_id).value_or(i));

        const fg::Color robot_color = to_color(get_color_for_index(robot.group), 0.7f);
        const fg::Color solid_color = to_color(get_color_for_index(robot.group), 1.0f);

        // Body
        {
            fg::SceneEntity entity = make_entity(robots.header, "robot_bounds", robot_id);
            fg::CubePrimitive cube;
            cube.pose = to_pose(robot.pose);
            cube.size = to_vector3(robot.size.x, robot.size.y, robot.size.z);
            cube.color = robot_color;
            entity.cubes.push_back(std::move(cube));
            update.entities.push_back(std::move(entity));
        }

        // Heading arrow along the robot's x-axis, 1.5x the body length, fixed diameter.
        {
            fg::SceneEntity entity = make_entity(robots.header, "robot_poses", robot_id);
            entity.arrows.push_back(make_pose_arrow(to_pose(robot.pose), robot.size.x * 1.5, 0.015,
                                                    0.015, solid_color));
            update.entities.push_back(std::move(entity));
        }

        // Label above the robot
        {
            fg::SceneEntity entity = make_entity(robots.header, "robot_labels", robot_id);
            fg::TextPrimitive text;
            text.pose = pose_at(robot.pose.position.x, robot.pose.position.y,
                                robot.pose.position.z + robot.size.z * 0.6);
            text.billboard = true;
            text.font_size = 0.1;
            text.scale_invariant = false;
            text.color = solid_color;
            text.text = enum_to_string_lower(robot.frame_id) + " (" +
                        enum_to_string_lower(robot.label) + ")";
            entity.texts.push_back(std::move(text));
            update.entities.push_back(std::move(entity));
        }

        for (const Position &keypoint : robot.keypoints) {
            fg::SceneEntity entity =
                make_entity(robots.header, "robot_keypoints", keypoint_counter++);
            fg::SpherePrimitive sphere;
            sphere.pose = pose_at(keypoint.x, keypoint.y, keypoint.z);
            sphere.size = to_vector3(0.02, 0.02, 0.02);
            sphere.color = solid_color;
            entity.spheres.push_back(std::move(sphere));
            update.entities.push_back(std::move(entity));
        }

        if (robot.keypoints.size() > 1) {
            fg::SceneEntity entity = make_entity(robots.header, "robot_keypoint_lines", robot_id);
            fg::LinePrimitive line =
                make_line(fg::LinePrimitive::LineType::LINE_STRIP, 0.005, solid_color);
            for (const Position &keypoint : robot.keypoints) {
                line.points.push_back(to_point3(keypoint.x, keypoint.y, keypoint.z));
            }
            entity.lines.push_back(std::move(line));
            update.entities.push_back(std::move(entity));
        }
    }
    return update;
}

fg::SceneUpdate to_navigation_scene(const NavigationVisualization &nav) {
    fg::SceneUpdate update;
    int id = 0;
    constexpr double kLifetime = 0.1;

    const RobotDescription *our_robot = nullptr;
    for (const auto &r : nav.robots.descriptions) {
        if (r.frame_id == FrameId::OUR_ROBOT_1) {
            our_robot = &r;
            break;
        }
    }

    // Pursuit line from our robot to the target, and a crosshair on the target.
    if (nav.path.has_value()) {
        const auto &path = nav.path.value();
        {
            fg::SceneEntity entity = make_entity(nav.header, "nav_pursuit_line", id++);
            fg::LinePrimitive line = make_line(fg::LinePrimitive::LineType::LINE_STRIP, 0.005,
                                               to_color(1.0f, 0.6f, 0.0f, 0.8f));
            line.points.push_back(to_point3(path.our_x, path.our_y, 0.01));
            line.points.push_back(to_point3(path.target_x, path.target_y, 0.01));
            entity.lines.push_back(std::move(line));
            update.entities.push_back(std::move(entity));
        }
        {
            constexpr double cross_size = 0.04;
            fg::SceneEntity entity = make_entity(nav.header, "nav_target", id++);
            fg::LinePrimitive cross = make_line(fg::LinePrimitive::LineType::LINE_LIST, 0.006,
                                                to_color(1.0f, 0.2f, 0.2f, 1.0f));
            const double tx = path.target_x, ty = path.target_y, tz = 0.01;
            cross.points.push_back(to_point3(tx - cross_size, ty, tz));
            cross.points.push_back(to_point3(tx + cross_size, ty, tz));
            cross.points.push_back(to_point3(tx, ty - cross_size, tz));
            cross.points.push_back(to_point3(tx, ty + cross_size, tz));
            entity.lines.push_back(std::move(cross));
            update.entities.push_back(std::move(entity));
        }
    }

    if (our_robot) {
        const Pose2D pose = pose_to_pose2d(our_robot->pose);
        const double cos_yaw = std::cos(pose.yaw);
        const double sin_yaw = std::sin(pose.yaw);

        // Commanded linear velocity, rotated into the field frame and scaled for display.
        {
            const double vx_field = nav.command.linear_x * cos_yaw - nav.command.linear_y * sin_yaw;
            const double vy_field = nav.command.linear_x * sin_yaw + nav.command.linear_y * cos_yaw;
            const double v_mag = std::sqrt(vx_field * vx_field + vy_field * vy_field);
            constexpr double arrow_scale = 0.15;
            if (v_mag > 0.01) {
                fg::SceneEntity entity = make_entity(nav.header, "nav_velocity", id++, kLifetime);
                const Eigen::Vector3d start(pose.x, pose.y, 0.02);
                const Eigen::Vector3d end(pose.x + vx_field * arrow_scale,
                                          pose.y + vy_field * arrow_scale, 0.02);
                entity.arrows.push_back(make_segment_arrow(start, end, 0.008, 0.014, 0.012,
                                                           to_color(0.2f, 1.0f, 0.2f, 0.9f)));
                update.entities.push_back(std::move(entity));
            }
        }

        // Commanded angular velocity as an arc with an arrowhead at its tip.
        {
            const double omega = nav.command.angular_z;
            if (std::abs(omega) > 0.05) {
                const fg::Color arc_color = to_color(0.4f, 0.6f, 1.0f, 0.9f);
                constexpr double arc_radius = 0.06;
                const double sweep = std::clamp(omega * M_PI * 0.5, -M_PI, M_PI);
                constexpr int segments = 16;

                fg::SceneEntity arc_entity =
                    make_entity(nav.header, "nav_angular", id++, kLifetime);
                fg::LinePrimitive arc =
                    make_line(fg::LinePrimitive::LineType::LINE_STRIP, 0.004, arc_color);
                for (int s = 0; s <= segments; ++s) {
                    const double frac = static_cast<double>(s) / segments;
                    const double a = pose.yaw + M_PI_2 + frac * sweep;
                    arc.points.push_back(to_point3(pose.x + arc_radius * std::cos(a),
                                                   pose.y + arc_radius * std::sin(a), 0.02));
                }
                arc_entity.lines.push_back(std::move(arc));

                const double tip_angle = pose.yaw + M_PI_2 + sweep;
                const double tangent = tip_angle + (omega > 0 ? M_PI_2 : -M_PI_2);
                constexpr double head_len = 0.012;
                fg::SceneEntity head_entity =
                    make_entity(nav.header, "nav_angular_head", id++, kLifetime);
                fg::LinePrimitive head =
                    make_line(fg::LinePrimitive::LineType::LINE_LIST, 0.004, arc_color);
                const fg::Point3 tip = to_point3(pose.x + arc_radius * std::cos(tip_angle),
                                                 pose.y + arc_radius * std::sin(tip_angle), 0.02);
                head.points.push_back(tip);
                head.points.push_back(to_point3(tip.x + head_len * std::cos(tangent + 2.5),
                                                tip.y + head_len * std::sin(tangent + 2.5), 0.02));
                head.points.push_back(tip);
                head.points.push_back(to_point3(tip.x + head_len * std::cos(tangent - 2.5),
                                                tip.y + head_len * std::sin(tangent - 2.5), 0.02));
                head_entity.lines.push_back(std::move(head));

                update.entities.push_back(std::move(arc_entity));
                update.entities.push_back(std::move(head_entity));
            }
        }
    }

    // Delete every namespace absent this tick so stale visuals clear.
    const char *all_ns[] = {"nav_pursuit_line", "nav_target", "nav_velocity", "nav_angular",
                            "nav_angular_head"};
    for (const char *ns : all_ns) {
        const std::string prefix = std::string(ns) + "/";
        const bool present =
            std::any_of(update.entities.begin(), update.entities.end(),
                        [&](const fg::SceneEntity &e) { return e.id.rfind(prefix, 0) == 0; });
        if (!present) update.deletions.push_back(make_deletion(nav.header, ns, 0));
    }
    return update;
}

}  // namespace foxglove_adapters
}  // namespace auto_battlebot
