#include "lvgl_platform_bound/lvgl_ui_overlay.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <magic_enum.hpp>
#include <opencv2/imgproc.hpp>
#include <string>

#include "data_structures/pose.hpp"
#include "enums/label.hpp"
#include "hazards/hazard_geometry.hpp"
#include "label_utils.hpp"
#include "lvgl_platform_bound/lvgl_ui_services.hpp"
#include "transform_utils.hpp"

namespace auto_battlebot::ui_internal {
namespace {

constexpr int MARKER_THICKNESS = 2;
constexpr int MARKER_BORDER_THICKNESS = 2;
constexpr int MIN_DOT_RADIUS_PX = 2;
constexpr int MAX_DOT_RADIUS_PX = 20;
const cv::Scalar MARKER_BORDER_COLOR(255, 255, 255);

cv::Scalar to_cv_bgr(Group group) {
    ColorRGBf color = get_color_for_index(group);
    return cv::Scalar(color.b * 255.0f, color.g * 255.0f, color.r * 255.0f);
}

bool project_field_point_to_pixel(const FieldDescription &field, const CameraInfo &camera_info,
                                  double x, double y, double z, cv::Point &out) {
    if (field.tf_camera_from_fieldcenter.tf.rows() < 3 ||
        field.tf_camera_from_fieldcenter.tf.cols() < 4) {
        return false;
    }
    if (camera_info.intrinsics.rows != 3 || camera_info.intrinsics.cols != 3) {
        return false;
    }
    if (camera_info.width <= 0 || camera_info.height <= 0) return false;

    const auto &tf = field.tf_camera_from_fieldcenter.tf;
    const double cx = tf(0, 0) * x + tf(0, 1) * y + tf(0, 2) * z + tf(0, 3);
    const double cy = tf(1, 0) * x + tf(1, 1) * y + tf(1, 2) * z + tf(1, 3);
    const double cz = tf(2, 0) * x + tf(2, 1) * y + tf(2, 2) * z + tf(2, 3);
    if (cz <= 1e-6) return false;

    const double fx = camera_info.intrinsics.at<double>(0, 0);
    const double fy = camera_info.intrinsics.at<double>(1, 1);
    const double px0 = camera_info.intrinsics.at<double>(0, 2);
    const double py0 = camera_info.intrinsics.at<double>(1, 2);

    const double u = fx * (cx / cz) + px0;
    const double v = fy * (cy / cz) + py0;
    if (!std::isfinite(u) || !std::isfinite(v)) return false;
    out = cv::Point(static_cast<int>(std::lround(u)), static_cast<int>(std::lround(v)));
    return true;
}

/* Bordered arrow: a thicker border arrow underneath, the group-colored arrow on top. */
void draw_bordered_arrow(cv::Mat &image, const cv::Point &start_px, const cv::Point &end_px,
                         const cv::Scalar &color) {
    cv::arrowedLine(image, start_px, end_px, MARKER_BORDER_COLOR,
                    MARKER_THICKNESS + 2 * MARKER_BORDER_THICKNESS, cv::LINE_AA, 0, 0.25);
    cv::arrowedLine(image, start_px, end_px, color, MARKER_THICKNESS, cv::LINE_AA, 0, 0.25);
}

/* Hollow bordered dot sized from the robot footprint, clamped so it stays legible at any range. */
void draw_bordered_dot(cv::Mat &image, const cv::Point &center_px, int radius_px,
                       const cv::Scalar &color) {
    cv::circle(image, center_px, radius_px, MARKER_BORDER_COLOR,
               MARKER_THICKNESS + 2 * MARKER_BORDER_THICKNESS, cv::LINE_AA);
    cv::circle(image, center_px, radius_px, color, MARKER_THICKNESS, cv::LINE_AA);
}

/* Ground-plane circle projected through the camera, so it keeps its perspective at any range.
 * Kept separate from the hazard ring's own loop: this is display only and must not change how a
 * keep-out is drawn. */
void draw_field_circle(cv::Mat &image, const FieldDescription &field, const CameraInfo &camera_info,
                       double center_x, double center_y, double radius_m, double z,
                       const cv::Scalar &color) {
    constexpr int kCircleSegments = 24;
    if (radius_m <= 0.0) return;
    std::vector<cv::Point> ring;
    ring.reserve(kCircleSegments);
    for (int step = 0; step < kCircleSegments; ++step) {
        const double angle = 2.0 * M_PI * step / kCircleSegments;
        cv::Point point;
        if (!project_field_point_to_pixel(field, camera_info, center_x + radius_m * std::cos(angle),
                                          center_y + radius_m * std::sin(angle), z, point)) {
            return;
        }
        ring.push_back(point);
    }
    if (ring.size() < 3) return;
    cv::polylines(image, ring, true, MARKER_BORDER_COLOR, MARKER_THICKNESS + 1, cv::LINE_AA);
    cv::polylines(image, ring, true, color, MARKER_THICKNESS, cv::LINE_AA);
}

}  // namespace

void draw_robot_markers(cv::Mat &image, const RobotDescriptionsStamped &robots,
                        const FieldDescription &field, const CameraInfo &camera_info) {
    for (const auto &robot : robots.descriptions) {
        Pose2D pose2d = pose_to_pose2d(robot.pose);
        const double z = robot.pose.position.z + 0.01;

        cv::Point start_px;
        if (!project_field_point_to_pixel(field, camera_info, pose2d.x, pose2d.y, z, start_px))
            continue;

        // The robot's own footprint radius, the same half-diagonal the hazard assembler inflates
        // by. A tracked keep-out ring is this circle for the hazard plus this circle for our robot
        // plus the margin, which is why it reads as roughly double the robot; drawing both makes
        // that sum visible instead of leaving it implicit.
        draw_field_circle(image, field, camera_info, pose2d.x, pose2d.y, half_diagonal(robot.size),
                          z, to_cv_bgr(robot.group));

        if (robot.group == Group::OURS) {
            const double arrow_len_m = std::max(robot.size.x * 1.5, 0.1);
            cv::Point end_px;
            if (!project_field_point_to_pixel(
                    field, camera_info, pose2d.x + arrow_len_m * std::cos(pose2d.yaw),
                    pose2d.y + arrow_len_m * std::sin(pose2d.yaw), z, end_px)) {
                continue;
            }
            draw_bordered_arrow(image, start_px, end_px, to_cv_bgr(robot.group));
        } else {
            const double radius_m = std::max(std::min(robot.size.x, robot.size.y) * 0.25, 0.025);
            cv::Point edge_px;
            if (!project_field_point_to_pixel(field, camera_info, pose2d.x + radius_m, pose2d.y, z,
                                              edge_px)) {
                continue;
            }
            const int radius_px =
                std::clamp(static_cast<int>(std::lround(cv::norm(edge_px - start_px))),
                           MIN_DOT_RADIUS_PX, MAX_DOT_RADIUS_PX);
            draw_bordered_dot(image, start_px, radius_px, to_cv_bgr(robot.group));
        }

        if (robot.frame_id == FrameId::EMPTY) continue;
        const std::string frame_label(magic_enum::enum_name(robot.frame_id));
        const cv::Point label_org(start_px.x + 8, start_px.y - 8);
        cv::putText(image, frame_label, label_org + cv::Point(1, 1), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(0, 0, 0), 2, cv::LINE_AA);
        cv::putText(image, frame_label, label_org, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    }
}

void draw_field_border(cv::Mat &image, const FieldDescription &field,
                       const CameraInfo &camera_info) {
    const double hx = field.size.size.x / 2.0;
    const double hy = field.size.size.y / 2.0;
    const std::array<cv::Point3d, 4> corners = {{
        {-hx, -hy, 0.0},
        {hx, -hy, 0.0},
        {hx, hy, 0.0},
        {-hx, hy, 0.0},
    }};

    std::array<cv::Point, 4> projected;
    for (size_t i = 0; i < corners.size(); ++i) {
        if (!project_field_point_to_pixel(field, camera_info, corners[i].x, corners[i].y,
                                          corners[i].z, projected[i])) {
            return;
        }
    }

    for (size_t i = 0; i < projected.size(); ++i) {
        const cv::Point &a = projected[i];
        const cv::Point &b = projected[(i + 1) % projected.size()];
        cv::line(image, a, b, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    }
}

void draw_hazard_overlay(cv::Mat &image, const FieldDescription &field,
                         const CameraInfo &camera_info) {
    for (const auto &hazard : field.hazards) {
        // Only the hazard itself is drawn. The keep-out ring it gets inflated to is our robot's
        // half-diagonal plus the configured margin further out, which the eye can judge from the
        // robot circles already on screen; drawing it as well was more clutter than information.
        //
        // Amber for arena geometry, magenta for a live neutral track, so the driver can tell a
        // hazard that will never move from one following the house bot around. A tracked hazard's
        // circle coincides with that robot's own marker circle by construction; it is still drawn
        // here so a hazard held through a track dropout does not vanish.
        const cv::Scalar color = hazard.source == HazardSource::STATIC ? cv::Scalar(0, 165, 255)
                                                                       : cv::Scalar(255, 0, 255);
        draw_field_circle(image, field, camera_info, hazard.center.x, hazard.center.y,
                          hazard.object_radius, 0.01, color);
    }
}

void draw_target_path_overlay(cv::Mat &image, const std::optional<NavigationPathSegment> &path,
                              const FieldDescription &field, const CameraInfo &camera_info) {
    if (!path.has_value()) return;

    cv::Point our_px;
    cv::Point target_px;
    if (project_field_point_to_pixel(field, camera_info, path->our_x, path->our_y, 0.01, our_px) &&
        project_field_point_to_pixel(field, camera_info, path->target_x, path->target_y, 0.01,
                                     target_px)) {
        cv::line(image, our_px, target_px, cv::Scalar(0, 153, 255), 2, cv::LINE_AA);
    }

    if (project_field_point_to_pixel(field, camera_info, path->target_x, path->target_y, 0.01,
                                     target_px)) {
        constexpr int cross_half = 9;
        cv::line(image, cv::Point(target_px.x - cross_half, target_px.y - cross_half),
                 cv::Point(target_px.x + cross_half, target_px.y + cross_half),
                 cv::Scalar(51, 51, 255), 2, cv::LINE_AA);
        cv::line(image, cv::Point(target_px.x - cross_half, target_px.y + cross_half),
                 cv::Point(target_px.x + cross_half, target_px.y - cross_half),
                 cv::Scalar(51, 51, 255), 2, cv::LINE_AA);
    }
}

class OpenCvDebugOverlayRenderer : public IDebugOverlayRenderer {
   public:
    void render(cv::Mat &image, const RobotDescriptionsStamped &robots,
                const std::optional<NavigationPathSegment> &path, const FieldDescription &field,
                const CameraInfo &camera_info) override {
        draw_field_border(image, field, camera_info);
        draw_hazard_overlay(image, field, camera_info);
        draw_robot_markers(image, robots, field, camera_info);
        draw_target_path_overlay(image, path, field, camera_info);
    }
};

std::unique_ptr<IDebugOverlayRenderer> make_debug_overlay_renderer() {
    return std::make_unique<OpenCvDebugOverlayRenderer>();
}

}  // namespace auto_battlebot::ui_internal
