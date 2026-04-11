#include "lvgl_ui_overlay.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <opencv2/imgproc.hpp>

#include "data_structures/pose.hpp"
#include "enums/label.hpp"
#include "label_utils.hpp"
#include "lvgl_ui_services.hpp"
#include "transform_utils.hpp"

namespace auto_battlebot::ui_internal {
namespace {

cv::Scalar to_cv_bgr(Label label) {
    ColorRGBf color = get_color_for_label(label);
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

}  // namespace

void draw_robot_pose_arrows(cv::Mat &image, const RobotDescriptionsStamped &robots,
                            const FieldDescription &field, const CameraInfo &camera_info) {
    for (const auto &robot : robots.descriptions) {
        Pose2D pose2d = pose_to_pose2d(robot.pose);
        const double arrow_len_m = std::max(robot.size.x * 1.5, 0.1);
        const double z = robot.pose.position.z + 0.01;

        cv::Point start_px;
        cv::Point end_px;
        if (!project_field_point_to_pixel(field, camera_info, pose2d.x, pose2d.y, z, start_px))
            continue;
        if (!project_field_point_to_pixel(
                field, camera_info, pose2d.x + arrow_len_m * std::cos(pose2d.yaw),
                pose2d.y + arrow_len_m * std::sin(pose2d.yaw), z, end_px)) {
            continue;
        }
        cv::arrowedLine(image, start_px, end_px, to_cv_bgr(robot.label), 2, cv::LINE_AA, 0, 0.25);
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
        draw_robot_pose_arrows(image, robots, field, camera_info);
        draw_target_path_overlay(image, path, field, camera_info);
    }
};

std::unique_ptr<IDebugOverlayRenderer> make_debug_overlay_renderer() {
    return std::make_unique<OpenCvDebugOverlayRenderer>();
}

}  // namespace auto_battlebot::ui_internal
