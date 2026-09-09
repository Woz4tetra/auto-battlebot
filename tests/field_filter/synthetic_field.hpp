#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <vector>

namespace auto_battlebot {
namespace testing_support {
/** e-CAM25_CUONX at 1920x1200 with the focal length backed out of the quoted vertical field of
 *  view, which is the axis least inflated by barrel distortion. */
inline Eigen::Matrix3d test_intrinsics() {
    Eigen::Matrix3d intrinsics = Eigen::Matrix3d::Identity();
    intrinsics(0, 0) = 1006.0;
    intrinsics(1, 1) = 1006.0;
    intrinsics(0, 2) = 960.0;
    intrinsics(1, 2) = 600.0;
    return intrinsics;
}

inline cv::Mat test_intrinsics_cv() {
    const Eigen::Matrix3d intrinsics = test_intrinsics();
    cv::Mat out(3, 3, CV_64F);
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            out.at<double>(row, col) = intrinsics(row, col);
        }
    }
    return out;
}

/**
 * @brief Camera looking at the field centre from `range` metres at `tilt_deg` off vertical.
 *
 * Field z points away from the camera, into the floor, which is the convention the outline fit
 * produces, so "up" in the field frame is -z. tilt 0 is straight overhead and 90 would be level
 * with the mat.
 */
inline Eigen::Matrix4d camera_pose(double range, double tilt_deg, double yaw_deg) {
    const double tilt = tilt_deg * M_PI / 180.0;
    const double height = range * std::cos(tilt);
    const double standoff = range * std::sin(tilt);

    const Eigen::Vector3d position(0.0, -standoff, -height);
    const Eigen::Vector3d forward = (-position).normalized();
    const Eigen::Vector3d world_up(0.0, 0.0, -1.0);
    const Eigen::Vector3d right = world_up.cross(forward).normalized();
    const Eigen::Vector3d down = forward.cross(right);

    Eigen::Matrix3d rotation_field_from_camera;
    rotation_field_from_camera.col(0) = right;
    rotation_field_from_camera.col(1) = down;
    rotation_field_from_camera.col(2) = forward;

    // Spin the field about its own vertical, so the rectangle is not axis-aligned in the image.
    const double yaw = yaw_deg * M_PI / 180.0;
    Eigen::Matrix3d spin = Eigen::Matrix3d::Identity();
    spin(0, 0) = std::cos(yaw);
    spin(0, 1) = -std::sin(yaw);
    spin(1, 0) = std::sin(yaw);
    spin(1, 1) = std::cos(yaw);

    Eigen::Matrix4d tf_field_from_camera = Eigen::Matrix4d::Identity();
    tf_field_from_camera.block<3, 3>(0, 0) = spin * rotation_field_from_camera;
    tf_field_from_camera.block<3, 1>(0, 3) = spin * position;
    return tf_field_from_camera.inverse();
}

/** Outline of a rectangle in the field plane, with corners rounded in the world rather than in
 *  the image, because that is where the mat's rounding actually is. */
inline std::vector<Eigen::Vector3d> field_outline_points(double size_x, double size_y,
                                                         double corner_radius, int arc_steps = 24) {
    const double half_x = size_x / 2.0;
    const double half_y = size_y / 2.0;
    const double radius = std::min(corner_radius, std::min(half_x, half_y) * 0.9);
    std::vector<Eigen::Vector3d> points;
    // Same winding as the object corners: (-,-) -> (-,+) -> (+,+) -> (+,-).
    const std::vector<std::pair<double, double>> corners = {
        {-half_x, -half_y}, {-half_x, half_y}, {half_x, half_y}, {half_x, -half_y}};
    for (size_t i = 0; i < corners.size(); ++i) {
        const auto &[x, y] = corners[i];
        if (radius <= 1e-9) {
            points.emplace_back(x, y, 0.0);
            continue;
        }
        const double centre_x = x - std::copysign(radius, x);
        const double centre_y = y - std::copysign(radius, y);
        // Quarter arc, swept from the side arriving at this corner to the side leaving it, so the
        // points stay in winding order.
        const double base = std::atan2(std::copysign(1.0, y), std::copysign(1.0, x));
        for (int step = 0; step <= arc_steps; ++step) {
            const double angle = base + M_PI / 4.0 - M_PI / 2.0 * step / arc_steps;
            points.emplace_back(centre_x + radius * std::cos(angle),
                                centre_y + radius * std::sin(angle), 0.0);
        }
    }
    return points;
}

/** Project field-plane points into the image. */
inline std::vector<cv::Point> project(const std::vector<Eigen::Vector3d> &points,
                                      const Eigen::Matrix4d &tf_camera_from_fieldcenter,
                                      const Eigen::Matrix3d &intrinsics) {
    const Eigen::Matrix3d rotation = tf_camera_from_fieldcenter.block<3, 3>(0, 0);
    const Eigen::Vector3d translation = tf_camera_from_fieldcenter.block<3, 1>(0, 3);
    std::vector<cv::Point> projected;
    projected.reserve(points.size());
    for (const auto &point : points) {
        const Eigen::Vector3d camera = rotation * point + translation;
        const Eigen::Vector3d image = intrinsics * camera;
        projected.emplace_back(static_cast<int>(std::lround(image.x() / image.z())),
                               static_cast<int>(std::lround(image.y() / image.z())));
    }
    return projected;
}

/** A filled field mask as the segmentation model would produce it. */
inline cv::Mat render_field_mask(const Eigen::Matrix4d &tf_camera_from_fieldcenter,
                                 const Eigen::Matrix3d &intrinsics, cv::Size image_size,
                                 double size_x, double size_y, double corner_radius = 0.0) {
    cv::Mat mask = cv::Mat::zeros(image_size, CV_8UC1);
    const auto points = project(field_outline_points(size_x, size_y, corner_radius),
                                tf_camera_from_fieldcenter, intrinsics);
    cv::fillPoly(mask, std::vector<std::vector<cv::Point>>{points}, cv::Scalar(255));
    return mask;
}
}  // namespace testing_support
}  // namespace auto_battlebot
