#include "keypoint_model/sim_keypoint_model.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <utility>

namespace auto_battlebot {
namespace {

/** Pixel of a field-frame floor point, or nullopt when it is behind the camera or off the image. */
std::optional<Eigen::Vector2d> project_floor_point(const Eigen::Matrix4d &camera_from_field,
                                                   const CameraInfo &camera_info, double x,
                                                   double y) {
    const Eigen::Vector4d p = camera_from_field * Eigen::Vector4d(x, y, 0.0, 1.0);
    if (p.z() <= 1e-9) return std::nullopt;
    const cv::Mat &k = camera_info.intrinsics;
    const double u = k.at<double>(0, 0) * p.x() / p.z() + k.at<double>(0, 2);
    const double v = k.at<double>(1, 1) * p.y() / p.z() + k.at<double>(1, 2);
    if (u < 0.0 || v < 0.0 || u > camera_info.width - 1 || v > camera_info.height - 1) {
        return std::nullopt;
    }
    return Eigen::Vector2d(u, v);
}

}  // namespace

ModelResultStamped sim_measurements_to_keypoints(const SimMeasurementFrame &frame,
                                                 const std::vector<Label> &labels,
                                                 double keypoint_offset_m,
                                                 const Eigen::Matrix4d &camera_from_field,
                                                 const CameraInfo &camera_info,
                                                 const Header &image_header) {
    ModelResultStamped result;
    result.header = image_header;
    if (!frame.delivered || camera_info.intrinsics.empty()) return result;
    result.header.stamp = frame.capture_stamp;

    const size_t slots = std::min(frame.robots.size(), labels.size());
    for (size_t i = 0; i < slots; ++i) {
        const SimRobotMeasurement &robot = frame.robots[i];
        if (!robot.measured) continue;
        const auto keypoint_labels = SimKeypointModelConfiguration::front_back_labels(labels[i]);
        if (!keypoint_labels) continue;

        const double dx = keypoint_offset_m * std::cos(robot.pose.yaw);
        const double dy = keypoint_offset_m * std::sin(robot.pose.yaw);
        const auto front = project_floor_point(camera_from_field, camera_info, robot.pose.x + dx,
                                               robot.pose.y + dy);
        const auto back = project_floor_point(camera_from_field, camera_info, robot.pose.x - dx,
                                              robot.pose.y - dy);
        if (!front || !back) continue;

        for (const auto &[pixel, keypoint_label] : {std::pair{*front, keypoint_labels->first},
                                                    std::pair{*back, keypoint_labels->second}}) {
            Keypoint keypoint;
            keypoint.label = labels[i];
            keypoint.keypoint_label = keypoint_label;
            keypoint.x = pixel.x();
            keypoint.y = pixel.y();
            keypoint.confidence = 1.0;
            keypoint.detection_index = static_cast<int>(i);
            keypoint.height_above_plane = 0.0;
            result.keypoints.push_back(keypoint);
        }
    }
    return result;
}

SimKeypointModel::SimKeypointModel(const SimKeypointModelConfiguration &config) : config_(config) {}

ModelResultStamped SimKeypointModel::update(RgbImage image, const CameraInfo &camera_info) {
    // Looked up here rather than in the constructor: SimRgbdCamera configures the connection, and
    // the factories do not promise which of the two is built first.
    if (!connection_) connection_ = SimConnection::instance();
    return sim_measurements_to_keypoints(
        connection_->last_measurements(), config_.labels, config_.keypoint_offset_m,
        connection_->last_camera_from_field(), camera_info, image.header);
}

}  // namespace auto_battlebot
