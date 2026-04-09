#include "robot_filter/robot_mask_detector.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include <opencv2/imgproc.hpp>

namespace auto_battlebot {

RobotMaskDetector::RobotMaskDetector(const RobotMaskDetectorConfig &config) : config_(config) {}

void RobotMaskDetector::reset() { tracked_blobs_.clear(); }

std::vector<RobotDescription> RobotMaskDetector::detect(
    const cv::Mat &mask, const FieldDescription &field, const CameraInfo &camera_info,
    const std::vector<RobotDescription> &own_robot_measurements, double timestamp) {
    if (mask.empty() || field.child_frame_id == FrameId::EMPTY) {
        return {};
    }

    auto intr = scale_intrinsics(camera_info, mask.cols, mask.rows);
    cv::Rect field_roi = compute_field_roi(field, intr);
    if (field_roi.area() == 0) {
        return {};
    }

    cv::Mat detection_mask = build_detection_mask(mask, field_roi);

    Eigen::Matrix4d tf_cam_from_field = field.tf_camera_from_fieldcenter.tf;
    mask_own_robots(detection_mask, own_robot_measurements, tf_cam_from_field, intr);

    Eigen::Matrix4d tf_field_from_cam = tf_cam_from_field.inverse();
    auto candidates = extract_blob_candidates(detection_mask, intr, tf_field_from_cam, field);
    auto persistent = update_tracked_blobs(candidates, timestamp);
    return candidates_to_measurements(persistent);
}

RobotMaskDetector::ScaledIntrinsics RobotMaskDetector::scale_intrinsics(
    const CameraInfo &camera_info, int mask_width, int mask_height) {
    double sx = static_cast<double>(mask_width) / camera_info.width;
    double sy = static_cast<double>(mask_height) / camera_info.height;
    const cv::Mat &K = camera_info.intrinsics;
    double fx = K.at<double>(0, 0);
    double fy = K.at<double>(1, 1);
    double cx = K.at<double>(0, 2);
    double cy = K.at<double>(1, 2);
    return {fx * sx, fy * sy, cx * sx, cy * sy, mask_width, mask_height};
}

cv::Rect RobotMaskDetector::compute_field_roi(const FieldDescription &field,
                                              const ScaledIntrinsics &intr) const {
    Eigen::Matrix4d tf_cam_from_field = field.tf_camera_from_fieldcenter.tf;

    double half_x = field.size.size.x / 2.0;
    double half_y = field.size.size.y / 2.0;

    std::array<Eigen::Vector4d, 4> corners = {
        Eigen::Vector4d(-half_x, -half_y, 0, 1), Eigen::Vector4d(half_x, -half_y, 0, 1),
        Eigen::Vector4d(half_x, half_y, 0, 1), Eigen::Vector4d(-half_x, half_y, 0, 1)};

    int min_u = intr.width, max_u = 0;
    int min_v = intr.height, max_v = 0;

    for (const auto &corner : corners) {
        Eigen::Vector4d p_cam = tf_cam_from_field * corner;
        if (p_cam.z() <= 0) continue;
        int u = static_cast<int>(intr.fx * p_cam.x() / p_cam.z() + intr.cx);
        int v = static_cast<int>(intr.fy * p_cam.y() / p_cam.z() + intr.cy);
        min_u = std::min(min_u, u);
        max_u = std::max(max_u, u);
        min_v = std::min(min_v, v);
        max_v = std::max(max_v, v);
    }

    min_u = std::max(0, min_u);
    min_v = std::max(0, min_v);
    max_u = std::min(intr.width - 1, max_u);
    max_v = std::min(intr.height - 1, max_v);

    if (min_u >= max_u || min_v >= max_v) {
        return cv::Rect(0, 0, 0, 0);
    }
    return cv::Rect(min_u, min_v, max_u - min_u, max_v - min_v);
}

cv::Mat RobotMaskDetector::build_detection_mask(const cv::Mat &mask,
                                                const cv::Rect &field_roi) const {
    cv::Mat result = cv::Mat::zeros(mask.size(), CV_8UC1);

    // Mask has class indices (0=background, 1=robot).
    // Threshold so robot pixels = 255, background = 0.
    cv::Mat binary;
    if (mask.type() == CV_8UC1) {
        cv::threshold(mask, binary, 0, 255, cv::THRESH_BINARY);
    } else {
        cv::Mat temp;
        mask.convertTo(temp, CV_8UC1);
        cv::threshold(temp, binary, 0, 255, cv::THRESH_BINARY);
    }

    // Restrict to field ROI.
    cv::Mat roi_region = result(field_roi);
    cv::Mat mask_roi = binary(field_roi);
    mask_roi.copyTo(roi_region);

    if (config_.morph_kernel_size > 0) {
        cv::Mat kernel = cv::getStructuringElement(
            cv::MORPH_ELLIPSE,
            cv::Size(config_.morph_kernel_size, config_.morph_kernel_size));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    }

    return mask;
}

void RobotMaskDetector::mask_own_robots(cv::Mat &mask,
                                        const std::vector<RobotDescription> &own_robots,
                                        const Eigen::Matrix4d &tf_cam_from_field,
                                        const ScaledIntrinsics &intr) const {
    for (const auto &robot : own_robots) {
        Eigen::Vector4d pos_field(robot.pose.position.x, robot.pose.position.y,
                                  robot.pose.position.z, 1.0);
        Eigen::Vector4d pos_cam = tf_cam_from_field * pos_field;
        if (pos_cam.z() <= 0) continue;

        int cu = static_cast<int>(intr.fx * pos_cam.x() / pos_cam.z() + intr.cx);
        int cv_coord = static_cast<int>(intr.fy * pos_cam.y() / pos_cam.z() + intr.cy);

        double robot_extent = std::max(robot.size.x, robot.size.y);
        double pixel_radius = (intr.fx * robot_extent) / pos_cam.z();
        int r = static_cast<int>(pixel_radius * 1.5);

        cv::Point center(cu, cv_coord);
        cv::circle(mask, center, r, cv::Scalar(0), cv::FILLED);
    }
}

std::vector<RobotMaskDetector::BlobCandidate> RobotMaskDetector::extract_blob_candidates(
    const cv::Mat &mask, const ScaledIntrinsics &intr,
    const Eigen::Matrix4d &tf_field_from_cam, const FieldDescription &field) const {
    cv::Mat labels, stats, centroids;
    int n_labels = cv::connectedComponentsWithStats(mask, labels, stats, centroids);

    std::vector<BlobCandidate> candidates;

    for (int i = 1; i < n_labels; ++i) {
        int area = stats.at<int>(i, cv::CC_STAT_AREA);
        if (area < config_.min_blob_area_pixels) continue;

        double cx_blob = centroids.at<double>(i, 0);
        double cy_blob = centroids.at<double>(i, 1);

        Eigen::Vector3d ray_dir_cam(
            (cx_blob - intr.cx) / intr.fx, (cy_blob - intr.cy) / intr.fy, 1.0);
        ray_dir_cam.normalize();

        Eigen::Vector3d ray_dir_field =
            tf_field_from_cam.block<3, 3>(0, 0) * ray_dir_cam;
        Eigen::Vector3d ray_origin_field = tf_field_from_cam.block<3, 1>(0, 3);

        Eigen::Vector3d field_plane_normal(0, 0, 1);
        Eigen::Vector3d field_plane_point(0, 0, 0);

        Eigen::Vector3d hit =
            ray_plane_intersect(ray_origin_field, ray_dir_field, field_plane_point, field_plane_normal);

        double half_x = field.size.size.x / 2.0;
        double half_y = field.size.size.y / 2.0;
        if (std::abs(hit.x()) > half_x * 1.2 || std::abs(hit.y()) > half_y * 1.2) {
            continue;
        }

        candidates.push_back({hit, 0.0});
    }

    return candidates;
}

Eigen::Vector3d RobotMaskDetector::ray_plane_intersect(const Eigen::Vector3d &ray_origin,
                                                       const Eigen::Vector3d &ray_dir,
                                                       const Eigen::Vector3d &plane_point,
                                                       const Eigen::Vector3d &plane_normal) const {
    double denom = plane_normal.dot(ray_dir);
    if (std::abs(denom) < 1e-10) {
        return ray_origin;
    }
    double t = plane_normal.dot(plane_point - ray_origin) / denom;
    return ray_origin + t * ray_dir;
}

std::vector<RobotMaskDetector::BlobCandidate> RobotMaskDetector::update_tracked_blobs(
    const std::vector<BlobCandidate> &candidates, double timestamp) {
    std::vector<bool> matched(candidates.size(), false);

    for (auto &blob : tracked_blobs_) {
        double best_dist = config_.blob_match_distance_meters;
        int best_idx = -1;

        for (size_t i = 0; i < candidates.size(); ++i) {
            if (matched[i]) continue;
            double d = (candidates[i].position_in_field - blob.position_in_field).norm();
            if (d < best_dist) {
                best_dist = d;
                best_idx = static_cast<int>(i);
            }
        }

        if (best_idx >= 0) {
            blob.position_in_field = candidates[static_cast<size_t>(best_idx)].position_in_field;
            blob.consecutive_frames++;
            blob.last_seen_timestamp = timestamp;
            matched[static_cast<size_t>(best_idx)] = true;
        }
    }

    for (size_t i = 0; i < candidates.size(); ++i) {
        if (!matched[i]) {
            tracked_blobs_.push_back({candidates[i].position_in_field, 1, timestamp});
        }
    }

    tracked_blobs_.erase(
        std::remove_if(tracked_blobs_.begin(), tracked_blobs_.end(),
                       [timestamp, this](const TrackedBlob &b) {
                           return (timestamp - b.last_seen_timestamp) >
                                  config_.tracked_blob_timeout_seconds;
                       }),
        tracked_blobs_.end());

    std::vector<BlobCandidate> persistent;
    for (const auto &blob : tracked_blobs_) {
        if (blob.consecutive_frames >= config_.persistence_frames_required) {
            persistent.push_back({blob.position_in_field, 0.0});
        }
    }
    return persistent;
}

std::vector<RobotDescription> RobotMaskDetector::candidates_to_measurements(
    const std::vector<BlobCandidate> &candidates) {
    std::vector<RobotDescription> measurements;
    measurements.reserve(candidates.size());
    for (const auto &c : candidates) {
        Position pos{c.position_in_field.x(), c.position_in_field.y(),
                     c.position_in_field.z()};
        Pose pose{pos, Rotation{1, 0, 0, 0}};
        measurements.push_back(
            {FrameId::EMPTY, Label::OPPONENT, Group::THEIRS, pose, Size{0.15, 0.15, 0.1}, {},
             Velocity2D{}, false});
    }
    return measurements;
}

}  // namespace auto_battlebot
