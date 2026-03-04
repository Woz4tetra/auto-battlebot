#include "robot_filter/elevation_detector.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "enums/frame_id.hpp"
#include "enums/label.hpp"
#include "transform_utils.hpp"

namespace auto_battlebot {

ElevationDetector::ElevationDetector(const ElevationDetectorConfig &config) : config_(config) {}

void ElevationDetector::reset() { tracked_blobs_.clear(); }

std::vector<RobotDescription> ElevationDetector::detect(
    const DepthImage &depth, const FieldDescription &field, const CameraInfo &camera_info,
    const std::vector<RobotDescription> &own_robot_measurements, double timestamp) {
    const cv::Mat &depth_img = depth.image;
    if (depth_img.empty()) return {};

    DepthIntrinsics intr = scale_intrinsics(camera_info, depth_img.cols, depth_img.rows);
    Eigen::Matrix4d tf_cam_from_field = field.tf_camera_from_fieldcenter.tf;

    cv::Rect roi = compute_field_roi(field, intr);
    if (roi.width <= 0 || roi.height <= 0) return {};

    cv::Mat elevation_mask = build_elevation_mask(depth_img, roi, intr, tf_cam_from_field);
    cv::Mat mask = elevation_mask.clone();
    mask_own_robots(mask, own_robot_measurements, tf_cam_from_field, intr);

    if (config_.debug) {
        visualize_debug_mosaic(elevation_mask, mask);
    }

    Eigen::Matrix4d tf_field_from_camera = tf_cam_from_field.inverse();
    auto candidates = extract_blob_candidates(mask, depth_img, intr, tf_field_from_camera);
    auto promoted = update_tracked_blobs(candidates, timestamp);
    return candidates_to_measurements(promoted);
}

// ---------------------------------------------------------------------------
// Intrinsics helpers
// ---------------------------------------------------------------------------

ElevationDetector::DepthIntrinsics ElevationDetector::scale_intrinsics(
    const CameraInfo &camera_info, int depth_width, int depth_height) {
    double sx = static_cast<double>(depth_width) / static_cast<double>(camera_info.width);
    double sy = static_cast<double>(depth_height) / static_cast<double>(camera_info.height);
    return {camera_info.intrinsics.at<double>(0, 0) * sx,
            camera_info.intrinsics.at<double>(1, 1) * sy,
            camera_info.intrinsics.at<double>(0, 2) * sx,
            camera_info.intrinsics.at<double>(1, 2) * sy,
            depth_width,
            depth_height};
}

// ---------------------------------------------------------------------------
// Field ROI
// ---------------------------------------------------------------------------

cv::Rect ElevationDetector::compute_field_roi(const FieldDescription &field,
                                              const DepthIntrinsics &intr) const {
    Eigen::Matrix4d tf_cam_from_field = field.tf_camera_from_fieldcenter.tf;
    double half_w = field.size.size.x / 2.0 - config_.field_inset_meters;
    double half_h = field.size.size.y / 2.0 - config_.field_inset_meters;

    std::vector<Eigen::Vector4d> corners = {
        {-half_w, -half_h, 0.0, 1.0},
        {half_w, -half_h, 0.0, 1.0},
        {half_w, half_h, 0.0, 1.0},
        {-half_w, half_h, 0.0, 1.0},
    };

    double min_u = intr.width, max_u = 0, min_v = intr.height, max_v = 0;

    for (const auto &corner : corners) {
        Eigen::Vector4d pt_cam = tf_cam_from_field * corner;
        if (pt_cam.z() <= 0.0) continue;
        double u = intr.fx * pt_cam.x() / pt_cam.z() + intr.cx;
        double v = intr.fy * pt_cam.y() / pt_cam.z() + intr.cy;
        min_u = std::min(min_u, u);
        max_u = std::max(max_u, u);
        min_v = std::min(min_v, v);
        max_v = std::max(max_v, v);
    }

    int x0 = std::max(0, static_cast<int>(std::floor(min_u)));
    int y0 = std::max(0, static_cast<int>(std::floor(min_v)));
    int x1 = std::min(intr.width, static_cast<int>(std::ceil(max_u)));
    int y1 = std::min(intr.height, static_cast<int>(std::ceil(max_v)));

    if (x1 <= x0 || y1 <= y0) return cv::Rect(0, 0, 0, 0);
    return cv::Rect(x0, y0, x1 - x0, y1 - y0);
}

// ---------------------------------------------------------------------------
// Elevation mask construction
// ---------------------------------------------------------------------------

cv::Mat ElevationDetector::build_elevation_mask(const cv::Mat &depth_img, const cv::Rect &roi,
                                                const DepthIntrinsics &intr,
                                                const Eigen::Matrix4d &tf_cam_from_field) const {
    Eigen::Vector3d plane_normal = tf_cam_from_field.block<3, 1>(0, 2);
    Eigen::Vector3d plane_point = tf_cam_from_field.block<3, 1>(0, 3);
    double plane_d = -plane_normal.dot(plane_point);

    cv::Mat mask = cv::Mat::zeros(intr.height, intr.width, CV_8UC1);

    for (int v = roi.y; v < roi.y + roi.height; ++v) {
        const float *depth_row = depth_img.ptr<float>(v);
        uchar *mask_row = mask.ptr<uchar>(v);
        for (int u = roi.x; u < roi.x + roi.width; ++u) {
            float z = depth_row[u];
            if (std::isnan(z) || z <= 0.0f) continue;

            double x = (u - intr.cx) * z / intr.fx;
            double y = (v - intr.cy) * z / intr.fy;

            double elevation =
                plane_normal.x() * x + plane_normal.y() * y + plane_normal.z() * z + plane_d;

            if (elevation < -config_.elevation_threshold_meters) {
                mask_row[u] = 255;
            }
        }
    }
    return mask;
}

// ---------------------------------------------------------------------------
// Own-robot masking
// ---------------------------------------------------------------------------

void ElevationDetector::mask_own_robots(cv::Mat &mask,
                                        const std::vector<RobotDescription> &own_robot_measurements,
                                        const Eigen::Matrix4d &tf_cam_from_field,
                                        const DepthIntrinsics &intr) const {
    for (const auto &own : own_robot_measurements) {
        if (own.label == Label::OPPONENT) continue;
        Eigen::Vector3d center_field(own.pose.position.x, own.pose.position.y, 0.0);
        Eigen::Vector4d center_cam_h =
            tf_cam_from_field * Eigen::Vector4d(center_field.x(), center_field.y(), 0.0, 1.0);
        if (center_cam_h.z() <= 0.0) continue;

        double robot_radius = std::max(own.size.x, own.size.y) / 2.0 + 0.05;
        std::vector<Eigen::Vector4d> box_corners = {
            tf_cam_from_field * Eigen::Vector4d(center_field.x() - robot_radius,
                                                center_field.y() - robot_radius, 0.0, 1.0),
            tf_cam_from_field * Eigen::Vector4d(center_field.x() + robot_radius,
                                                center_field.y() + robot_radius, 0.0, 1.0),
        };

        double min_u = intr.width, max_u = 0, min_v = intr.height, max_v = 0;
        for (const auto &c : box_corners) {
            if (c.z() <= 0.0) continue;
            double pu = intr.fx * c.x() / c.z() + intr.cx;
            double pv = intr.fy * c.y() / c.z() + intr.cy;
            min_u = std::min(min_u, pu);
            max_u = std::max(max_u, pu);
            min_v = std::min(min_v, pv);
            max_v = std::max(max_v, pv);
        }

        int ex0 = std::max(0, static_cast<int>(std::floor(min_u)));
        int ey0 = std::max(0, static_cast<int>(std::floor(min_v)));
        int ex1 = std::min(intr.width, static_cast<int>(std::ceil(max_u)));
        int ey1 = std::min(intr.height, static_cast<int>(std::ceil(max_v)));
        if (ex1 > ex0 && ey1 > ey0) {
            mask(cv::Rect(ex0, ey0, ex1 - ex0, ey1 - ey0)).setTo(0);
        }
    }
}

// ---------------------------------------------------------------------------
// Blob extraction from elevation mask
// ---------------------------------------------------------------------------

std::vector<ElevationCandidate> ElevationDetector::extract_blob_candidates(
    const cv::Mat &mask, const cv::Mat &depth_img, const DepthIntrinsics &intr,
    const Eigen::Matrix4d &tf_fieldcenter_from_camera) const {
    std::vector<ElevationCandidate> candidates;

    cv::Mat labels, stats, centroids;
    int num_labels = cv::connectedComponentsWithStats(mask, labels, stats, centroids);

    for (int i = 1; i < num_labels; ++i) {
        int area = stats.at<int>(i, cv::CC_STAT_AREA);
        if (area < config_.min_blob_area_pixels) continue;

        double cu = centroids.at<double>(i, 0);
        double cv_val = centroids.at<double>(i, 1);

        int cu_int = std::clamp(static_cast<int>(cu), 0, intr.width - 1);
        int cv_int = std::clamp(static_cast<int>(cv_val), 0, intr.height - 1);
        float z = depth_img.at<float>(cv_int, cu_int);
        if (std::isnan(z) || z <= 0.0f) continue;

        double x3d = (cu - intr.cx) * z / intr.fx;
        double y3d = (cv_val - intr.cy) * z / intr.fy;

        Eigen::Vector4d pt_cam(x3d, y3d, z, 1.0);
        Eigen::Vector4d pt_field = tf_fieldcenter_from_camera * pt_cam;

        double pixel_size_m = z / intr.fx;
        double area_m2 = area * pixel_size_m * pixel_size_m;

        candidates.push_back({pt_field.head<3>(), area_m2});
    }

    return candidates;
}

// ---------------------------------------------------------------------------
// Temporal tracking
// ---------------------------------------------------------------------------

std::vector<ElevationCandidate> ElevationDetector::update_tracked_blobs(
    const std::vector<ElevationCandidate> &candidates, double timestamp) {
    std::vector<bool> candidate_matched(candidates.size(), false);

    for (auto &blob : tracked_blobs_) {
        double best_dist = config_.blob_match_distance_meters;
        int best_idx = -1;
        for (size_t i = 0; i < candidates.size(); ++i) {
            if (candidate_matched[i]) continue;
            double dist = (blob.position_in_field - candidates[i].position_in_field).norm();
            if (dist < best_dist) {
                best_dist = dist;
                best_idx = static_cast<int>(i);
            }
        }
        if (best_idx >= 0) {
            candidate_matched[best_idx] = true;
            blob.position_in_field = candidates[best_idx].position_in_field;
            blob.consecutive_frames++;
            blob.last_seen_timestamp = timestamp;
        }
    }

    double timeout = config_.tracked_blob_timeout_seconds;
    tracked_blobs_.erase(std::remove_if(tracked_blobs_.begin(), tracked_blobs_.end(),
                                        [timestamp, timeout](const TrackedBlob &b) {
                                            return (timestamp - b.last_seen_timestamp) > timeout;
                                        }),
                         tracked_blobs_.end());

    for (size_t i = 0; i < candidates.size(); ++i) {
        if (!candidate_matched[i]) {
            tracked_blobs_.push_back({candidates[i].position_in_field, 1, timestamp});
        }
    }

    std::vector<ElevationCandidate> promoted;
    for (const auto &blob : tracked_blobs_) {
        if (blob.consecutive_frames >= config_.persistence_frames_required) {
            promoted.push_back({blob.position_in_field, 0.0});
        }
    }
    return promoted;
}

// ---------------------------------------------------------------------------
// Candidate → RobotDescription conversion
// ---------------------------------------------------------------------------

std::vector<RobotDescription> ElevationDetector::candidates_to_measurements(
    const std::vector<ElevationCandidate> &candidates) {
    std::vector<RobotDescription> measurements;
    for (const auto &candidate : candidates) {
        Position pos;
        pos.x = candidate.position_in_field.x();
        pos.y = candidate.position_in_field.y();
        pos.z = 0.0;

        Pose pose;
        pose.position = pos;
        pose.rotation = euler_to_quaternion(0.0, 0.0, 0.0);

        RobotDescription desc;
        desc.frame_id = FrameId::EMPTY;
        desc.label = Label::OPPONENT;
        desc.pose = pose;
        desc.size = Size{0.15, 0.15, 0.05};
        desc.keypoints = {pos};
        measurements.push_back(desc);
    }
    return measurements;
}

// ---------------------------------------------------------------------------
// Debug visualization
// ---------------------------------------------------------------------------

void ElevationDetector::visualize_debug_mosaic(const cv::Mat &elevation_mask,
                                               const cv::Mat &filtered_mask) const {
    cv::Mat elevation_vis, filtered_vis;
    cv::cvtColor(elevation_mask, elevation_vis, cv::COLOR_GRAY2BGR);
    cv::cvtColor(filtered_mask, filtered_vis, cv::COLOR_GRAY2BGR);

    cv::putText(elevation_vis, "Elevation Mask", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                cv::Scalar(0, 255, 0), 2);
    cv::putText(filtered_vis, "After Own Robot Mask", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX,
                0.7, cv::Scalar(0, 255, 0), 2);

    cv::Mat mosaic;
    cv::hconcat(std::vector<cv::Mat>{elevation_vis, filtered_vis}, mosaic);

    cv::imshow("Elevation Detector Debug", mosaic);
    cv::waitKey(1);
}

}  // namespace auto_battlebot
