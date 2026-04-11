#pragma once

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <vector>

#include "data_structures/camera.hpp"
#include "data_structures/field.hpp"
#include "data_structures/robot.hpp"

namespace auto_battlebot {

struct RobotMaskDetectorConfig {
    int min_blob_area_pixels = 100;
    int persistence_frames_required = 3;
    double blob_match_distance_meters = 0.3;
    double tracked_blob_timeout_seconds = 0.5;
    int morph_kernel_size = 5;
};

struct TrackedBlob {
    Eigen::Vector3d position_in_field = Eigen::Vector3d::Zero();
    int consecutive_frames = 0;
    double last_seen_timestamp = 0.0;
};

class RobotMaskDetector {
   public:
    explicit RobotMaskDetector(const RobotMaskDetectorConfig &config);

    void reset();

    std::vector<RobotDescription> detect(
        const cv::Mat &mask, const FieldDescription &field, const CameraInfo &camera_info,
        const std::vector<RobotDescription> &own_robot_measurements, double timestamp);

   private:
    struct ScaledIntrinsics {
        double fx, fy, cx, cy;
        int width, height;
    };

    RobotMaskDetectorConfig config_;
    std::vector<TrackedBlob> tracked_blobs_;

    static ScaledIntrinsics scale_intrinsics(const CameraInfo &camera_info, int mask_width,
                                             int mask_height);

    cv::Rect compute_field_roi(const FieldDescription &field, const ScaledIntrinsics &intr) const;

    cv::Mat build_detection_mask(const cv::Mat &mask, const cv::Rect &field_roi) const;

    void mask_own_robots(cv::Mat &mask, const std::vector<RobotDescription> &own_robots,
                         const Eigen::Matrix4d &tf_cam_from_field,
                         const ScaledIntrinsics &intr) const;

    struct BlobCandidate {
        Eigen::Vector3d position_in_field;
        double area_m2;
    };

    std::vector<BlobCandidate> extract_blob_candidates(const cv::Mat &mask,
                                                       const ScaledIntrinsics &intr,
                                                       const Eigen::Matrix4d &tf_field_from_cam,
                                                       const FieldDescription &field) const;

    Eigen::Vector3d ray_plane_intersect(const Eigen::Vector3d &ray_origin,
                                        const Eigen::Vector3d &ray_dir,
                                        const Eigen::Vector3d &plane_point,
                                        const Eigen::Vector3d &plane_normal) const;

    std::vector<BlobCandidate> update_tracked_blobs(const std::vector<BlobCandidate> &candidates,
                                                    double timestamp);

    static std::vector<RobotDescription> candidates_to_measurements(
        const std::vector<BlobCandidate> &candidates);
};

}  // namespace auto_battlebot
