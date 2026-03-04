#pragma once

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <vector>

#include "data_structures/camera.hpp"
#include "data_structures/field.hpp"
#include "data_structures/image.hpp"
#include "data_structures/robot.hpp"

namespace auto_battlebot {

struct ElevationDetectorConfig {
    double elevation_threshold_meters = 0.025;
    int min_blob_area_pixels = 50;
    int persistence_frames_required = 3;
    double blob_match_distance_meters = 0.3;
    double tracked_blob_timeout_seconds = 0.5;
    double field_inset_meters = 0.2;
    bool debug = false;
};

struct ElevationCandidate {
    Eigen::Vector3d position_in_field;
    double area_m2;
};

/** Blob tracked across frames for temporal persistence filtering. */
struct TrackedBlob {
    Eigen::Vector3d position_in_field = Eigen::Vector3d::Zero();
    int consecutive_frames = 0;
    double last_seen_timestamp = 0.0;
};

/**
 * @brief Detects unknown robots on the field using depth-based elevation above
 *        the RANSAC-fitted field plane.
 *
 * Stateful: maintains temporal blob tracking across frames. Call reset() when
 * the match or field changes.
 */
class ElevationDetector {
   public:
    explicit ElevationDetector(const ElevationDetectorConfig &config);

    void reset();

    /**
     * @brief Run the full elevation detection pipeline.
     *
     * Detects elevated blobs from the depth image, applies temporal persistence
     * filtering, and converts promoted blobs to OPPONENT RobotDescriptions.
     *
     * @param depth Depth image (possibly low-resolution).
     * @param field Current field description with plane transform.
     * @param camera_info Camera intrinsics (at RGB resolution).
     * @param own_robot_measurements Already-detected own-robot measurements to mask out.
     * @param timestamp Current frame timestamp for persistence tracking.
     * @return Opponent RobotDescriptions for blobs that passed persistence.
     */
    std::vector<RobotDescription> detect(
        const DepthImage &depth, const FieldDescription &field, const CameraInfo &camera_info,
        const std::vector<RobotDescription> &own_robot_measurements, double timestamp);

    int tracked_blob_count() const { return static_cast<int>(tracked_blobs_.size()); }

   private:
    /** Camera intrinsics scaled to depth image resolution. */
    struct DepthIntrinsics {
        double fx, fy, cx, cy;
        int width, height;
    };

    ElevationDetectorConfig config_;
    std::vector<TrackedBlob> tracked_blobs_;

    static DepthIntrinsics scale_intrinsics(const CameraInfo &camera_info, int depth_width,
                                            int depth_height);

    cv::Rect compute_field_roi(const FieldDescription &field, const DepthIntrinsics &intr) const;

    cv::Mat build_elevation_mask(const cv::Mat &depth_img, const cv::Rect &roi,
                                 const DepthIntrinsics &intr,
                                 const Eigen::Matrix4d &tf_cam_from_field) const;

    void mask_own_robots(cv::Mat &mask, const std::vector<RobotDescription> &own_robot_measurements,
                         const Eigen::Matrix4d &tf_cam_from_field,
                         const DepthIntrinsics &intr) const;

    std::vector<ElevationCandidate> extract_blob_candidates(
        const cv::Mat &mask, const cv::Mat &depth_img, const DepthIntrinsics &intr,
        const Eigen::Matrix4d &tf_fieldcenter_from_camera) const;

    std::vector<ElevationCandidate> update_tracked_blobs(
        const std::vector<ElevationCandidate> &candidates, double timestamp);

    static std::vector<RobotDescription> candidates_to_measurements(
        const std::vector<ElevationCandidate> &candidates);

    void visualize_debug_mosaic(const cv::Mat &elevation_mask, const cv::Mat &filtered_mask) const;
};

}  // namespace auto_battlebot
