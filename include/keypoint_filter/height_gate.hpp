#pragma once

#include <Eigen/Dense>
#include <memory>
#include <opencv2/core.hpp>
#include <vector>

#include "data_structures.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"
#include "keypoint_filter/config.hpp"

namespace auto_battlebot {

/**
 * @brief Measures how far each detection stands above the floor around it, and rejects the ones
 *        that do not stand above it at all.
 *
 * Two jobs, deliberately separable:
 *
 * 1. Measurement, always on. Each surviving keypoint carries the height measured at its own pixel,
 *    which RobotKeypointTracker and FrontBackKeypointConverter project with in place of the
 *    configured `keypoint_height_meters` constant. A keypoint that cannot be measured keeps NaN
 *    and falls back to the constant, so this has no false-negative risk.
 *
 * 2. Rejection, on by default, against the *local* relief: the top percentile inside the
 *    detection's inscribed circle minus the floor percentile of a ring around it. Comparing a
 *    detection with its own surroundings rather than with the fitted field plane cancels plane-fit
 *    error, plane drift and depth bias. On the 2026-08-29 MassD replay that lifts separation of
 *    arena-logo detections from moving ones to AUC 0.928, against 0.903 for absolute height.
 *
 * Two things this gate learned the hard way, both pinned by tests:
 *
 * - Sample inside the detection box, never at the keypoint pixels. Blob keypoints are the ends of
 *   a box midline and land on the floor beside a robot: mr_stabs_mk2 read -0.019 m at its keypoint
 *   pixels and +0.056 m over its box.
 * - Take an upper percentile, never a median. Even an inscribed circle holds floor alongside the
 *   robot, because the camera views the arena about 23 degrees off horizontal.
 *
 * The gate abstains rather than guesses. With no depth, no field transform, no detection box, or
 * too few valid samples in either region, the group passes through untouched.
 */
class KeypointHeightGate {
   public:
    struct Stats {
        int groups_in = 0;
        int groups_passed = 0;
        int rejected_low = 0;   //< below the band: flat on the field
        int rejected_high = 0;  //< above the band: not a robot on the floor
        int abstained = 0;      //< depth unusable, group passed through unmeasured
    };

    explicit KeypointHeightGate(const KeypointHeightGateConfiguration &config);

    /**
     * @brief Annotate `model_results` with measured heights, rejecting groups outside the band when
     *        rejection is enabled.
     * @param model_results Keypoint groups, grouped by (label, detection_index)
     * @param depth Depth image for this frame, in metres along the optical axis. May be empty
     * @param camera_info Intrinsics matching the depth image
     * @param field Field description supplying the plane via tf_camera_from_fieldcenter
     * @return The surviving keypoints, with `height_above_plane` set where it was measured
     */
    ModelResultStamped filter(const ModelResultStamped &model_results, const DepthImage &depth,
                              const CameraInfo &camera_info, const FieldDescription &field);

    const Stats &last_stats() const { return stats_; }

   private:
    /** Plane geometry for one frame, resolved once rather than per keypoint. */
    struct Plane {
        Eigen::Vector3d center;
        Eigen::Vector3d normal;
        /** transform_to_plane_center_normal does not fix which side the normal points to. This
         *  flips it so the camera's own height, and everything above the floor, comes out
         *  positive. */
        double orientation;
    };

    /** What one detection's depth says about it. */
    struct Relief {
        /** Top of the detection above the field plane, in metres. NaN when unmeasurable. */
        double top = 0.0;
        /** Top minus the floor immediately around the detection. NaN when unmeasurable. */
        double elevation = 0.0;
        bool valid = false;
    };

    /** Height of the surface imaged at one pixel, or NaN when depth there is unusable. */
    double height_at(const cv::Mat &depth, const CameraInfo &camera_info, const Plane &plane,
                     int col, int row) const;

    /** Median height over the sample window centred on (x, y), for the per-keypoint annotation.
     *  A median rather than a mean because depth bleeds across object edges, where the outliers
     *  are hundreds of millimetres rather than a few. */
    double height_at_keypoint(const cv::Mat &depth, const CameraInfo &camera_info,
                              const Plane &plane, double x, double y) const;

    /** Top and local relief for the group's detection box, sampling its inscribed circle and the
     *  floor ring around it in one pass. */
    Relief measure_relief(const cv::Mat &depth, const CameraInfo &camera_info, const Plane &plane,
                          const ModelResultStamped &model_results,
                          const std::vector<size_t> &indices) const;

    KeypointHeightGateConfiguration config_;
    std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;
    Stats stats_;
};

}  // namespace auto_battlebot
