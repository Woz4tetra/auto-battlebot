#pragma once

#include <Eigen/Dense>
#include <memory>
#include <opencv2/core.hpp>
#include <vector>

#include "data_structures.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"
#include "keypoint_filter/config.hpp"

namespace auto_battlebot {

class KeypointHeightGate {
   public:
    struct Stats {
        int groups_in = 0;
        int groups_passed = 0;
        int rejected_low = 0;
        int rejected_high = 0;
        int abstained = 0;
    };

    explicit KeypointHeightGate(const KeypointHeightGateConfiguration &config);

    ModelResultStamped filter(const ModelResultStamped &model_results, const DepthImage &depth,
                              const CameraInfo &camera_info, const FieldDescription &field);

    const Stats &last_stats() const { return stats_; }

   private:
    struct Plane {
        Eigen::Vector3d center;
        Eigen::Vector3d normal;
        /** transform_to_plane_center_normal does not fix which side the normal points to. This
         *  flips it so the camera's own height, and everything above the floor, comes out
         *  positive. */
        double orientation;
    };

    struct Relief {
        /** Top of the detection above the field plane, in metres. NaN when unmeasurable. */
        double top = 0.0;
        /** Top minus the floor immediately around the detection. NaN when unmeasurable. */
        double elevation = 0.0;
        bool valid = false;
    };

    double height_at(const cv::Mat &depth, const CameraInfo &camera_info, const Plane &plane,
                     int col, int row) const;

    double height_at_keypoint(const cv::Mat &depth, const CameraInfo &camera_info,
                              const Plane &plane, double x, double y) const;

    Relief measure_relief(const cv::Mat &depth, const CameraInfo &camera_info, const Plane &plane,
                          const ModelResultStamped &model_results,
                          const std::vector<size_t> &indices) const;

    KeypointHeightGateConfiguration config_;
    std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;
    Stats stats_;
};

}  // namespace auto_battlebot
