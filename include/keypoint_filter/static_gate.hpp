#pragma once

#include <Eigen/Dense>
#include <memory>
#include <vector>

#include "data_structures.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"
#include "keypoint_filter/config.hpp"

namespace auto_battlebot {

/**
 * @brief Suppresses robot-blob detections that hold the same field position for seconds on end.
 *
 * Arena floor graphics are fixed in the field frame; robots are not. Tracking each detection's
 * position in field coordinates (so camera motion does not make scenery look mobile) and dropping
 * the ones that never move removes the repeat offenders a per-frame appearance test cannot.
 *
 * The known cost: an immobilised opponent that stops moving for `min_dwell_seconds` looks exactly
 * like scenery and will be suppressed. That is what `min_dwell_seconds` trades. A cluster loses
 * its static verdict the moment an observation lands outside `static_radius_meters` of its running
 * mean, so a robot that stops and then moves again is picked straight back up.
 *
 * Robot blobs only. Our own robot's keypoints come from a separate model and are not gated here.
 */
class StaticDetectionGate {
   public:
    struct Stats {
        int groups_in = 0;
        int groups_passed = 0;
        int suppressed = 0;
        int tracked_clusters = 0;
        int static_clusters = 0;
    };

    explicit StaticDetectionGate(const StaticDetectionGateConfiguration &config);

    /**
     * @brief Drop keypoint groups sitting on a cluster that has not moved.
     * @param model_results Robot-blob keypoint groups for this frame
     * @param camera_info Intrinsics used to project keypoints onto the field plane
     * @param field Field description supplying the camera-to-field transform
     * @param stamp Frame timestamp in seconds, driving dwell and forget timing
     * @return The surviving keypoints
     */
    ModelResultStamped filter(const ModelResultStamped &model_results,
                              const CameraInfo &camera_info, const FieldDescription &field,
                              double stamp);

    /** Forget every tracked cluster. Called when the field is re-initialised, since field
     *  coordinates recorded against the old origin no longer mean anything. */
    void reset();

    const Stats &last_stats() const { return stats_; }

   private:
    struct Cluster {
        Eigen::Vector2d mean = Eigen::Vector2d::Zero();
        double first_seen = 0.0;
        double last_seen = 0.0;
        int observations = 0;
        /** Running mean distance of observations from `mean`. Mean, not maximum: one noisy frame
         *  must not disqualify a logo, and one frame of a robot clipping the cluster must not
         *  either. Both statistics stop averaging past responsiveness_window and start tracking,
         *  so a robot parking on a logo can still move the verdict after thousands of frames. */
        double mean_deviation = 0.0;

        /** Fold one observation in, after the suppression decision has been taken. */
        void observe(const Eigen::Vector2d &position,
                     const StaticDetectionGateConfiguration &config, double stamp);
        bool is_static(const StaticDetectionGateConfiguration &config, double now) const;
    };

    StaticDetectionGateConfiguration config_;
    std::vector<Cluster> clusters_;
    std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;
    Stats stats_;
};

}  // namespace auto_battlebot
