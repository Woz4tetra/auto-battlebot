#pragma once

#include <spdlog/spdlog.h>

#include <Eigen/Dense>
#include <map>
#include <unordered_map>
#include <vector>

#include "data_structures/command_feedback.hpp"
#include "data_structures/measurement.hpp"
#include "diagnostics_logger/diagnostics_module_logger.hpp"
#include "robot_filter/config.hpp"
#include "robot_filter/frame_id_assigner.hpp"
#include "robot_filter/front_back_keypoint_converter.hpp"
#include "robot_filter/robot_filter_interface.hpp"
#include "robot_filter/robot_keypoint_tracker.hpp"
#include "robot_filter/robot_temporal_motion_filter.hpp"

namespace auto_battlebot {
class RobotFrontBackSimpleFilter : public RobotFilterInterface {
   public:
    explicit RobotFrontBackSimpleFilter(RobotFrontBackSimpleFilterConfiguration &config);

    /** Sets up per-label FrameId mappings and resets all filter state for a new match. */
    bool initialize(int opponent_count) override;

    /**
     * Runs one filter cycle: converts keypoints and blob detections to field-frame measurements,
     * merges them, applies temporal prediction, and returns stamped robot descriptions.
     */
    RobotDescriptionsStamped update(KeypointsStamped keypoints, FieldDescription field,
                                    CameraInfo camera_info, KeypointsStamped robot_blob_keypoints,
                                    CommandFeedback command_feedback) override;

   private:
    std::unordered_map<Label, RobotConfig> robot_configs_;
    std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;
    std::unique_ptr<FrontBackKeypointConverter> keypoint_converter_;
    std::map<Label, std::vector<FrameId>> label_to_frame_ids_;
    FrameId default_frame_id_;
    /** Exponential moving average smoothing factor for opponent velocity estimation (0..1, higher =
     * more responsive). */
    double velocity_ema_alpha_;
    /** Reject measurements that jump further than this (meters) from last known position. */
    double max_jump_distance_;
    /** After this many consecutive rejected frames, accept the measurement (tracking reset). */
    int max_consecutive_jump_rejects_;
    /** Minimum radius (meters) for suppressing blobs near keypoint detections. */
    double blob_overwrite_min_distance_meters_;
    /** Radius scale for suppression based on blob/keypoint size estimates. */
    double blob_overwrite_size_scale_;
    /** Extra margin added to field half extents before out-of-bounds rejection. */
    double field_bounds_margin_meters_;
    RobotKeypointTracker robot_keypoint_tracker_;
    FrameIdAssigner frame_id_assigner_;
    RobotTemporalMotionFilter temporal_motion_filter_;

    /**
     * Converts front/back keypoint detections into field-frame RobotDescriptions.
     * Clears stale last-positions for single-FrameId labels with no detections this frame.
     */
    std::vector<RobotDescription> convert_keypoints_to_measurements(
        const KeypointsStamped &keypoints, const FieldDescription &field,
        const CameraInfo &camera_info, const Eigen::Matrix4d &tf_fieldcenter_from_camera);

    /**
     * Builds the set of valid MeasurementWithConfidence entries for one label from its
     * front/back assignments, transforming keypoints into field-frame poses and sizes.
     * Entries where pose estimation fails are discarded.
     */
    std::vector<MeasurementWithConfidence> build_valid_measurements(
        const Eigen::Matrix4d &tf_fieldcenter_from_camera, Label label,
        const std::vector<std::pair<FrontBackAssignment, double>> &assignments_with_conf);

    /** Returns the FrameIds mapped to this label, or a single default if none are configured. */
    std::vector<FrameId> get_frame_ids_for_label(Label label) const;
    /** Returns the first configured FrameId for this label, or default_frame_id_ as fallback. */
    FrameId get_default_frame_id_for_label(const Label label) const;

    /**
     * Returns true if blob falls within the suppression radius of any keypoint measurement.
     * The radius is max(blob_overwrite_min_distance, blob_overwrite_size_scale * combined_size).
     */
    bool is_blob_suppressed_by_keypoint(
        const RobotKeypointDetection &blob,
        const std::vector<RobotDescription> &keypoint_measurements) const;

    /**
     * Returns the free FrameIds available to assign for this label. If the label's own FrameIds
     * are all taken and it belongs to Group::THEIRS, falls back to free OPPONENT FrameIds.
     */
    std::vector<FrameId> get_assignment_frame_ids(
        Label label, const std::vector<RobotDescription> &used_measurements) const;

    /**
     * Runs the full blob detection pipeline: detect, transform to field frame, filter by field
     * bounds, suppress detections already covered by keypoint measurements, then perform a
     * single global FrameId assignment over all surviving blobs (regardless of label) using
     * per-measurement allowed-FrameId constraints. Appends assigned results to
     * `all_measurements`. The global assignment makes the output independent of `Label` enum
     * order, which previously decided which label got first claim on shared OPPONENT slots.
     */
    void merge_blob_detections(const KeypointsStamped &robot_blob_keypoints,
                               const std::vector<RobotDescription> &keypoint_measurements,
                               const FieldDescription &field, const CameraInfo &camera_info,
                               const Eigen::Matrix4d &tf_fieldcenter_from_camera,
                               std::vector<RobotDescription> &all_measurements);
};

}  // namespace auto_battlebot
