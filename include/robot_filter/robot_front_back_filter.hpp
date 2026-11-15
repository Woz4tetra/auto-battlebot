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
#include "robot_filter/motion_estimator_interface.hpp"
#include "robot_filter/robot_filter_interface.hpp"
#include "robot_filter/robot_keypoint_tracker.hpp"

namespace auto_battlebot {
class RobotFrontBackFilter : public RobotFilterInterface {
   public:
    explicit RobotFrontBackFilter(RobotFrontBackFilterConfiguration &config);

    bool initialize(int opponent_count) override;

    void predict(double now, CommandFeedback command_feedback) override;

    void correct(ModelResultStamped keypoints, FieldDescription field, CameraInfo camera_info,
                 ModelResultStamped robot_blob_keypoints) override;

    RobotDescriptionsStamped state() const override { return state_; }

    bool last_our_blob_present_no_keypoint() const override {
        return our_blob_present_no_keypoint_;
    }

   private:
    std::unordered_map<Label, RobotConfig> robot_configs_;
    std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;
    std::unique_ptr<FrontBackKeypointConverter> keypoint_converter_;
    std::map<Label, std::vector<FrameId>> label_to_frame_ids_;
    FrameId default_frame_id_;
    /** Reject measurements that jump further than this (meters) from last known position. */
    double max_jump_distance_;
    /** After this many consecutive rejected frames, accept the measurement (tracking reset). */
    int max_consecutive_jump_rejects_;
    /** Minimum radius (meters) for suppressing blobs near keypoint detections. */
    double blob_overwrite_min_distance_meters_;
    /** Radius scale for suppression based on blob/keypoint size estimates. */
    double blob_overwrite_size_scale_;
    std::map<Label, Size> size_overrides_;
    std::map<Label, Position2D> footprint_center_offsets_;
    /** Extra margin added to field half extents before out-of-bounds rejection. */
    double field_bounds_margin_meters_;
    /** Seconds an unmeasured our-robot track is held before the decay drops it (<= 0 disables). */
    double our_robot_hold_window_s_;
    /** Radius around the held our-robot pose in which blobs are dropped during a keypoint miss. */
    double our_keypoint_dropout_blob_radius_meters_;
    /** Seconds the dropout suppression radius stays active after the last keypoint-confirmed
     * frame. */
    double our_keypoint_dropout_blob_window_s_;
    RobotKeypointTracker robot_keypoint_tracker_;
    FrameIdAssigner frame_id_assigner_;
    std::unique_ptr<MotionEstimatorInterface> motion_estimator_;

    RobotDescriptionsStamped state_;

    bool our_blob_present_no_keypoint_ = false;
    /**
     * Last emitted OUR_ROBOT_1 pose (measured or predicted) with the stamp of the frame that
     * produced it. The anchor deliberately outlives the track: our_robot_hold_window_s_ drops the
     * track sooner than the suppression window needs the position, so staleness is decided by the
     * stamp (is_our_anchor_fresh) rather than by the track still existing.
     */
    bool has_last_our_position_ = false;
    Position last_our_position_{};
    double last_our_size_x_ = 0.0;
    double last_our_position_stamp_ = 0.0;

    std::vector<RobotDescription> convert_keypoints_to_measurements(
        const ModelResultStamped &model_results, const FieldDescription &field,
        const CameraInfo &camera_info, const Eigen::Matrix4d &tf_fieldcenter_from_camera);

    std::vector<MeasurementWithConfidence> build_valid_measurements(
        const Eigen::Matrix4d &tf_fieldcenter_from_camera, Label label,
        const std::vector<std::pair<FrontBackAssignment, double>> &assignments_with_conf);

    std::vector<FrameId> get_frame_ids_for_label(Label label) const;
    FrameId get_default_frame_id_for_label(const Label label) const;

    void apply_known_geometry(std::vector<RobotDescription> &descriptions) const;

    bool is_blob_suppressed_by_keypoint(
        const RobotKeypointDetection &blob,
        const std::vector<RobotDescription> &keypoint_measurements) const;

    /**
     * Drops blobs that fall within our_keypoint_dropout_blob_radius_meters_ of our robot's held
     * pose while no OUR_ROBOT_1 keypoint measurement exists this frame. The blob model has no
     * class for our robot, so such a blob would otherwise be assigned an opponent FrameId at our
     * own location and steered into. Does nothing when our keypoint is present (the existing
     * is_blob_suppressed_by_keypoint rule covers that case) or when the anchor is stale.
     * Returns the number of blobs removed.
     */
    int suppress_blobs_near_our_anchor(double stamp,
                                       const std::vector<RobotDescription> &keypoint_measurements,
                                       std::vector<RobotKeypointDetection> &blobs) const;

    bool is_our_anchor_fresh(double stamp) const;

    void update_our_position_anchor(const std::vector<RobotDescription> &descriptions,
                                    double stamp);

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
    void merge_blob_detections(const ModelResultStamped &robot_blob_keypoints,
                               const std::vector<RobotDescription> &keypoint_measurements,
                               const FieldDescription &field, const CameraInfo &camera_info,
                               const Eigen::Matrix4d &tf_fieldcenter_from_camera, double stamp,
                               std::vector<RobotDescription> &all_measurements);
};

}  // namespace auto_battlebot
