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

namespace auto_battlebot {
class RobotFrontBackFilter : public RobotFilterInterface {
   public:
    explicit RobotFrontBackFilter(const RobotFrontBackFilterConfiguration &config);

    bool initialize(int opponent_count) override;

    void predict(double now, CommandFeedback command_feedback) override;

    void correct(ModelResultStamped keypoints, FieldDescription field,
                 CameraInfo camera_info) override;

    RobotDescriptionsStamped state() const override { return state_; }

   private:
    std::unordered_map<Label, RobotConfig> robot_configs_;
    std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;
    std::unique_ptr<FrontBackKeypointConverter> keypoint_converter_;
    std::map<Label, std::vector<FrameId>> label_to_frame_ids_;
    FrameId default_frame_id_;
    double max_jump_distance_;
    int max_consecutive_jump_rejects_;
    std::map<Label, Size> size_overrides_;
    std::map<Label, Position2D> footprint_center_offsets_;
    double field_bounds_margin_meters_;
    FrameIdAssigner frame_id_assigner_;
    std::unique_ptr<MotionEstimatorInterface> motion_estimator_;

    RobotDescriptionsStamped state_;

    std::vector<RobotDescription> convert_keypoints_to_measurements(
        const ModelResultStamped &model_results, const FieldDescription &field,
        const CameraInfo &camera_info, const Eigen::Matrix4d &tf_fieldcenter_from_camera);

    std::vector<MeasurementWithConfidence> build_valid_measurements(
        const Eigen::Matrix4d &tf_fieldcenter_from_camera, Label label,
        const std::vector<std::pair<FrontBackAssignment, double>> &assignments_with_conf);

    std::vector<FrameId> get_frame_ids_for_label(Label label) const;
    FrameId get_default_frame_id_for_label(const Label label) const;

    void apply_known_geometry(std::vector<RobotDescription> &descriptions) const;
};

}  // namespace auto_battlebot
