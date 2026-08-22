#include "robot_filter/dead_reckoning_motion_estimator.hpp"

namespace auto_battlebot {

void DeadReckoningMotionEstimator::reset() {
    temporal_motion_filter_.reset();
    command_feedback_ = CommandFeedback{};
}

void DeadReckoningMotionEstimator::predict([[maybe_unused]] double now,
                                           const CommandFeedback &command_feedback) {
    command_feedback_ = command_feedback;
}

std::vector<RobotDescription> DeadReckoningMotionEstimator::update(
    std::vector<RobotDescription> measurements, double timestamp,
    FrameIdAssigner &frame_id_assigner, const FieldDescription &field,
    const MotionEstimatorContext &context) {
    return temporal_motion_filter_.update_with_prediction(
        std::move(measurements), command_feedback_, timestamp, frame_id_assigner, field,
        context.field_bounds_margin_meters, context.our_robot_hold_window_s);
}

}  // namespace auto_battlebot
