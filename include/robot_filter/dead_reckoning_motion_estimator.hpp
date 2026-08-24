#pragma once

#include "robot_filter/motion_estimator_config.hpp"
#include "robot_filter/motion_estimator_interface.hpp"
#include "robot_filter/robot_temporal_motion_filter.hpp"

namespace auto_battlebot {

/**
 * Dead-reckoning propagation behind the motion estimator seam: applies the last stick command,
 * scaled to body velocity through the fitted plant gains, as the achieved velocity via
 * RobotTemporalMotionFilter, holds opponents at their
 * last pose, and only moves state on update(). coast() stays nullopt so the emitted state
 * keeps its correct()-only cadence.
 */
class DeadReckoningMotionEstimator : public MotionEstimatorInterface {
   public:
    explicit DeadReckoningMotionEstimator(const DeadReckoningMotionEstimatorConfiguration &config)
        : temporal_motion_filter_(config.plant.value_or(JigPlantParams{})) {}

    void reset() override;

    void predict(double now, const CommandFeedback &command_feedback) override;

    std::vector<RobotDescription> update(std::vector<RobotDescription> measurements,
                                         double timestamp, FrameIdAssigner &frame_id_assigner,
                                         const FieldDescription &field,
                                         const MotionEstimatorContext &context) override;

   private:
    RobotTemporalMotionFilter temporal_motion_filter_;
    /** Control input recorded by predict(), consumed by the next update(). */
    CommandFeedback command_feedback_;
};

}  // namespace auto_battlebot
