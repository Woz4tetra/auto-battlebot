#pragma once

#include <Eigen/Dense>
#include <optional>
#include <vector>

#include "data_structures/camera.hpp"
#include "data_structures/command_feedback.hpp"
#include "data_structures/field.hpp"
#include "data_structures/robot.hpp"
#include "robot_filter/frame_id_assigner.hpp"

namespace auto_battlebot {

struct MotionEstimatorContext {
    Eigen::Matrix4d tf_fieldcenter_from_camera = Eigen::Matrix4d::Identity();
    CameraInfo camera_info;
    double field_bounds_margin_meters = 0.0;
    double our_robot_hold_window_s = 0.0;
};

/**
 * The propagation seam inside RobotFrontBackFilter: owns per-track motion state between the
 * association front-end (keypoint conversion, blob merging, FrameId assignment) and the emitted
 * RobotDescriptions. Implementations swap the state estimation strategy (dead reckoning vs
 * Kalman) without touching association.
 *
 * Call pattern mirrors RobotFilterInterface: predict() at control rate, update() at perception
 * rate, and the two are not in lockstep.
 */
class MotionEstimatorInterface {
   public:
    virtual ~MotionEstimatorInterface() = default;

    virtual void reset() = 0;

    virtual void predict(double now, const CommandFeedback &command_feedback) = 0;

    virtual std::vector<RobotDescription> update(std::vector<RobotDescription> measurements,
                                                 double timestamp,
                                                 FrameIdAssigner &frame_id_assigner,
                                                 const FieldDescription &field,
                                                 const MotionEstimatorContext &context) = 0;

    virtual std::optional<std::vector<RobotDescription>> coast([[maybe_unused]] double now) {
        return std::nullopt;
    }

    /**
     * How far past `now` coast() renders. Consumers steer by a state that has to describe the
     * field at the moment the command they are about to compute actually reaches the wheels, not
     * at the moment the shutter closed. Returning 0 means coast() renders at `now`.
     */
    virtual double render_lead_s() const { return 0.0; }
};

}  // namespace auto_battlebot
