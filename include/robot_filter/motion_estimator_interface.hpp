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

/**
 * Per-update inputs the estimator needs beyond the associated measurements themselves.
 * Carries the camera geometry so an estimator can build a range-dependent measurement
 * covariance in camera coordinates and rotate it into the field frame.
 */
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

    /** Clears all per-track state. Called on filter initialize(). */
    virtual void reset() = 0;

    /** Control-rate tick: record the command and advance internal state to `now`. */
    virtual void predict(double now, const CommandFeedback &command_feedback) = 0;

    /**
     * Perception-rate: fold in this frame's associated measurements (FrameId already assigned)
     * and return one description per live track. Unmeasured tracks are propagated or held per
     * the implementation and flagged is_stale. `timestamp` is the measurement capture time.
     */
    virtual std::vector<RobotDescription> update(std::vector<RobotDescription> measurements,
                                                 double timestamp,
                                                 FrameIdAssigner &frame_id_assigner,
                                                 const FieldDescription &field,
                                                 const MotionEstimatorContext &context) = 0;

    /**
     * Descriptions advanced to `now + render_lead_s()` without folding a measurement, for
     * control-rate coasting between perception frames. Returns nullopt when the estimator's
     * output only moves on measurements (dead reckoning), in which case the caller keeps the
     * last update() result.
     */
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
