#pragma once

#include "data_structures.hpp"

namespace auto_battlebot {
/**
 * Two-phase robot filter: predict advances state through time, correct folds in a perception
 * measurement, state reads the current estimate.
 *
 * predict() and state() run at the control rate; correct() runs at the slower perception rate.
 * Implementations must not assume the two are called in lockstep, at the same frequency, or that
 * a correct() always follows a predict().
 */
class RobotFilterInterface {
   public:
    virtual ~RobotFilterInterface() = default;
    virtual bool initialize(int opponent_count) = 0;

    /**
     * Advance every track to `now`, using the commanded velocity as the control input.
     *
     * Defaults to a no-op, so filters whose state only moves when a measurement arrives need no
     * implementation. RobotFrontBackFilter forwards this to its motion estimator: the
     * dead-reckoning estimator only records the control input for correct() to consume, while
     * the Kalman estimator also advances state() to `now` so the output coasts between frames.
     */
    virtual void predict([[maybe_unused]] double now,
                         [[maybe_unused]] CommandFeedback command_feedback) {}

    /**
     * Fold in one perception measurement. `keypoints.header.stamp` is the capture time, which is
     * older than the most recent predict() time, since the control loop outruns perception.
     */
    virtual void correct(KeypointsStamped keypoints, FieldDescription field, CameraInfo camera_info,
                         KeypointsStamped robot_blob_keypoints) = 0;

    /** Best estimate as of the most recent predict() or correct(), whichever ran last. */
    virtual RobotDescriptionsStamped state() const = 0;

    /**
     * True if, on the most recent correct(), our robot's keypoint was missing this frame yet a blob
     * detection coincided with our robot's held pose, and was therefore discarded rather than
     * assigned an opponent FrameId at our own position. Counts the ticks where the suppression
     * actually did work. Filters that do not track this return false.
     */
    virtual bool last_our_blob_present_no_keypoint() const { return false; }
};

}  // namespace auto_battlebot
