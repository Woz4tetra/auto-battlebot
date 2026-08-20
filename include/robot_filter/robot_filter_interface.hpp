#pragma once

#include "data_structures.hpp"

namespace auto_battlebot {
class RobotFilterInterface {
   public:
    virtual ~RobotFilterInterface() = default;
    virtual bool initialize(int opponent_count) = 0;
    /**
     * @brief Update the all robot estimates
     *
     * @param keypoints Keypoints of *our* robot(s).
     * @param field Estimated field, relative to the camera.
     * @param camera_info Camera params.
     * @param robot_blob_keypoints Opponent (possibly our robot) blob keypoints.
     * @param command_feedback Feedback from the transmitted?
     * @return Best estimate of all robots in the field.
     */
    virtual RobotDescriptionsStamped update(KeypointsStamped keypoints, FieldDescription field,
                                            CameraInfo camera_info,
                                            KeypointsStamped robot_blob_keypoints,
                                            CommandFeedback command_feedback) = 0;

    /**
     * True if, on the most recent update(), our robot's keypoint was missing this frame yet a blob
     * detection coincided with our robot's held pose, and was therefore discarded rather than
     * assigned an opponent FrameId at our own position. Counts the ticks where the suppression
     * actually did work. Filters that do not track this return false.
     */
    virtual bool last_our_blob_present_no_keypoint() const { return false; }
};

}  // namespace auto_battlebot
