#pragma once

#include "data_structures.hpp"

namespace auto_battlebot {
class RobotFilterInterface {
   public:
    virtual ~RobotFilterInterface() = default;
    virtual bool initialize(int opponent_count) = 0;
    virtual RobotDescriptionsStamped update(KeypointsStamped keypoints, FieldDescription field,
                                            CameraInfo camera_info,
                                            KeypointsStamped robot_blob_keypoints,
                                            CommandFeedback command_feedback) = 0;

    /**
     * True if, on the most recent update(), our robot's keypoint was missing this frame yet a blob
     * detection coincided with our robot's held pose -- the exact "leak-opportunity" tick the
     * keypoint-override identity decay would fix (see docs/robot_filter_decay_plan.md). Filters
     * that do not track this return false.
     */
    virtual bool last_our_blob_present_no_keypoint() const { return false; }
};

}  // namespace auto_battlebot
