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
};

}  // namespace auto_battlebot
