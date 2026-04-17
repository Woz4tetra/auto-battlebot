#pragma once

#include "robot_filter/robot_filter_interface.hpp"

namespace auto_battlebot {
class NoopRobotFilter : public RobotFilterInterface {
   public:
    bool initialize([[maybe_unused]] int opponent_count) override {
        return true;
    }

    RobotDescriptionsStamped update([[maybe_unused]] KeypointsStamped keypoints,
                                    [[maybe_unused]] FieldDescription field,
                                    [[maybe_unused]] CameraInfo camera_info,
                                    [[maybe_unused]] KeypointsStamped robot_blob_keypoints,
                                    [[maybe_unused]] CommandFeedback command_feedback) override {
        return RobotDescriptionsStamped{};
    }
};

}  // namespace auto_battlebot
