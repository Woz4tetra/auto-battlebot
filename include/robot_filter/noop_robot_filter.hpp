#pragma once

#include "robot_filter/robot_filter_interface.hpp"

namespace auto_battlebot {
class NoopRobotFilter : public RobotFilterInterface {
   public:
    bool initialize([[maybe_unused]] int opponent_count) override { return true; }

    void correct([[maybe_unused]] ModelResultStamped keypoints,
                 [[maybe_unused]] FieldDescription field, [[maybe_unused]] CameraInfo camera_info,
                 [[maybe_unused]] ModelResultStamped robot_blob_keypoints) override {}

    RobotDescriptionsStamped state() const override { return RobotDescriptionsStamped{}; }
};

}  // namespace auto_battlebot
