#pragma once

#include "data_structures.hpp"

namespace auto_battlebot {
class RobotFilterInterface {
   public:
    virtual ~RobotFilterInterface() = default;
    virtual bool initialize(int opponent_count) = 0;

    virtual void predict([[maybe_unused]] double now,
                         [[maybe_unused]] CommandFeedback command_feedback) {}

    virtual void correct(ModelResultStamped keypoints, FieldDescription field,
                         CameraInfo camera_info) = 0;

    virtual RobotDescriptionsStamped state() const = 0;
};

}  // namespace auto_battlebot
