#pragma once

#include "data_structures.hpp"
#include <vector>

namespace auto_battlebot
{
    class RobotFilterInterface
    {
    public:
        virtual ~RobotFilterInterface() = default;
        virtual bool initialize(const std::vector<RobotConfig> &robots) = 0;
        virtual RobotDescriptionsStamped update(KeypointsStamped keypoints, FieldDescription field, CameraInfo camera_info, CommandFeedback command_feedback) = 0;
        /** Set number of opponent slots (1-3). Returns true if supported. */
        virtual bool set_opponent_count(int count) { (void)count; return false; }
    };

} // namespace auto_battlebot
