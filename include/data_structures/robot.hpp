#pragma once

#include <string>
#include <vector>
#include "header.hpp"
#include "pose.hpp"
#include "field.hpp"
#include "enums.hpp"

namespace auto_battlebot
{
    struct RobotDescription
    {
        FrameId frame_id = FrameId::EMPTY;
        Label label = Label::EMPTY;
        Pose pose;
        Size size;
        std::vector<Position> keypoints = std::vector<Position>();
    };

    struct RobotDescriptionsStamped
    {
        Header header;
        std::vector<RobotDescription> descriptions;
    };

    struct RobotConfig
    {
        Label label = Label::MR_STABS_MK1;
        Group group = Group::OURS;
    };

} // namespace auto_battlebot
