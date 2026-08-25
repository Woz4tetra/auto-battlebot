#pragma once

#include <string>
#include <vector>

#include "enums.hpp"
#include "field.hpp"
#include "header.hpp"
#include "pose.hpp"
#include "velocity.hpp"

namespace auto_battlebot {
struct RobotDescription {
    FrameId frame_id = FrameId::EMPTY;
    Label label = Label::EMPTY;
    Group group = Group::OURS;
    Pose pose;
    Size size;
    std::vector<Position> keypoints = std::vector<Position>();
    Velocity2D velocity;
    bool is_stale = false;
};

struct RobotDescriptionsStamped {
    Header header;
    std::vector<RobotDescription> descriptions;
};

struct RobotConfig {
    Label label = Label::MR_STABS_MK1;
    Group group = Group::OURS;
};

}  // namespace auto_battlebot
