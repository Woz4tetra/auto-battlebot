#pragma once

#include <string>
#include <vector>

#include "enums.hpp"
#include "field.hpp"
#include "header.hpp"
#include "pose.hpp"

namespace auto_battlebot {
struct Velocity2D {
    double vx = 0.0;     // m/s in field frame x
    double vy = 0.0;     // m/s in field frame y
    double omega = 0.0;  // rad/s heading rate
};

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
