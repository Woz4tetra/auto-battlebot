#pragma once

#include <string>
#include <vector>
#include "header.hpp"
#include "pose.hpp"
#include "field.hpp"

namespace auto_battlebot {

struct RobotDescription {
    std::string label;
    std::string group;
    Pose pose;
    Size size;
};

struct RobotDescriptionsStamped {
    Header header;
    std::vector<RobotDescription> descriptions;
};

struct RobotConfig {
    std::string label;
    std::string group;
};

}  // namespace auto_battlebot
