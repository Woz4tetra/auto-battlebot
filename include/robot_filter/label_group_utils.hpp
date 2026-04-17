#pragma once

#include <unordered_map>

#include "data_structures/robot.hpp"

namespace auto_battlebot {
inline Group group_for_label(Label label,
                             const std::unordered_map<Label, RobotConfig> &robot_configs) {
    auto robot_config_it = robot_configs.find(label);
    if (robot_config_it != robot_configs.end()) {
        return robot_config_it->second.group;
    }
    return Group::THEIRS;
}
}  // namespace auto_battlebot
