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

inline Group group_for_frame_id(FrameId frame_id, Group fallback_group) {
    switch (frame_id) {
        case FrameId::OUR_ROBOT_1:
        case FrameId::OUR_ROBOT_2:
            return Group::OURS;
        case FrameId::THEIR_ROBOT_1:
        case FrameId::THEIR_ROBOT_2:
        case FrameId::THEIR_ROBOT_3:
            return Group::THEIRS;
        case FrameId::NEUTRAL_ROBOT_1:
        case FrameId::NEUTRAL_ROBOT_2:
            return Group::NEUTRAL;
        default:
            return fallback_group;
    }
}
}  // namespace auto_battlebot
