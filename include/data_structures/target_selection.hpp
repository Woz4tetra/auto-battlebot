#pragma once

#include "enums/behavior_mode.hpp"
#include "enums/label.hpp"
#include "pose.hpp"

namespace auto_battlebot {
struct TargetSelection {
    Pose2D pose{};
    Label label = Label::EMPTY;
    BehaviorMode mode = BehaviorMode::ATTACK;
};
}  // namespace auto_battlebot
