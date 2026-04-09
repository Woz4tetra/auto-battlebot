#pragma once

#include "enums/label.hpp"
#include "pose.hpp"

namespace auto_battlebot {
/** Resolved field-frame target selected from tracking data. */
struct TargetSelection {
    Pose2D pose{};
    Label label = Label::EMPTY;
};
}  // namespace auto_battlebot
