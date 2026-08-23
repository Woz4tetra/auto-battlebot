#pragma once

#include "enums/behavior_mode.hpp"
#include "enums/label.hpp"
#include "pose.hpp"

namespace auto_battlebot {
/** Resolved field-frame target selected from tracking data. */
struct TargetSelection {
    Pose2D pose{};
    Label label = Label::EMPTY;
    /** Behavior mode this target was resolved under. Navigation reads it to pick the terminal
     * speed: drive through the opponent in ATTACK, arrive stopped at the safe point in RUN_AWAY.
     * ControlLoop::resolve_target stamps it, so it stays current even when the selector found
     * nothing this cycle and the previous target is reused. Selectors leave it alone. */
    BehaviorMode mode = BehaviorMode::ATTACK;
};
}  // namespace auto_battlebot
