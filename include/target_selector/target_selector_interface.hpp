#pragma once

#include <optional>

#include "data_structures/field.hpp"
#include "data_structures/pose.hpp"
#include "data_structures/robot.hpp"

namespace auto_battlebot {
/** Resolved field-frame target selected from tracking data. */
struct TargetSelection {
    Pose2D pose{};
    Label label = Label::EMPTY;
};

class TargetSelectorInterface {
   public:
    virtual ~TargetSelectorInterface() = default;
    virtual std::optional<TargetSelection> get_target(const RobotDescriptionsStamped &robots,
                                                      const FieldDescription &field) = 0;
};
}  // namespace auto_battlebot
