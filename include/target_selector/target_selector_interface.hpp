#pragma once

#include <optional>

#include "data_structures/field.hpp"
#include "data_structures/robot.hpp"
#include "data_structures/target_selection.hpp"
#include "enums/group.hpp"

namespace auto_battlebot {
class TargetSelectorInterface {
   public:
    virtual ~TargetSelectorInterface() = default;
    virtual std::optional<TargetSelection> get_target(const RobotDescriptionsStamped &robots,
                                                      const FieldDescription &field) = 0;
};
}  // namespace auto_battlebot
