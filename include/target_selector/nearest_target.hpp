#pragma once

#include "target_selector/target_selector_interface.hpp"

namespace auto_battlebot {
class NearestTarget : public TargetSelectorInterface {
   public:
    std::optional<TargetSelection> get_target(const RobotDescriptionsStamped &robots,
                                              const FieldDescription &field) override;
};
}  // namespace auto_battlebot
