#pragma once

#include "target_selector/target_selector_interface.hpp"

namespace auto_battlebot {
class NoopTarget : public TargetSelectorInterface {
   public:
    std::optional<TargetSelection> get_target(const RobotDescriptionsStamped &,
                                              const FieldDescription &) override {
        TargetSelection selection;
        selection.pose.x = 0.0;
        selection.pose.y = 0.0;
        selection.pose.yaw = 0.0;
        return selection;
    }
};
}  // namespace auto_battlebot
