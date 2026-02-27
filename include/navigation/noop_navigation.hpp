#pragma once

#include "navigation/navigation_interface.hpp"

namespace auto_battlebot {
class NoopNavigation : public NavigationInterface {
   public:
    bool initialize() override { return true; }

    VelocityCommand update([[maybe_unused]] RobotDescriptionsStamped robots,
                           [[maybe_unused]] FieldDescription field) override {
        return VelocityCommand{};
    }
};

}  // namespace auto_battlebot
