#pragma once

#include "navigation/navigation_interface.hpp"

namespace auto_battlebot {
class NoopNavigation : public NavigationInterface {
   public:
    bool initialize() override { return true; }

    VelocityCommand update(RobotDescriptionsStamped robots,
                           [[maybe_unused]] FieldDescription field,
                           [[maybe_unused]] const TargetSelection &target) override {
        VelocityCommand cmd{};
        last_visualization_ = NavigationVisualization{};
        last_visualization_.header = robots.header;
        last_visualization_.command = cmd;
        last_visualization_.robots = std::move(robots);
        return cmd;
    }

    const NavigationVisualization &get_last_visualization() const override {
        return last_visualization_;
    }

   private:
    NavigationVisualization last_visualization_;
};

}  // namespace auto_battlebot
