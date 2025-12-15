#pragma once

#include "navigation/navigation_interface.hpp"

namespace auto_battlebot
{
    class NoopNavigation : public NavigationInterface
    {
    public:
        bool initialize() override { return true; }

        VelocityCommand update(RobotDescriptionsStamped robots) override
        {
            return VelocityCommand{};
        }
    };

} // namespace auto_battlebot
