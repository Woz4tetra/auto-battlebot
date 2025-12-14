#pragma once

#include "data_structures.hpp"

namespace auto_battlebot
{
    class NavigationInterface
    {
    public:
        virtual ~NavigationInterface() = default;
        virtual bool initialize() = 0;
        virtual VelocityCommand update(RobotDescriptionsStamped robots) = 0;
    };

} // namespace auto_battlebot
