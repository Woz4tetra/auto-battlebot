#pragma once

#include "data_structures.hpp"

namespace auto_battlebot
{
    struct NavigationConfiguration
    {
        std::string type;
    };

    struct NoopNavigationConfiguration : public NavigationConfiguration
    {
        NoopNavigationConfiguration()
        {
            type = "NoopNavigation";
        }
    };
} // namespace auto_battlebot
