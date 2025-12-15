#pragma once

#include "data_structures.hpp"
#include "navigation/navigation_interface.hpp"

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

    std::shared_ptr<NavigationInterface> make_navigation(const NavigationConfiguration &config);
} // namespace auto_battlebot
