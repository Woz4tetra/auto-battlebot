#include "navigation/config.hpp"
#include "navigation/noop_navigation.hpp"

namespace auto_battlebot
{
    std::shared_ptr<NavigationInterface> make_navigation(const NavigationConfiguration &config)
    {
        if (config.type == "NoopNavigation")
        {
            return std::make_shared<NoopNavigation>();
        }
        return nullptr;
    }
} // namespace auto_battlebot
