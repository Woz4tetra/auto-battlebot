#include "field_filter/config.hpp"
#include "field_filter/noop_field_filter.hpp"

namespace auto_battlebot
{
    std::shared_ptr<FieldFilterInterface> make_field_filter(const FieldFilterConfiguration &config)
    {
        if (config.type == "NoopFieldFilter")
        {
            return std::make_shared<NoopFieldFilter>();
        }
        return nullptr;
    }
} // namespace auto_battlebot
