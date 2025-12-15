#include "field_model/config.hpp"
#include "field_model/noop_field_model.hpp"

namespace auto_battlebot
{
    std::shared_ptr<FieldModelInterface> make_field_model(const FieldModelConfiguration &config)
    {
        if (config.type == "NoopFieldModel")
        {
            return std::make_shared<NoopFieldModel>();
        }
        return nullptr;
    }
} // namespace auto_battlebot
