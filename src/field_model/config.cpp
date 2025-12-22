#include "field_model/config.hpp"
#include "field_model/noop_field_model.hpp"
#include "field_model/deeplab_field_model.hpp"

namespace auto_battlebot
{
    // Automatic registration of config types
    REGISTER_CONFIG(FieldModelConfiguration, NoopFieldModelConfiguration, "NoopFieldModel")
    REGISTER_CONFIG(FieldModelConfiguration, DeepLabFieldModelConfiguration, "DeepLabFieldModel")

    std::unique_ptr<FieldModelConfiguration> parse_field_model_config(ConfigParser &parser)
    {
        return ConfigFactory<FieldModelConfiguration>::instance().create_and_parse(parser);
    }

    std::shared_ptr<FieldModelInterface> make_field_model(const FieldModelConfiguration &config)
    {
        std::cout << "Selected " + config.type + " for FieldModel" << std::endl;
        if (config.type == "NoopFieldModel")
        {
            return std::make_shared<NoopFieldModel>();
        }
        else if (config.type == "DeepLabFieldModel")
        {
            return std::make_shared<DeepLabFieldModel>(
                config_cast<DeepLabFieldModelConfiguration>(config));
        }
        throw std::invalid_argument("Failed to load FieldModel of type " + config.type);
    }
} // namespace auto_battlebot
