#include "field_model/config.hpp"

#include <toml++/toml.h>

#include "config/config_parser.hpp"
#include "field_model/deeplab_field_model.hpp"
#include "field_model/noop_field_model.hpp"

namespace auto_battlebot {
// Automatic registration of config types
REGISTER_CONFIG(FieldModelConfiguration, NoopFieldModelConfiguration, "NoopFieldModel")
REGISTER_CONFIG(FieldModelConfiguration, DeepLabFieldModelConfiguration, "DeepLabFieldModel")

std::unique_ptr<FieldModelConfiguration> parse_field_model_config(ConfigParser &parser) {
    return ConfigFactory<FieldModelConfiguration>::instance().create_and_parse(parser);
}

std::unique_ptr<FieldModelConfiguration> load_field_model_from_toml(
    toml::table const &toml_data, std::vector<std::string> &parsed_sections) {
    auto section = toml_data["field_model"].as_table();
    if (!section) {
        throw ConfigValidationError("Missing required section [field_model]");
    }
    ConfigParser parser(*section, "field_model");
    auto config = parse_field_model_config(parser);
    parsed_sections.push_back("field_model");
    return config;
}

std::shared_ptr<FieldModelInterface> make_field_model(const FieldModelConfiguration &config) {
    std::cout << "Selected " + config.type + " for FieldModel" << std::endl;
    if (config.type == "NoopFieldModel") {
        return std::make_shared<NoopFieldModel>();
    } else if (config.type == "DeepLabFieldModel") {
        return std::make_shared<DeepLabFieldModel>(
            config_cast<DeepLabFieldModelConfiguration>(config));
    }
    throw std::invalid_argument("Failed to load FieldModel of type " + config.type);
}
}  // namespace auto_battlebot
