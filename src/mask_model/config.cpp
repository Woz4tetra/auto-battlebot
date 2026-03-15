#include "mask_model/config.hpp"

#include <spdlog/spdlog.h>
#include <toml++/toml.h>

#include "config/config_parser.hpp"
#include "mask_model/deeplab_mask_model.hpp"
#include "mask_model/noop_mask_model.hpp"

namespace auto_battlebot {
// Automatic registration of config types
REGISTER_CONFIG(MaskModelConfiguration, NoopMaskModelConfiguration, "NoopMaskModel")
REGISTER_CONFIG(MaskModelConfiguration, DeepLabMaskModelConfiguration, "DeepLabMaskModel")

std::unique_ptr<MaskModelConfiguration> parse_mask_model_config(ConfigParser &parser) {
    return ConfigFactory<MaskModelConfiguration>::instance().create_and_parse(parser);
}

std::unique_ptr<MaskModelConfiguration> load_field_model_from_toml(
    toml::table const &toml_data, std::vector<std::string> &parsed_sections) {
    auto section = toml_data["field_model"].as_table();
    if (!section) {
        throw ConfigValidationError("Missing required section [field_model]");
    }
    ConfigParser parser(*section, "field_model");
    auto config = parse_mask_model_config(parser);
    parsed_sections.push_back("field_model");
    return config;
}

std::unique_ptr<MaskModelConfiguration> load_robot_mask_model_from_toml(
    toml::table const &toml_data, std::vector<std::string> &parsed_sections) {
    auto section = toml_data["robot_mask_model"].as_table();
    if (!section) {
        throw ConfigValidationError("Missing required section [robot_mask_model]");
    }
    ConfigParser parser(*section, "robot_mask_model");
    auto config = parse_mask_model_config(parser);
    parsed_sections.push_back("robot_mask_model");
    return config;
}

std::shared_ptr<MaskModelInterface> make_mask_model(const MaskModelConfiguration &config) {
    spdlog::info("Selected {} for MaskModel", config.type);
    if (config.type == "NoopMaskModel") {
        return std::make_shared<NoopMaskModel>();
    } else if (config.type == "DeepLabMaskModel") {
        return std::make_shared<DeepLabMaskModel>(
            config_cast<DeepLabMaskModelConfiguration>(config));
    }
    throw std::invalid_argument("Failed to load MaskModel of type " + config.type);
}
}  // namespace auto_battlebot
