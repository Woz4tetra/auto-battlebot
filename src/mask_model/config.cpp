#include "mask_model/config.hpp"

#include <spdlog/spdlog.h>
#include <toml++/toml.h>

#include "config/config_parser.hpp"
#include "mask_model/deeplab_mask_model.hpp"
#include "mask_model/fixed_mask_model.hpp"
#include "mask_model/free_roam_mask_model.hpp"
#include "mask_model/noop_mask_model.hpp"

namespace auto_battlebot {
void FreeRoamMaskModelConfiguration::parse_fields(ConfigParser &parser) {
    roi_fraction = parser.get_optional_double("roi_fraction", roi_fraction);
    if (roi_fraction <= 0.0 || roi_fraction > 1.0) {
        throw ConfigValidationError(
            "Field 'roi_fraction' must be in (0, 1] in section [field_model]");
    }
    color_filter = parser.get_optional_bool("color_filter", color_filter);
    seed_patch_fraction = parser.get_optional_double("seed_patch_fraction", seed_patch_fraction);
    if (seed_patch_fraction <= 0.0 || seed_patch_fraction > 1.0) {
        throw ConfigValidationError(
            "Field 'seed_patch_fraction' must be in (0, 1] in section [field_model]");
    }
    tolerance_l = parser.get_optional_double("tolerance_l", tolerance_l);
    tolerance_ab = parser.get_optional_double("tolerance_ab", tolerance_ab);
    debug_visualization = parser.get_optional_bool("debug_visualization", debug_visualization);
    parser.validate_no_extra_fields();
}

// Automatic registration of config types
REGISTER_CONFIG(MaskModelConfiguration, NoopMaskModelConfiguration, "NoopMaskModel")
REGISTER_CONFIG(MaskModelConfiguration, FixedMaskModelConfiguration, "FixedMaskModel")
REGISTER_CONFIG(MaskModelConfiguration, FreeRoamMaskModelConfiguration, "FreeRoamMaskModel")
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
    } else if (config.type == "FixedMaskModel") {
        return std::make_shared<FixedMaskModel>();
    } else if (config.type == "FreeRoamMaskModel") {
        return std::make_shared<FreeRoamMaskModel>(
            config_cast<FreeRoamMaskModelConfiguration>(config));
    } else if (config.type == "DeepLabMaskModel") {
        auto &model_config = config_cast<DeepLabMaskModelConfiguration>(config);
        return std::make_shared<DeepLabMaskModel>(
            model_config,
            std::make_shared<EngineSelector>(model_config.engine, "DeepLabMaskModel"));
    }
    throw std::invalid_argument("Failed to load MaskModel of type " + config.type);
}
}  // namespace auto_battlebot
