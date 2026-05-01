#include "target_selector/config.hpp"

#include <spdlog/spdlog.h>

#include "target_selector/nearest_target.hpp"
#include "target_selector/noop_target.hpp"

namespace auto_battlebot {
REGISTER_CONFIG(TargetSelectorConfiguration, NearestTargetConfiguration, "NearestTarget")
REGISTER_CONFIG(TargetSelectorConfiguration, NoopTargetConfiguration, "NoopTarget")

std::unique_ptr<TargetSelectorConfiguration> parse_target_selector_config(ConfigParser &parser) {
    return ConfigFactory<TargetSelectorConfiguration>::instance().create_and_parse(parser);
}

std::unique_ptr<TargetSelectorConfiguration> load_target_selector_from_toml(
    toml::table const &toml_data, std::vector<std::string> &parsed_sections) {
    auto section = toml_data["target_selector"].as_table();
    if (!section) {
        throw ConfigValidationError("Missing required section [target_selector]");
    }
    ConfigParser parser(*section, "target_selector");
    auto config = parse_target_selector_config(parser);
    parsed_sections.push_back("target_selector");
    return config;
}

std::shared_ptr<TargetSelectorInterface> make_target_selector(
    const TargetSelectorConfiguration &config) {
    spdlog::info("Selected {} for TargetSelector", config.type);
    if (config.type == "NearestTarget") {
        return std::make_shared<NearestTarget>();
    }
    if (config.type == "NoopTarget") {
        return std::make_shared<NoopTarget>();
    }
    throw std::invalid_argument("Failed to load TargetSelector of type " + config.type);
}
}  // namespace auto_battlebot
