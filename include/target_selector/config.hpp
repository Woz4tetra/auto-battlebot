#pragma once

#include "config/config_factory.hpp"
#include "config/config_parser.hpp"
#include "target_selector/target_selector_interface.hpp"

namespace auto_battlebot {
struct TargetSelectorConfiguration {
    std::string type;
    virtual ~TargetSelectorConfiguration() = default;
    virtual void parse_fields([[maybe_unused]] ConfigParser &parser) {}
};

struct NearestTargetConfiguration : public TargetSelectorConfiguration {
    NearestTargetConfiguration() { type = "NearestTarget"; }

    PARSE_CONFIG_FIELDS(
        // No additional fields
    )
};

struct NoopTargetConfiguration : public TargetSelectorConfiguration {
    NoopTargetConfiguration() { type = "NoopTarget"; }

    PARSE_CONFIG_FIELDS(
        // No additional fields
    )
};

std::shared_ptr<TargetSelectorInterface> make_target_selector(
    const TargetSelectorConfiguration &config);
std::unique_ptr<TargetSelectorConfiguration> parse_target_selector_config(ConfigParser &parser);
std::unique_ptr<TargetSelectorConfiguration> load_target_selector_from_toml(
    toml::table const &toml_data, std::vector<std::string> &parsed_sections);
}  // namespace auto_battlebot
