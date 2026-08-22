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

struct SafestPointTargetConfiguration : public TargetSelectorConfiguration {
    /** Radius (m) a new candidate must beat the held target by before retargeting.
     *  Suppresses solver jitter as opponents move. */
    double retarget_improvement_m = 0.0;

    SafestPointTargetConfiguration() { type = "SafestPointTarget"; }

    PARSE_CONFIG_FIELDS(PARSE_FIELD_DOUBLE(retarget_improvement_m))
};

std::shared_ptr<TargetSelectorInterface> make_target_selector(
    const TargetSelectorConfiguration &config);
std::unique_ptr<TargetSelectorConfiguration> parse_target_selector_config(ConfigParser &parser);
std::unique_ptr<TargetSelectorConfiguration> load_target_selector_from_toml(
    toml::table const &toml_data, std::vector<std::string> &parsed_sections);
}  // namespace auto_battlebot
