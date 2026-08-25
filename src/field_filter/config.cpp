#include "field_filter/config.hpp"

#include <spdlog/spdlog.h>
#include <toml++/toml.h>

#include "config/config_cast.hpp"
#include "config/config_parser.hpp"
#include "field_filter/fixed_field_filter.hpp"
#include "field_filter/noop_field_filter.hpp"
#include "field_filter/point_cloud_field_filter.hpp"
#include "hazards/hazard_config.hpp"

namespace auto_battlebot {
// Automatic registration of config types
REGISTER_CONFIG(FieldFilterConfiguration, NoopFieldFilterConfiguration, "NoopFieldFilter")
REGISTER_CONFIG(FieldFilterConfiguration, FixedFieldFilterConfiguration, "FixedFieldFilter")
REGISTER_CONFIG(FieldFilterConfiguration, PointCloudFieldFilterConfiguration,
                "PointCloudFieldFilter")

std::unique_ptr<FieldFilterConfiguration> parse_field_filter_config(ConfigParser &parser) {
    // Hand-rolled rather than create_and_parse, because the hazard fields are shared by every
    // field filter type and have to be read before validate_no_extra_fields rejects them.
    auto &factory = ConfigFactory<FieldFilterConfiguration>::instance();
    auto config = factory.create(parser.get_required_string("type"));
    config->parse_common_fields(parser);
    factory.parse(*config, parser);
    parser.validate_no_extra_fields();
    return config;
}

std::shared_ptr<HazardAssembler> make_hazard_assembler(const FieldFilterConfiguration &config,
                                                       std::shared_ptr<ClockInterface> clock) {
    std::vector<StaticHazardConfig> hazards;
    if (!config.hazards_file.empty()) {
        hazards = load_static_hazards(config.hazards_file);
        spdlog::info("Loaded {} static hazard(s) from {}", hazards.size(), config.hazards_file);
    }
    return std::make_shared<HazardAssembler>(config.hazard_assembler_config(), std::move(hazards),
                                             std::move(clock));
}

std::unique_ptr<FieldFilterConfiguration> load_field_filter_from_toml(
    toml::table const &toml_data, std::vector<std::string> &parsed_sections) {
    auto section = toml_data["field_filter"].as_table();
    if (!section) {
        throw ConfigValidationError("Missing required section [field_filter]");
    }
    ConfigParser parser(*section, "field_filter");
    auto config = parse_field_filter_config(parser);
    parsed_sections.push_back("field_filter");
    return config;
}

std::shared_ptr<FieldFilterInterface> make_field_filter(const FieldFilterConfiguration &config) {
    spdlog::info("Selected {} for FieldFilter", config.type);
    if (config.type == "NoopFieldFilter") {
        return std::make_shared<NoopFieldFilter>();
    } else if (config.type == "FixedFieldFilter") {
        const auto &fixed = config_cast<FixedFieldFilterConfiguration>(config);
        return std::make_shared<FixedFieldFilter>(fixed.size_x, fixed.size_y);
    } else if (config.type == "PointCloudFieldFilter") {
        return std::make_shared<PointCloudFieldFilter>(
            config_cast<PointCloudFieldFilterConfiguration>(config));
    }
    throw std::invalid_argument("Failed to load FieldFilter of type " + config.type);
}
}  // namespace auto_battlebot
