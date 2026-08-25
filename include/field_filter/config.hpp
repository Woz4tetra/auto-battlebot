#pragma once

#include "config/config_factory.hpp"
#include "data_structures.hpp"
#include "field_filter/field_filter_interface.hpp"
#include "hazards/hazard_assembler.hpp"
#include "time/clock_interface.hpp"

namespace auto_battlebot {
struct FieldFilterConfiguration {
    std::string type;

    /** Shared arena geometry file, relative to the config directory (".toml" optional). The
     * kinematic sim reads the same file, so the simulated floor and the keep-out discs the
     * controller steers on cannot drift apart. Empty = no static hazards. */
    std::string hazards_file;

    /** Clearance added to a static hazard on top of our robot's half-diagonal. */
    double hazard_static_margin_m = 0.10;
    /** Clearance added to a hazard derived from a live neutral track. */
    double hazard_tracked_margin_m = 0.05;
    /** How long a stale neutral track keeps producing a hazard. */
    double hazard_tracked_hold_s = 0.75;

    virtual ~FieldFilterConfiguration() = default;
    virtual void parse_fields([[maybe_unused]] ConfigParser &parser) {}

    /** Fields every field filter shares, parsed by the loader before the type-specific ones. */
    void parse_common_fields(ConfigParser &parser) {
        hazards_file = parser.get_optional_string("hazards_file", hazards_file);
        hazard_static_margin_m =
            parser.get_optional_double("hazard_static_margin_m", hazard_static_margin_m);
        hazard_tracked_margin_m =
            parser.get_optional_double("hazard_tracked_margin_m", hazard_tracked_margin_m);
        hazard_tracked_hold_s =
            parser.get_optional_double("hazard_tracked_hold_s", hazard_tracked_hold_s);
    }

    HazardAssemblerConfig hazard_assembler_config() const {
        HazardAssemblerConfig config;
        config.static_margin_m = hazard_static_margin_m;
        config.tracked_margin_m = hazard_tracked_margin_m;
        config.tracked_hold_s = hazard_tracked_hold_s;
        return config;
    }
};

struct NoopFieldFilterConfiguration : public FieldFilterConfiguration {
    NoopFieldFilterConfiguration() { type = "NoopFieldFilter"; }

    PARSE_CONFIG_FIELDS(
        // No additional fields
    )
};

struct FixedFieldFilterConfiguration : public FieldFilterConfiguration {
    /** Arena width (m) along field x. */
    double size_x = 2.4;
    /** Arena height (m) along field y. */
    double size_y = 2.4;

    FixedFieldFilterConfiguration() { type = "FixedFieldFilter"; }

    // clang-format off
    PARSE_CONFIG_FIELDS(
        PARSE_FIELD_DOUBLE(size_x)
        PARSE_FIELD_DOUBLE(size_y)
    )
    // clang-format on
};

struct PointCloudFieldFilterConfiguration : public FieldFilterConfiguration {
    double distance_threshold = 0.1;
    bool local_visualize_debug = false;
    double depth_units_per_meter = 1.0;
    int ransac_max_iterations = 1000;
    double ransac_probability = 0.999;

    PointCloudFieldFilterConfiguration() { type = "PointCloudFieldFilter"; }

    // clang-format off
        PARSE_CONFIG_FIELDS(
            PARSE_FIELD_DOUBLE(distance_threshold)
            PARSE_FIELD_BOOL(local_visualize_debug)
            PARSE_FIELD_DOUBLE(depth_units_per_meter)
            PARSE_FIELD(ransac_max_iterations)
            PARSE_FIELD_DOUBLE(ransac_probability)
        )
    // clang-format on
};

std::shared_ptr<FieldFilterInterface> make_field_filter(const FieldFilterConfiguration &config);
std::shared_ptr<HazardAssembler> make_hazard_assembler(const FieldFilterConfiguration &config,
                                                       std::shared_ptr<ClockInterface> clock);
std::unique_ptr<FieldFilterConfiguration> parse_field_filter_config(ConfigParser &parser);
std::unique_ptr<FieldFilterConfiguration> load_field_filter_from_toml(
    toml::table const &toml_data, std::vector<std::string> &parsed_sections);
}  // namespace auto_battlebot
