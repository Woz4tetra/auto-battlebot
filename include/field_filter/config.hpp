#pragma once

#include "config/config_factory.hpp"
#include "data_structures.hpp"
#include "field_filter/field_filter_interface.hpp"
#include "hazards/hazard_assembler.hpp"
#include "time/clock_interface.hpp"

namespace auto_battlebot {
struct FieldFilterConfiguration {
    std::string type;

    std::string hazards_file;

    double hazard_static_margin_m = 0.10;
    double hazard_tracked_margin_m = 0.05;
    double hazard_tracked_hold_s = 0.75;
    double hazard_hard_margin_m = 0.02;

    virtual ~FieldFilterConfiguration() = default;
    virtual void parse_fields([[maybe_unused]] ConfigParser &parser) {}

    void parse_common_fields(ConfigParser &parser) {
        hazards_file = parser.get_optional_string("hazards_file", hazards_file);
        hazard_static_margin_m =
            parser.get_optional_double("hazard_static_margin_m", hazard_static_margin_m);
        hazard_tracked_margin_m =
            parser.get_optional_double("hazard_tracked_margin_m", hazard_tracked_margin_m);
        hazard_tracked_hold_s =
            parser.get_optional_double("hazard_tracked_hold_s", hazard_tracked_hold_s);
        hazard_hard_margin_m =
            parser.get_optional_double("hazard_hard_margin_m", hazard_hard_margin_m);
    }

    HazardAssemblerConfig hazard_assembler_config() const {
        HazardAssemblerConfig config;
        config.static_margin_m = hazard_static_margin_m;
        config.tracked_margin_m = hazard_tracked_margin_m;
        config.tracked_hold_s = hazard_tracked_hold_s;
        config.hard_margin_m = hazard_hard_margin_m;
        return config;
    }
};

struct NoopFieldFilterConfiguration : public FieldFilterConfiguration {
    NoopFieldFilterConfiguration() { type = "NoopFieldFilter"; }

    PARSE_CONFIG_FIELDS()
};

struct FixedFieldFilterConfiguration : public FieldFilterConfiguration {
    double size_x = 2.4;
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

struct FieldOutlineFieldFilterConfiguration : public FieldFilterConfiguration {
    double field_size_x = 2.35;
    double field_size_y = 2.35;
    double max_quad_coverage = 1.05;
    int border_margin_px = 2;
    int refine_iterations = 4;
    double corner_skip_fraction = 0.20;
    int min_side_points = 20;

    // clang-format off
    void parse_outline_fields(ConfigParser &parser) {
        PARSE_FIELD_DOUBLE(field_size_x)
        PARSE_FIELD_DOUBLE(field_size_y)
        PARSE_FIELD_DOUBLE(max_quad_coverage)
        PARSE_FIELD(border_margin_px)
        PARSE_FIELD(refine_iterations)
        PARSE_FIELD_DOUBLE(corner_skip_fraction)
        PARSE_FIELD(min_side_points)
    }
    // clang-format on
};

struct HomographyFieldFilterConfiguration : public FieldOutlineFieldFilterConfiguration {
    HomographyFieldFilterConfiguration() { type = "HomographyFieldFilter"; }

    PARSE_CONFIG_FIELDS(parse_outline_fields(parser);)
};

struct CalibratedFieldFilterConfiguration : public FieldOutlineFieldFilterConfiguration {
    std::string calibration_file;
    bool seating_check = true;
    double max_seating_error_px = 5.0;

    CalibratedFieldFilterConfiguration() { type = "CalibratedFieldFilter"; }

    // clang-format off
    PARSE_CONFIG_FIELDS(
        parse_outline_fields(parser);
        PARSE_FIELD_STRING(calibration_file)
        PARSE_FIELD_BOOL(seating_check)
        PARSE_FIELD_DOUBLE(max_seating_error_px)
    )
    // clang-format on
};

struct FiducialFieldFilterConfiguration : public FieldFilterConfiguration {
    int board_cols = 3;
    int board_rows = 5;
    double marker_size = 0.065;
    double marker_separation = 0.015;
    int first_marker_id = 160;

    FieldCorner corner = FieldCorner::NEG_X_NEG_Y;
    double board_offset_x = 0.0;
    double board_offset_y = 0.0;
    double board_offset_z = 0.0;
    double board_yaw_deg = 0.0;
    double board_pitch_deg = 0.0;

    double field_size_x = 2.35;
    double field_size_y = 2.35;

    int min_markers = 4;
    int accumulate_frames = 10;
    double max_reprojection_error_px = 3.0;

    FiducialFieldFilterConfiguration() { type = "FiducialFieldFilter"; }

    // clang-format off
    PARSE_CONFIG_FIELDS(
        PARSE_FIELD(board_cols)
        PARSE_FIELD(board_rows)
        PARSE_FIELD_DOUBLE(marker_size)
        PARSE_FIELD_DOUBLE(marker_separation)
        PARSE_FIELD(first_marker_id)
        PARSE_ENUM(corner, FieldCorner)
        PARSE_FIELD_DOUBLE(board_offset_x)
        PARSE_FIELD_DOUBLE(board_offset_y)
        PARSE_FIELD_DOUBLE(board_offset_z)
        PARSE_FIELD_DOUBLE(board_yaw_deg)
        PARSE_FIELD_DOUBLE(board_pitch_deg)
        PARSE_FIELD_DOUBLE(field_size_x)
        PARSE_FIELD_DOUBLE(field_size_y)
        PARSE_FIELD(min_markers)
        PARSE_FIELD(accumulate_frames)
        PARSE_FIELD_DOUBLE(max_reprojection_error_px)
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
