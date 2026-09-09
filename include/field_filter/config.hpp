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
    /** Slack on the hard (loss-boundary) radius beyond geometry plus our half-diagonal. */
    double hazard_hard_margin_m = 0.02;

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

/** Shared by every filter that fits the field outline in RGB. */
struct FieldOutlineFieldFilterConfiguration : public FieldFilterConfiguration {
    /** The floor mat, measured per venue with the depth path, not read off a rulebook. Nominal
     *  8 ft (2.4384 m) runs about 6% long against measured mats of 2.30 to 2.40 m, and the size
     *  enters the pose linearly. */
    double field_size_x = 2.35;
    double field_size_y = 2.35;
    /** Mask area over quad area. Above this the outline is not a quadrilateral, so at least one
     *  side is not a field edge. Structural, because the residual cannot see it. */
    double max_quad_coverage = 1.05;
    /** Contour points this close to the frame edge are dropped before the edge fit. */
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

/**
 * A cage measured once, then replayed unchanged at match time.
 *
 * The line fit still runs every start as a seating check: a static camera cannot otherwise tell
 * that the fixture did not seat or that the cage got bumped, because there is no visual odometry
 * left to notice. Above `max_seating_error_px` it warns and keeps publishing the stored
 * calibration, which is the answer we trust.
 */
struct CalibratedFieldFilterConfiguration : public FieldOutlineFieldFilterConfiguration {
    /** Per-cage calibration, relative to the config directory: config/cages/<venue>_<cage>.toml */
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

/**
 * Field pose from an AprilTag board on the floor, ignoring the mask entirely.
 *
 * This is the mode for a venue nobody has surveyed: it depends on nothing we have to segment and
 * on no assumed mat size. Put the board at a *near* corner. A 65 mm marker needs about 18 px of
 * edge to decode; at the far mat corner foreshortening leaves it 8 px, and scaling the markers
 * does not rescue that. The near corners are also exactly the ones a cage-mounted camera cannot
 * resolve as mat corners, so the board covers the weak spot.
 */
struct FiducialFieldFilterConfiguration : public FieldFilterConfiguration {
    // Board geometry. Defaults match the manufactured board.
    int board_cols = 3;
    int board_rows = 5;
    double marker_size = 0.065;  // metres, printed edge
    double marker_separation = 0.015;
    int first_marker_id = 160;

    // Where the board sits. `corner` names which field corner it marks; the offsets are the board
    // origin measured from that corner, in the field frame, because the board cannot physically
    // sit in the corner itself.
    FieldCorner corner = FieldCorner::NEG_X_NEG_Y;
    double board_offset_x = 0.0;
    double board_offset_y = 0.0;
    double board_offset_z = 0.0;  // non-zero for a wall-mounted board
    double board_yaw_deg = 0.0;
    double board_pitch_deg = 0.0;  // 90 for a board hung flat on a wall

    // Field extent measured from that corner.
    double field_size_x = 2.35;
    double field_size_y = 2.35;

    int min_markers = 4;
    /** Correspondences are stacked across this many frames before the pose latches, so detection
     *  noise averages out. */
    int accumulate_frames = 10;
    /** A real guard here, unlike the four-corner homography: 15 markers give 60 correspondences
     *  against 6 unknowns, so the residual measures something. It catches a mis-measured
     *  marker_size, a board printed at "fit to page", a mirrored id mapping, and a board that
     *  was not flat. */
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
