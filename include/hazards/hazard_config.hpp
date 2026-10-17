#pragma once

#include <string>
#include <vector>

#include "config/config_parser.hpp"
#include "enums/hazard_kind.hpp"

namespace auto_battlebot {

/** One hazard as it appears in a shared geometry file, before inflation. */
struct StaticHazardConfig {
    /** HOLE or WALL_BLOCK, written "hole" or "wall_block" in the file. HazardAvoidance treats
     * both the same way; RlPolicyNavigation's observation tells them apart. ROBOT is never read
     * from a file: it marks live tracked robots. */
    HazardKind kind = HazardKind::HOLE;
    double center_x = 0.0;
    double center_y = 0.0;
    /** Raw hazard radius (m) in the field frame. Inflation happens at assembly. */
    double radius = 0.0;
};

/** Load the shared arena geometry file named by [field_filter].hazards_file.
 *
 * The same file is read by the kinematic sim (simulation/hazards.py), so the simulated floor and
 * the controller's keep-out discs cannot drift apart. Format:
 *
 *     [[hazards]]
 *     kind = "hole"
 *     center = [0.3, -0.2]
 *     radius = 0.25
 *
 * A missing file throws rather than returning an empty list: the floor hole does not move during
 * a match, so config is the source of truth for it, and a path typo must not silently delete it.
 */
std::vector<StaticHazardConfig> load_static_hazards(const std::string &path);

}  // namespace auto_battlebot
