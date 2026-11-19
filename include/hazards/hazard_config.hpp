#pragma once

#include <string>
#include <vector>

#include "config/config_parser.hpp"
#include "enums/hazard_kind.hpp"

namespace auto_battlebot {

struct StaticHazardConfig {
    HazardKind kind = HazardKind::HOLE;
    double center_x = 0.0;
    double center_y = 0.0;
    double raw_radius_m = 0.0;
};

std::vector<StaticHazardConfig> load_static_hazards(const std::string &path);

}  // namespace auto_battlebot
