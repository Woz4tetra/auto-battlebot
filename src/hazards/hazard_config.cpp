#include "hazards/hazard_config.hpp"

#include <toml++/toml.h>

#include <filesystem>
#include <magic_enum.hpp>

#include "directories.hpp"

namespace auto_battlebot {
namespace {
// Hazard paths are written repo-relative ("config/hazards/one_hole.toml") so the same string works
// in the C++ config and in the sim's, which have no directory in common. Each side resolves it
// against the project root itself. Unlike `extends`, this is a plain file path: no implied
// directory and no implied extension.
std::filesystem::path resolve(const std::string &value) {
    std::filesystem::path path(value);
    if (path.is_absolute()) return path;
    return get_project_root() / path;
}

// The file spells kinds in lowercase because simulation/hazards.py reads the same file and
// checks the same words. ROBOT is not a file kind: it marks live tracked robots.
HazardKind parse_kind(const std::string &text, const std::string &where) {
    const auto kind = magic_enum::enum_cast<HazardKind>(text, magic_enum::case_insensitive);
    if (!kind.has_value() || *kind == HazardKind::ROBOT) {
        throw ConfigValidationError(where + " is '" + text + "'; valid values: hole, wall_block");
    }
    return *kind;
}
}  // namespace

std::vector<StaticHazardConfig> load_static_hazards(const std::string &path) {
    const std::filesystem::path resolved = resolve(path);
    if (!std::filesystem::exists(resolved)) {
        throw ConfigValidationError("hazards_file not found: " + resolved.string());
    }

    toml::table data = toml::parse_file(resolved.string());
    for (const auto &[key, _value] : data) {
        if (key.str() != "hazards") {
            throw ConfigValidationError("Unexpected key '" + std::string(key.str()) + "' in " +
                                        resolved.string() + "; expected [[hazards]]");
        }
    }

    const toml::array *entries = data["hazards"].as_array();
    if (entries == nullptr) return {};

    std::vector<StaticHazardConfig> hazards;
    hazards.reserve(entries->size());
    for (size_t i = 0; i < entries->size(); ++i) {
        const toml::table *entry = (*entries)[i].as_table();
        if (entry == nullptr) {
            throw ConfigValidationError("hazards[" + std::to_string(i) + "] in " +
                                        resolved.string() + " is not a table");
        }
        StaticHazardConfig hazard;
        hazard.kind = parse_kind((*entry)["kind"].value_or(std::string("hole")),
                                 "hazards[" + std::to_string(i) + "].kind in " + resolved.string());
        const toml::array *center = (*entry)["center"].as_array();
        if (center == nullptr || center->size() != 2) {
            throw ConfigValidationError("hazards[" + std::to_string(i) + "].center in " +
                                        resolved.string() + " must be [x, y]");
        }
        hazard.center_x = (*center)[0].value_or(0.0);
        hazard.center_y = (*center)[1].value_or(0.0);
        hazard.radius = (*entry)["radius"].value_or(0.0);
        if (hazard.radius <= 0.0) {
            throw ConfigValidationError("hazards[" + std::to_string(i) + "].radius in " +
                                        resolved.string() + " must be > 0");
        }
        hazards.push_back(hazard);
    }
    return hazards;
}
}  // namespace auto_battlebot
