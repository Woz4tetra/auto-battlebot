#include "engine_selector/config.hpp"

namespace auto_battlebot {

void EngineSelectorConfiguration::parse(ConfigParser &parser, const std::string &field_name) {
    const toml::table *table = parser.get_table(field_name);
    if (!table) {
        throw ConfigValidationError("Missing required table '" + field_name +
                                    "' (expected a nested [" + field_name +
                                    "] section with a 'candidates' array)");
    }

    ConfigParser child_parser_rejecting_unknown_keys(*table, field_name);
    candidates = child_parser_rejecting_unknown_keys.get_required_vector<std::string>("candidates");
    if (candidates.empty()) {
        throw ConfigValidationError("'" + field_name +
                                    ".candidates' must list at least one engine file");
    }
    child_parser_rejecting_unknown_keys.validate_no_extra_fields();
}

}  // namespace auto_battlebot
