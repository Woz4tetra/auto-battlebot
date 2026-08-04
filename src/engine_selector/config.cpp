#include "engine_selector/config.hpp"

namespace auto_battlebot {

void EngineSelectorConfiguration::parse(ConfigParser &parser, const std::string &field_name) {
    const toml::table *table = parser.get_table(field_name);
    if (!table) {
        throw ConfigValidationError("Missing required table '" + field_name +
                                    "' (expected a nested [" + field_name +
                                    "] section with a 'candidates' array)");
    }

    // A child parser so unknown keys inside the subconfig are rejected the same way the
    // parent section rejects them.
    ConfigParser sub_parser(*table, field_name);
    candidates = sub_parser.get_required_vector<std::string>("candidates");
    if (candidates.empty()) {
        throw ConfigValidationError("'" + field_name +
                                    ".candidates' must list at least one engine file");
    }
    sub_parser.validate_no_extra_fields();
}

}  // namespace auto_battlebot
