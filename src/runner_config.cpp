#include "runner_config.hpp"

#include <toml++/toml.h>

#include "config/config_parser.hpp"

namespace auto_battlebot {
void load_runner_from_toml(toml::table const &toml_data, std::vector<std::string> &parsed_sections,
                           RunnerConfiguration &out) {
    auto section = toml_data["runner"].as_table();
    if (!section) {
        return;
    }
    ConfigParser parser(*section, "runner");
    out.max_loop_rate = parser.get_optional_double("max_loop_rate", 300.0);
    out.default_opponent_count = static_cast<int>(
        parser.get_optional_int("default_opponent_count", out.default_opponent_count));
    if (out.default_opponent_count < 1 || out.default_opponent_count > 3) {
        throw ConfigValidationError("runner.default_opponent_count must be 1..3");
    }
    out.autonomy_enabled_by_default =
        parser.get_optional_bool("autonomy_enabled_by_default", out.autonomy_enabled_by_default);
    parser.validate_no_extra_fields();
    parsed_sections.push_back("runner");
}
}  // namespace auto_battlebot
