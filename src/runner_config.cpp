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
    parser.validate_no_extra_fields();
    parsed_sections.push_back("runner");
}
}  // namespace auto_battlebot
