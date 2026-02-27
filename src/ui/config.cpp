#include "ui/config.hpp"

#include <toml++/toml.h>

namespace auto_battlebot {
std::unique_ptr<UiConfiguration> parse_ui_config(ConfigParser &parser) {
    auto config = std::make_unique<UiConfiguration>();
    config->parse_fields(parser);
    parser.validate_no_extra_fields();
    return config;
}

std::unique_ptr<UiConfiguration> load_ui_from_toml(toml::table const &toml_data,
                                                   std::vector<std::string> &parsed_sections) {
    auto section = toml_data["ui"].as_table();
    if (!section) {
        return nullptr;
    }
    ConfigParser parser(*section, "ui");
    auto config = parse_ui_config(parser);
    parsed_sections.push_back("ui");
    return config;
}
}  // namespace auto_battlebot
