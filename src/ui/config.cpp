#include "ui/config.hpp"

#include <toml++/toml.h>

#include <algorithm>
#include <cctype>

namespace auto_battlebot {
void UiConfiguration::parse_fields(ConfigParser &parser) {
    rate_fail_threshold = parser.get_optional_double("rate_fail_threshold", rate_fail_threshold);
    rate_fail_duration_sec =
        parser.get_optional_double("rate_fail_duration_sec", rate_fail_duration_sec);
    if (const toml::table *colors = parser.get_table("label_colors")) {
        label_colors.clear();
        for (const auto &[key, value] : *colors) {
            std::string label(key.str());
            std::transform(label.begin(), label.end(), label.begin(),
                           [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            const auto color = value.value<std::string>();
            const bool hex = color && color->size() == 7 && (*color)[0] == '#' &&
                             std::all_of(color->begin() + 1, color->end(),
                                         [](unsigned char c) { return std::isxdigit(c) != 0; });
            if (!hex) {
                throw ConfigValidationError("[ui.label_colors] " + label +
                                            " must be a \"#rrggbb\" string");
            }
            label_colors.emplace_back(std::move(label), *color);
        }
    }
}

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
