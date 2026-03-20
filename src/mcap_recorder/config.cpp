#include "mcap_recorder/config.hpp"

#include "config/config_parser.hpp"

namespace auto_battlebot {

McapRecorderConfig load_mcap_config_from_toml(const toml::table& toml_data,
                                              std::vector<std::string>& parsed_sections) {
    McapRecorderConfig config;
    auto section = toml_data["mcap"].as_table();
    if (!section) {
        return config;
    }

    ConfigParser parser(*section, "mcap");
    config.enable = parser.get_optional_bool("enable", config.enable);
    config.ignored_topics = parser.get_optional_vector<std::string>("ignored_topics");
    parser.validate_no_extra_fields();

    parsed_sections.push_back("mcap");
    return config;
}

}  // namespace auto_battlebot
