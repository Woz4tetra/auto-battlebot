#pragma once

#include <toml++/toml.h>

#include <string>
#include <vector>

namespace auto_battlebot {

struct McapRecorderConfig {
    std::vector<std::string> ignored_topics;
};

McapRecorderConfig load_mcap_config_from_toml(const toml::table& toml_data,
                                              std::vector<std::string>& parsed_sections);

}  // namespace auto_battlebot
