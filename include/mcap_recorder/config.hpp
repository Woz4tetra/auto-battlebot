#pragma once

#include <toml++/toml.h>

#include <cstdint>
#include <string>
#include <vector>

namespace auto_battlebot {

struct McapRecorderConfig {
    bool enable = false;
    std::vector<std::string> ignored_topics;
    uint64_t max_size_gb = 10;
    uint64_t holding_dir_max_size_gb = 50;
    std::string recordings_dir = "data/recordings";
};

McapRecorderConfig load_mcap_config_from_toml(const toml::table& toml_data,
                                              std::vector<std::string>& parsed_sections);

}  // namespace auto_battlebot
