#pragma once

#include <toml++/toml.h>

#include <cstdint>
#include <string>
#include <vector>

namespace auto_battlebot {

struct McapRecorderConfig {
    bool enable = false;
    std::vector<std::string> ignored_topics;
    /** Roll to a new file past this size. Video moved into this file, so it is the large one now.
     */
    uint64_t max_size_gb = 10;
    /** Oldest recordings in the output directory are evicted to stay under this. */
    uint64_t holding_dir_max_size_gb = 50;
};

McapRecorderConfig load_mcap_config_from_toml(const toml::table& toml_data,
                                              std::vector<std::string>& parsed_sections);

}  // namespace auto_battlebot
