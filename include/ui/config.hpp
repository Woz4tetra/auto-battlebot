#pragma once

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "config/config_cast.hpp"
#include "config/config_factory.hpp"
#include "config/config_parser.hpp"
#include "data_structures.hpp"
#include "enums.hpp"
#include "rgbd_camera/rgbd_camera_interface.hpp"

namespace auto_battlebot {
struct UiConfiguration {
    ~UiConfiguration() = default;
    UiConfiguration() = default;

    double rate_fail_threshold = 0.5;
    double rate_fail_duration_sec = 2.0;
    std::vector<std::pair<std::string, std::string>> label_colors;

    void parse_fields(ConfigParser &parser);
};
std::unique_ptr<UiConfiguration> parse_ui_config(ConfigParser &parser);
std::unique_ptr<UiConfiguration> load_ui_from_toml(toml::table const &toml_data,
                                                   std::vector<std::string> &parsed_sections);
}  // namespace auto_battlebot
