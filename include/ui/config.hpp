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
/** [ui]: settings the web dashboard reads from /status/app. */
struct UiConfiguration {
    ~UiConfiguration() = default;
    UiConfiguration() = default;

    /** Fraction of max_loop_rate below which rate is considered "not met" (e.g. 0.99). */
    double rate_fail_threshold = 0.5;
    /** Seconds the rate must be below threshold before showing error (prevents strobing). */
    double rate_fail_duration_sec = 2.0;
    /** [ui.label_colors]: lowercase model label -> "#rrggbb", the colors training draws each
     *  class in. The web dashboard uses them for boxes and tracks; labels not listed get a
     *  generated color. */
    std::vector<std::pair<std::string, std::string>> label_colors;

    void parse_fields(ConfigParser &parser);
};
std::unique_ptr<UiConfiguration> parse_ui_config(ConfigParser &parser);
std::unique_ptr<UiConfiguration> load_ui_from_toml(toml::table const &toml_data,
                                                   std::vector<std::string> &parsed_sections);
}  // namespace auto_battlebot
