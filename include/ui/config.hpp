#pragma once

#include <memory>
#include <stdexcept>
#include <string>

#include "config/config_cast.hpp"
#include "config/config_factory.hpp"
#include "config/config_parser.hpp"
#include "data_structures.hpp"
#include "enums.hpp"
#include "rgbd_camera/rgbd_camera_interface.hpp"
#include "ui/battery_options.hpp"

namespace auto_battlebot {
struct UiConfiguration {
    ~UiConfiguration() = default;
    UiConfiguration() = default;

    bool enable = false;
    bool fullscreen = true;
    int width = 1024;
    int height = 600;
    /** Number of samples for rolling average of loop_rate_hz (1 = no averaging). */
    int rate_avg_window = 10;
    /** Fraction of max_loop_rate below which rate is considered "not met" (e.g. 0.99). */
    double rate_fail_threshold = 0.5;
    /** Seconds the rate must be below threshold before showing error (prevents strobing). */
    double rate_fail_duration_sec = 2.0;
    /** Battery source for top bar status ("Waveshare UPS" or "None"). */
    std::string battery_source = "Waveshare UPS";
    /** Battery estimator settings under [ui.battery]. */
    BatteryOptions battery;

    void parse_fields(ConfigParser &parser);
};
std::unique_ptr<UiConfiguration> parse_ui_config(ConfigParser &parser);
std::unique_ptr<UiConfiguration> load_ui_from_toml(toml::table const &toml_data,
                                                   std::vector<std::string> &parsed_sections);
}  // namespace auto_battlebot
