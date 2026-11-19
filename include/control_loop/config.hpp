#pragma once

#include <memory>

#include "config/config_factory.hpp"
#include "config/config_parser.hpp"
#include "control_loop/control_loop.hpp"
#include "control_loop/control_loop_interface.hpp"

namespace auto_battlebot {

struct ControlLoopConfiguration {
    std::string type;
    double rate_hz = 250.0;
    double watchdog_timeout_ms = 20.0;
    double robot_cache_max_age_s = 0.25;

    virtual ~ControlLoopConfiguration() = default;
    virtual void parse_fields(ConfigParser &parser) {
        rate_hz = parser.get_optional_double("rate_hz", rate_hz);
        watchdog_timeout_ms =
            parser.get_optional_double("watchdog_timeout_ms", watchdog_timeout_ms);
        robot_cache_max_age_s =
            parser.get_optional_double("robot_cache_max_age_s", robot_cache_max_age_s);
    }
};

struct SteppedControlLoopConfiguration : public ControlLoopConfiguration {
    SteppedControlLoopConfiguration() {
        type = "SteppedControlLoop";
        rate_hz = 0.0;
    }

    int max_catchup_cycles = 64;

    void parse_fields(ConfigParser &parser) override {
        ControlLoopConfiguration::parse_fields(parser);
        max_catchup_cycles =
            static_cast<int>(parser.get_optional_int("max_catchup_cycles", max_catchup_cycles));
    }
};

struct ThreadedControlLoopConfiguration : public ControlLoopConfiguration {
    ThreadedControlLoopConfiguration() { type = "ThreadedControlLoop"; }
};

std::shared_ptr<ControlLoopInterface> make_control_loop(const ControlLoopConfiguration &config,
                                                        std::shared_ptr<ControlLoop> loop);
std::unique_ptr<ControlLoopConfiguration> load_control_loop_from_toml(
    toml::table const &toml_data, std::vector<std::string> &parsed_sections);

}  // namespace auto_battlebot
