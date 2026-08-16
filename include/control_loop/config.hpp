#pragma once

#include <memory>

#include "config/config_factory.hpp"
#include "config/config_parser.hpp"
#include "control_loop/control_loop.hpp"
#include "control_loop/control_loop_interface.hpp"

namespace auto_battlebot {

struct ControlLoopConfiguration {
    std::string type;
    /** Cycles per second. 0 on SteppedControlLoop means one cycle per perception frame, which
     *  reproduces the pre-Phase-2 pipeline. Ignored shape-wise by drivers that pace themselves. */
    double rate_hz = 250.0;
    /** Threaded driver only: a cycle older than this marks the loop unhealthy. */
    double watchdog_timeout_ms = 20.0;

    virtual ~ControlLoopConfiguration() = default;
    virtual void parse_fields(ConfigParser &parser) {
        rate_hz = parser.get_optional_double("rate_hz", rate_hz);
        watchdog_timeout_ms =
            parser.get_optional_double("watchdog_timeout_ms", watchdog_timeout_ms);
    }
};

struct SteppedControlLoopConfiguration : public ControlLoopConfiguration {
    SteppedControlLoopConfiguration() {
        type = "SteppedControlLoop";
        rate_hz = 0.0;  // default to pre-Phase-2 behavior: one cycle per perception frame
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
