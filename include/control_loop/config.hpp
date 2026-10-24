#pragma once

#include <memory>

#include "config/config_factory.hpp"
#include "config/config_parser.hpp"
#include "control_loop/control_loop.hpp"
#include "control_loop/control_loop_interface.hpp"

namespace auto_battlebot {

struct ControlLoopConfiguration {
    std::string type;
    /** Cycles per second. 0 on SteppedControlLoop means one cycle per perception frame. */
    double rate_hz = 250.0;
    /** Threaded driver only: a cycle older than this marks the loop unhealthy. */
    double watchdog_timeout_ms = 20.0;
    /** Longest gap (seconds) RobotDescriptionsCache fills with the last set that had both our robot
     * and an opponent. Past it navigation sees the robots as they are and runs its no-robot
     * path. */
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
        rate_hz = 0.0;  // one cycle per perception frame
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
