#include "control_loop/config.hpp"

#include <spdlog/spdlog.h>
#include <toml++/toml.h>

#include "control_loop/stepped_control_loop.hpp"
#include "control_loop/threaded_control_loop.hpp"

namespace auto_battlebot {

std::unique_ptr<ControlLoopConfiguration> load_control_loop_from_toml(
    toml::table const &toml_data, std::vector<std::string> &parsed_sections) {
    auto section = toml_data["control_loop"].as_table();
    if (!section) {
        // Absent section means one cycle per perception frame with no thread, so configs written
        // before the control loop existed keep their original behavior.
        return std::make_unique<SteppedControlLoopConfiguration>();
    }
    ConfigParser parser(*section, "control_loop");
    const std::string type = parser.get_optional_string("type", "SteppedControlLoop");

    std::unique_ptr<ControlLoopConfiguration> config;
    if (type == "SteppedControlLoop") {
        config = std::make_unique<SteppedControlLoopConfiguration>();
    } else if (type == "ThreadedControlLoop") {
        config = std::make_unique<ThreadedControlLoopConfiguration>();
    } else {
        throw ConfigValidationError("Unknown control_loop type: " + type);
    }
    config->parse_fields(parser);
    parser.validate_no_extra_fields();
    parsed_sections.push_back("control_loop");
    return config;
}

std::shared_ptr<ControlLoopInterface> make_control_loop(const ControlLoopConfiguration &config,
                                                        std::shared_ptr<ControlLoop> loop) {
    spdlog::info("Selected {} for ControlLoop at {} Hz", config.type, config.rate_hz);
    if (config.type == "SteppedControlLoop") {
        return std::make_shared<SteppedControlLoop>(std::move(loop), config.rate_hz);
    } else if (config.type == "ThreadedControlLoop") {
        return std::make_shared<ThreadedControlLoop>(std::move(loop), config.rate_hz,
                                                     config.watchdog_timeout_ms);
    }
    throw std::invalid_argument("Failed to load ControlLoop of type " + config.type);
}

}  // namespace auto_battlebot
