#include "transmitter/config.hpp"

#include <spdlog/spdlog.h>
#include <toml++/toml.h>

#include "config/config_cast.hpp"
#include "config/config_parser.hpp"
#include "transmitter/composite_transmitter.hpp"
#include "transmitter/esp32_diagnostics_opentx_transmitter.hpp"
#include "transmitter/espnow_transmitter.hpp"
#include "transmitter/noop_transmitter.hpp"
#include "transmitter/opentx_transmitter.hpp"
#include "transmitter/playback_transmitter.hpp"
#include "transmitter/sim_transmitter.hpp"

namespace auto_battlebot {
// Automatic registration of config types
REGISTER_CONFIG(TransmitterConfiguration, NoopTransmitterConfiguration, "NoopTransmitter")
REGISTER_CONFIG(TransmitterConfiguration, PlaybackTransmitterConfiguration, "PlaybackTransmitter")
REGISTER_CONFIG(TransmitterConfiguration, OpenTxTransmitterConfiguration, "OpenTxTransmitter")
REGISTER_CONFIG(TransmitterConfiguration, Esp32DiagnosticsOpenTxTransmitterConfiguration,
                "Esp32DiagnosticsOpenTxTransmitter")
REGISTER_CONFIG(TransmitterConfiguration, SimTransmitterConfiguration, "SimTransmitter")
REGISTER_CONFIG(TransmitterConfiguration, EspNowTransmitterConfiguration, "EspNowTransmitter")
REGISTER_CONFIG(TransmitterConfiguration, CompositeTransmitterConfiguration, "CompositeTransmitter")

namespace {

// The ESP-NOW child and the OpenTX child each run their own drive processor over the same
// command. If their settings differ the two transports disagree by a fixed offset, so refuse it
// at parse time.
void check_processor_settings_match(const OpenTxTransmitterConfiguration &opentx,
                                    const EspNowTransmitterConfiguration &espnow) {
    const auto mismatch = [](const std::string &field) {
        throw ConfigValidationError("[[transmitter.children]] EspNowTransmitter." + field +
                                    " must equal the OpenTX child's " + field);
    };
    if (espnow.drive_processor != opentx.drive_processor) mismatch("drive_processor");
    if (espnow.velocity_saturation_limit != opentx.velocity_saturation_limit) {
        mismatch("velocity_saturation_limit");
    }
    if (espnow.zero_deadzone_percent != opentx.zero_deadzone_percent) {
        mismatch("zero_deadzone_percent");
    }
    if (espnow.lifted_deadzone_percent != opentx.lifted_deadzone_percent) {
        mismatch("lifted_deadzone_percent");
    }
    if (espnow.reverse_linear_channel != opentx.reverse_linear_channel) {
        mismatch("reverse_linear_channel");
    }
    if (espnow.reverse_angular_channel != opentx.reverse_angular_channel) {
        mismatch("reverse_angular_channel");
    }
}

}  // namespace

void CompositeTransmitterConfiguration::parse_fields(ConfigParser &parser) {
    const toml::array *array_ptr = parser.get_array("children");
    if (!array_ptr) {
        throw ConfigValidationError(
            "Missing required field 'children' in [transmitter] (expected an array of tables)");
    }
    for (const auto &item : *array_ptr) {
        const toml::table *entry = item.as_table();
        if (!entry) {
            throw ConfigValidationError("Each entry in [transmitter] 'children' must be a table");
        }
        // transmit_rate_hz belongs to the composite, not the child, so take it out before the child
        // parser checks for unknown fields.
        toml::table child_table = *entry;
        double rate_hz = 0.0;
        if (const auto rate = child_table["transmit_rate_hz"].value<double>()) {
            rate_hz = *rate;
            child_table.erase("transmit_rate_hz");
        }
        if (rate_hz < 0.0) {
            throw ConfigValidationError("[[transmitter.children]] transmit_rate_hz must be >= 0");
        }
        ConfigParser child_parser(child_table, "transmitter.children");
        auto child = parse_transmitter_config(child_parser);
        if (child->type == "CompositeTransmitter") {
            throw ConfigValidationError("CompositeTransmitter cannot nest inside itself");
        }
        children.push_back(std::move(child));
        transmit_rate_hz.push_back(rate_hz);
    }
    if (children.empty()) {
        throw ConfigValidationError("[transmitter] 'children' must hold at least one transmitter");
    }

    const OpenTxTransmitterConfiguration *opentx = nullptr;
    for (const auto &child : children) {
        if (!opentx) opentx = dynamic_cast<const OpenTxTransmitterConfiguration *>(child.get());
    }
    for (const auto &child : children) {
        const auto *espnow = dynamic_cast<const EspNowTransmitterConfiguration *>(child.get());
        if (espnow && opentx) check_processor_settings_match(*opentx, *espnow);
    }
}

std::unique_ptr<TransmitterConfiguration> parse_transmitter_config(ConfigParser &parser) {
    return ConfigFactory<TransmitterConfiguration>::instance().create_and_parse(parser);
}

std::unique_ptr<TransmitterConfiguration> load_transmitter_from_toml(
    toml::table const &toml_data, std::vector<std::string> &parsed_sections) {
    auto section = toml_data["transmitter"].as_table();
    if (!section) {
        throw ConfigValidationError("Missing required section [transmitter]");
    }
    ConfigParser parser(*section, "transmitter");
    auto config = parse_transmitter_config(parser);
    parsed_sections.push_back("transmitter");
    return config;
}

std::shared_ptr<TransmitterInterface> make_transmitter(
    const TransmitterConfiguration &config, std::shared_ptr<ClockInterface> clock,
    std::shared_ptr<VizSink> sink, std::shared_ptr<McapRecorder> mcap_recorder) {
    spdlog::info("Selected {} for Transmitter", config.type);
    if (config.type == "NoopTransmitter") {
        return std::make_shared<NoopTransmitter>();
    } else if (config.type == "PlaybackTransmitter") {
        return std::make_shared<PlaybackTransmitter>(
            config_cast<PlaybackTransmitterConfiguration>(config), std::move(clock));
    } else if (config.type == "OpenTxTransmitter") {
        return std::make_shared<OpenTxTransmitter>(
            config_cast<OpenTxTransmitterConfiguration>(config), std::move(clock));
    } else if (config.type == "Esp32DiagnosticsOpenTxTransmitter") {
        return std::make_shared<Esp32DiagnosticsOpenTxTransmitter>(
            config_cast<Esp32DiagnosticsOpenTxTransmitterConfiguration>(config), std::move(clock),
            std::move(sink), std::move(mcap_recorder));
    } else if (config.type == "SimTransmitter") {
        return std::make_shared<SimTransmitter>(config_cast<SimTransmitterConfiguration>(config),
                                                std::move(clock));
    } else if (config.type == "EspNowTransmitter") {
        return std::make_shared<EspNowTransmitter>(
            config_cast<EspNowTransmitterConfiguration>(config), std::move(sink),
            std::move(mcap_recorder));
    } else if (config.type == "CompositeTransmitter") {
        const auto &composite = config_cast<CompositeTransmitterConfiguration>(config);
        std::vector<std::shared_ptr<TransmitterInterface>> children;
        children.reserve(composite.children.size());
        for (const auto &child_config : composite.children) {
            // Copies, not moves: every child shares the same clock and sinks.
            children.push_back(make_transmitter(*child_config, clock, sink, mcap_recorder));
        }
        return std::make_shared<CompositeTransmitter>(std::move(children),
                                                      composite.transmit_rate_hz);
    }
    throw std::invalid_argument("Failed to load Transmitter of type " + config.type);
}
}  // namespace auto_battlebot
