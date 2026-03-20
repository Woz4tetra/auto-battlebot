#pragma once

#include "config/config_cast.hpp"
#include "config/config_factory.hpp"
#include "config/config_parser.hpp"
#include "data_structures.hpp"
#include "transmitter/transmitter_interface.hpp"

namespace auto_battlebot {
struct TransmitterConfiguration {
    std::string type;
    virtual ~TransmitterConfiguration() = default;
    virtual void parse_fields([[maybe_unused]] ConfigParser &parser) {}
};

struct NoopTransmitterConfiguration : public TransmitterConfiguration {
    NoopTransmitterConfiguration() { type = "NoopTransmitter"; }

    PARSE_CONFIG_FIELDS(
        // No additional fields
    )
};

struct PlaybackTransmitterConfiguration : public TransmitterConfiguration {
    double init_delay_seconds = 0.0;

    PlaybackTransmitterConfiguration() { type = "PlaybackTransmitter"; }

    PARSE_CONFIG_FIELDS(PARSE_FIELD_DOUBLE(init_delay_seconds))
};

struct OpenTxTransmitterConfiguration : public TransmitterConfiguration {
    int init_button_channel = 5;      // RC channel index used as the init button
    int init_button_threshold = 500;  // channel value above which = button pressed
    int linear_channel = 3;
    int angular_channel = 0;

    OpenTxTransmitterConfiguration() { type = "OpenTxTransmitter"; }

    // clang-format off
    PARSE_CONFIG_FIELDS(
        PARSE_FIELD(init_button_channel)
        PARSE_FIELD(init_button_threshold)
        PARSE_FIELD(linear_channel)
        PARSE_FIELD(angular_channel)
    )
    // clang-format on
};

struct SimTransmitterConfiguration : public TransmitterConfiguration {
    double init_delay_seconds = 0.5;
    /** Artificial command delay in milliseconds (0 = no delay). Used for lag experiments. */
    double command_delay_ms = 0.0;

    SimTransmitterConfiguration() { type = "SimTransmitter"; }

    // clang-format off
    PARSE_CONFIG_FIELDS(
        PARSE_FIELD_DOUBLE(init_delay_seconds)
        PARSE_FIELD_DOUBLE(command_delay_ms)
    )
    // clang-format on
};

std::shared_ptr<TransmitterInterface> make_transmitter(const TransmitterConfiguration &config);
std::unique_ptr<TransmitterConfiguration> parse_transmitter_config(ConfigParser &parser);
std::unique_ptr<TransmitterConfiguration> load_transmitter_from_toml(
    toml::table const &toml_data, std::vector<std::string> &parsed_sections);
}  // namespace auto_battlebot
