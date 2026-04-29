#pragma once

#include "config/config_factory.hpp"
#include "config/config_parser.hpp"
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
    int linear_channel = 0;
    int angular_channel = 1;
    bool reverse_linear_channel = false;
    bool reverse_angular_channel = false;
    /** Minimum non-zero output magnitude (%) after zero deadzone is exceeded. */
    double lifted_deadzone_percent = 0.0;
    /** Input magnitude (%) below which output is forced to zero. */
    double zero_deadzone_percent = 0.0;
    double wheel_track_width = 1.0;
    double max_motor_rpm = 1500.0;
    double wheel_diameter = 0.05;

    /** Combined output budget: |linear| + |angular| <= limit.
     *  Angular takes priority; linear fills remaining headroom.
     *  0 = disabled (each channel clamped independently to [-1, 1]). */
    double velocity_saturation_limit = 1.0;

    OpenTxTransmitterConfiguration() { type = "OpenTxTransmitter"; }

    void parse_fields(ConfigParser &parser) override {
        init_button_channel =
            static_cast<int>(parser.get_optional_int("init_button_channel", init_button_channel));
        init_button_threshold = static_cast<int>(
            parser.get_optional_int("init_button_threshold", init_button_threshold));
        linear_channel =
            static_cast<int>(parser.get_optional_int("linear_channel", linear_channel));
        angular_channel =
            static_cast<int>(parser.get_optional_int("angular_channel", angular_channel));
        reverse_linear_channel =
            parser.get_optional_bool("reverse_linear_channel", reverse_linear_channel);
        reverse_angular_channel =
            parser.get_optional_bool("reverse_angular_channel", reverse_angular_channel);
        lifted_deadzone_percent =
            parser.get_optional_double("lifted_deadzone_percent", lifted_deadzone_percent);
        zero_deadzone_percent =
            parser.get_optional_double("zero_deadzone_percent", zero_deadzone_percent);
        wheel_track_width = parser.get_optional_double("wheel_track_width", wheel_track_width);
        max_motor_rpm = parser.get_optional_double("max_motor_rpm", max_motor_rpm);
        wheel_diameter = parser.get_optional_double("wheel_diameter", wheel_diameter);
        velocity_saturation_limit =
            parser.get_optional_double("velocity_saturation_limit", velocity_saturation_limit);
    }
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
