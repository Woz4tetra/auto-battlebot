#pragma once

#include <arpa/inet.h>

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "config/config_factory.hpp"
#include "config/config_parser.hpp"
#include "mcap_recorder/mcap_recorder.hpp"
#include "time/clock_interface.hpp"
#include "transmitter/transmitter_interface.hpp"
#include "viz/viz_sink.hpp"

namespace auto_battlebot {
struct TransmitterConfiguration {
    std::string type;
    virtual ~TransmitterConfiguration() = default;
    virtual void parse_fields([[maybe_unused]] ConfigParser &parser) {}
};

struct NoopTransmitterConfiguration : public TransmitterConfiguration {
    NoopTransmitterConfiguration() { type = "NoopTransmitter"; }

    PARSE_CONFIG_FIELDS()
};

struct PlaybackTransmitterConfiguration : public TransmitterConfiguration {
    double init_delay_seconds = 0.0;
    double velocity_saturation_limit = 1.0;
    double zero_deadzone_percent = 0.0;
    double lifted_deadzone_percent = 0.0;
    bool reverse_linear_channel = false;
    bool reverse_angular_channel = false;

    PlaybackTransmitterConfiguration() { type = "PlaybackTransmitter"; }

    // clang-format off
    PARSE_CONFIG_FIELDS(
        PARSE_FIELD_DOUBLE(init_delay_seconds)
        PARSE_FIELD_DOUBLE(velocity_saturation_limit)
        PARSE_FIELD_DOUBLE(zero_deadzone_percent)
        PARSE_FIELD_DOUBLE(lifted_deadzone_percent)
        PARSE_FIELD_BOOL(reverse_linear_channel)
        PARSE_FIELD_BOOL(reverse_angular_channel)
    )
    // clang-format on
};

struct OpenTxTransmitterConfiguration : public TransmitterConfiguration {
    std::optional<int64_t> init_button_channel;
    int init_button_threshold = 500;
    std::string drive_processor = "TankDriveProcessor";
    int linear_channel = 0;
    int angular_channel = 1;
    int trainer_enable_channel = 5;
    bool reverse_linear_channel = false;
    bool reverse_angular_channel = false;
    bool invert_angular_channel_readback = false;
    double lifted_deadzone_percent = 0.0;
    double zero_deadzone_percent = 0.0;
    double max_motor_rpm = 1500.0;
    std::optional<double> max_motor_rpm_per_s;

    double velocity_saturation_limit = 1.0;

    OpenTxTransmitterConfiguration() { type = "OpenTxTransmitter"; }

    void parse_fields(ConfigParser &parser) override {
        init_button_channel = parser.get_nullable_int("init_button_channel", init_button_channel);
        init_button_threshold = static_cast<int>(
            parser.get_optional_int("init_button_threshold", init_button_threshold));
        drive_processor = parser.get_optional_string("drive_processor", drive_processor);
        linear_channel =
            static_cast<int>(parser.get_optional_int("linear_channel", linear_channel));
        angular_channel =
            static_cast<int>(parser.get_optional_int("angular_channel", angular_channel));
        trainer_enable_channel = static_cast<int>(
            parser.get_optional_int("trainer_enable_channel", trainer_enable_channel));
        reverse_linear_channel =
            parser.get_optional_bool("reverse_linear_channel", reverse_linear_channel);
        reverse_angular_channel =
            parser.get_optional_bool("reverse_angular_channel", reverse_angular_channel);
        invert_angular_channel_readback = parser.get_optional_bool(
            "invert_angular_channel_readback", invert_angular_channel_readback);
        lifted_deadzone_percent =
            parser.get_optional_double("lifted_deadzone_percent", lifted_deadzone_percent);
        zero_deadzone_percent =
            parser.get_optional_double("zero_deadzone_percent", zero_deadzone_percent);
        max_motor_rpm = parser.get_optional_double("max_motor_rpm", max_motor_rpm);
        parser.reject_renamed("max_motor_rpm_per_sec", "max_motor_rpm_per_s");
        max_motor_rpm_per_s =
            parser.get_nullable_double("max_motor_rpm_per_s", max_motor_rpm_per_s);
        velocity_saturation_limit =
            parser.get_optional_double("velocity_saturation_limit", velocity_saturation_limit);
    }
};

struct Esp32DiagnosticsOpenTxTransmitterConfiguration : public OpenTxTransmitterConfiguration {
    std::string esp32_host = "192.168.4.1";
    int esp32_port = 80;
    double reconnect_period_s = 1.0;

    Esp32DiagnosticsOpenTxTransmitterConfiguration() { type = "Esp32DiagnosticsOpenTxTransmitter"; }

    void parse_fields(ConfigParser &parser) override {
        OpenTxTransmitterConfiguration::parse_fields(parser);
        esp32_host = parser.get_optional_string("esp32_host", esp32_host);
        esp32_port = static_cast<int>(parser.get_optional_int("esp32_port", esp32_port));
        reconnect_period_s = parser.get_optional_double("reconnect_period_s", reconnect_period_s);
        in_addr address{};
        if (inet_pton(AF_INET, esp32_host.c_str(), &address) != 1) {
            throw ConfigValidationError(
                "esp32_host must be an IPv4 address literal in section [transmitter]");
        }
        if (esp32_port <= 0 || esp32_port > 65535) {
            throw ConfigValidationError("esp32_port must be in 1..65535 in section [transmitter]");
        }
        if (reconnect_period_s <= 0.0) {
            throw ConfigValidationError("reconnect_period_s must be > 0 in section [transmitter]");
        }
    }
};

struct EspNowTransmitterConfiguration : public TransmitterConfiguration {
    std::string dongle_serial_path = "/dev/espnow_dongle";

    std::string drive_processor = "DifferentialDriveProcessor";

    double velocity_saturation_limit = 1.0;
    double zero_deadzone_percent = 0.0;
    double lifted_deadzone_percent = 0.0;
    bool reverse_linear_channel = false;
    bool reverse_angular_channel = false;

    double max_motor_rpm = 1500.0;
    std::optional<double> max_motor_rpm_per_s;

    EspNowTransmitterConfiguration() { type = "EspNowTransmitter"; }

    void parse_fields(ConfigParser &parser) override {
        dongle_serial_path = parser.get_optional_string("dongle_serial_path", dongle_serial_path);
        drive_processor = parser.get_optional_string("drive_processor", drive_processor);
        velocity_saturation_limit =
            parser.get_optional_double("velocity_saturation_limit", velocity_saturation_limit);
        zero_deadzone_percent =
            parser.get_optional_double("zero_deadzone_percent", zero_deadzone_percent);
        lifted_deadzone_percent =
            parser.get_optional_double("lifted_deadzone_percent", lifted_deadzone_percent);
        reverse_linear_channel =
            parser.get_optional_bool("reverse_linear_channel", reverse_linear_channel);
        reverse_angular_channel =
            parser.get_optional_bool("reverse_angular_channel", reverse_angular_channel);
        max_motor_rpm = parser.get_optional_double("max_motor_rpm", max_motor_rpm);
        parser.reject_renamed("max_motor_rpm_per_sec", "max_motor_rpm_per_s");
        max_motor_rpm_per_s =
            parser.get_nullable_double("max_motor_rpm_per_s", max_motor_rpm_per_s);
    }
};

struct CompositeTransmitterConfiguration : public TransmitterConfiguration {
    std::vector<std::unique_ptr<TransmitterConfiguration>> children;
    std::vector<double> transmit_rate_hz;

    CompositeTransmitterConfiguration() { type = "CompositeTransmitter"; }

    void parse_fields(ConfigParser &parser) override;
};

struct SimTransmitterConfiguration : public TransmitterConfiguration {
    double init_delay_seconds = 0.5;
    double command_delay_ms = 0.0;
    double velocity_saturation_limit = 1.0;

    SimTransmitterConfiguration() { type = "SimTransmitter"; }

    // clang-format off
    PARSE_CONFIG_FIELDS(
        PARSE_FIELD_DOUBLE(init_delay_seconds)
        PARSE_FIELD_DOUBLE(command_delay_ms)
        PARSE_FIELD_DOUBLE(velocity_saturation_limit)
    )
    // clang-format on
};

std::shared_ptr<TransmitterInterface> make_transmitter(const TransmitterConfiguration &config,
                                                       std::shared_ptr<ClockInterface> clock,
                                                       std::shared_ptr<VizSink> sink,
                                                       std::shared_ptr<McapRecorder> mcap_recorder);
std::unique_ptr<TransmitterConfiguration> parse_transmitter_config(ConfigParser &parser);
std::unique_ptr<TransmitterConfiguration> load_transmitter_from_toml(
    toml::table const &toml_data, std::vector<std::string> &parsed_sections);
}  // namespace auto_battlebot
