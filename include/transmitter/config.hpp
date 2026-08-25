#pragma once

#include "config/config_factory.hpp"
#include "config/config_parser.hpp"
#include "enums/behavior_mode.hpp"
#include "time/clock_interface.hpp"
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
    double velocity_saturation_limit = 1.0;
    double zero_deadzone_percent = 0.0;
    double lifted_deadzone_percent = 0.0;
    bool reverse_linear_channel = false;
    bool reverse_angular_channel = false;
    /** Behavior mode for the run. Playback has no radio, so this stands in for the
     *  physical switch OpenTxTransmitter reads on behavior_mode_channel. */
    BehaviorMode behavior_mode = BehaviorMode::ATTACK;

    PlaybackTransmitterConfiguration() { type = "PlaybackTransmitter"; }

    // clang-format off
    PARSE_CONFIG_FIELDS(
        PARSE_FIELD_DOUBLE(init_delay_seconds)
        PARSE_FIELD_DOUBLE(velocity_saturation_limit)
        PARSE_FIELD_DOUBLE(zero_deadzone_percent)
        PARSE_FIELD_DOUBLE(lifted_deadzone_percent)
        PARSE_FIELD_BOOL(reverse_linear_channel)
        PARSE_FIELD_BOOL(reverse_angular_channel)
        PARSE_ENUM(behavior_mode, BehaviorMode)
    )
    // clang-format on
};

struct OpenTxTransmitterConfiguration : public TransmitterConfiguration {
    /** Init button disabled. Neither the mrs_buff_mk3 transmitter nor mr_stabs_mk2
     *  maps a button for it; field re-initialization is a touchscreen action. 31 is
     *  the last array slot with nothing mapped there, so the threshold never trips. */
    int init_button_channel = 31;
    int init_button_threshold = 500;  // channel value above which = button pressed
    /** Mixer that turns the body-frame command into the two output channels:
     *  "TankDriveProcessor" sends left and right motor commands directly,
     *  "DifferentialDriveProcessor" sends linear and angular for the radio-side mixer. */
    std::string drive_processor = "TankDriveProcessor";
    /** RC channel carrying the first mixer output: left wheel under tank drive, linear under
     *  differential drive. */
    int linear_channel = 0;
    /** RC channel carrying the second mixer output: right wheel under tank drive, angular under
     *  differential drive. */
    int angular_channel = 1;
    /** RC channel carrying the trainer enable switch (SG). Autonomous motion only reaches the
     *  wheels while this switch is engaged, at which point the channel reads negative. The linear
     *  acceleration limiter is held at standstill until then. Requires SG to be mapped to this
     *  output channel (radio CH6) in the OpenTX model. */
    int trainer_enable_channel = 5;
    /** RC channel carrying the behavior mode switch (radio CH7). Reads above
     *  behavior_mode_threshold to select RUN_AWAY, otherwise ATTACK. */
    int behavior_mode_channel = 6;
    int behavior_mode_threshold = 500;
    bool reverse_linear_channel = false;
    bool reverse_angular_channel = false;
    /** Minimum non-zero output magnitude (%) after zero deadzone is exceeded. */
    double lifted_deadzone_percent = 0.0;
    /** Input magnitude (%) below which output is forced to zero. */
    double zero_deadzone_percent = 0.0;
    double max_motor_rpm = 1500.0;  // Max output shaft RPM
    /** Max output-shaft angular acceleration (RPM/s). Slew-limits the linear command so the wheel
     *  surface acceleration stays below the slip/flip threshold. Derivation from the measured
     *  8.5 m/s^2 limit: dRPM/dt = a * 60 / (pi * wheel_diameter) = 8.5 * 60 / (pi * 0.05)
     *  ~= 3247 RPM/s. 0 = disabled (no acceleration limit). */
    double max_motor_rpm_per_sec = 3247.0;

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
        drive_processor = parser.get_optional_string("drive_processor", drive_processor);
        linear_channel =
            static_cast<int>(parser.get_optional_int("linear_channel", linear_channel));
        angular_channel =
            static_cast<int>(parser.get_optional_int("angular_channel", angular_channel));
        trainer_enable_channel = static_cast<int>(
            parser.get_optional_int("trainer_enable_channel", trainer_enable_channel));
        behavior_mode_channel = static_cast<int>(
            parser.get_optional_int("behavior_mode_channel", behavior_mode_channel));
        behavior_mode_threshold = static_cast<int>(
            parser.get_optional_int("behavior_mode_threshold", behavior_mode_threshold));
        reverse_linear_channel =
            parser.get_optional_bool("reverse_linear_channel", reverse_linear_channel);
        reverse_angular_channel =
            parser.get_optional_bool("reverse_angular_channel", reverse_angular_channel);
        lifted_deadzone_percent =
            parser.get_optional_double("lifted_deadzone_percent", lifted_deadzone_percent);
        zero_deadzone_percent =
            parser.get_optional_double("zero_deadzone_percent", zero_deadzone_percent);
        max_motor_rpm = parser.get_optional_double("max_motor_rpm", max_motor_rpm);
        max_motor_rpm_per_sec =
            parser.get_optional_double("max_motor_rpm_per_sec", max_motor_rpm_per_sec);
        velocity_saturation_limit =
            parser.get_optional_double("velocity_saturation_limit", velocity_saturation_limit);
    }
};

struct SimTransmitterConfiguration : public TransmitterConfiguration {
    double init_delay_seconds = 0.5;
    /** Artificial command delay in milliseconds (0 = no delay). Used for lag experiments. */
    double command_delay_ms = 0.0;
    /** Same meaning as the drive processors' field: angular gets priority and linear is clipped to
     * (limit - |angular|). The sim has no drive processor, so without this the simulated robot
     * keeps forward authority during a turn that the real one has already lost in the mixer.
     * 0 = disabled. */
    double velocity_saturation_limit = 1.0;
    /** Behavior mode the simulated driver holds for the whole run. The real transmitter reads
     * this off a switch; the sim has no switch, so a sweep picks the mission here. */
    BehaviorMode behavior_mode = BehaviorMode::ATTACK;

    SimTransmitterConfiguration() { type = "SimTransmitter"; }

    // clang-format off
    PARSE_CONFIG_FIELDS(
        PARSE_FIELD_DOUBLE(init_delay_seconds)
        PARSE_FIELD_DOUBLE(command_delay_ms)
        PARSE_FIELD_DOUBLE(velocity_saturation_limit)
        PARSE_ENUM(behavior_mode, BehaviorMode)
    )
    // clang-format on
};

std::shared_ptr<TransmitterInterface> make_transmitter(const TransmitterConfiguration &config,
                                                       std::shared_ptr<ClockInterface> clock);
std::unique_ptr<TransmitterConfiguration> parse_transmitter_config(ConfigParser &parser);
std::unique_ptr<TransmitterConfiguration> load_transmitter_from_toml(
    toml::table const &toml_data, std::vector<std::string> &parsed_sections);
}  // namespace auto_battlebot
