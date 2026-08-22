#include "transmitter/opentx_transmitter.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>

namespace auto_battlebot {
namespace {
constexpr auto kReconnectInterval = std::chrono::seconds(1);
constexpr int kChannelMax = 1000;  // raw RC channel range [-1000, 1000]
constexpr int kTrainerMax = 500;   // OpenTX trainer output range [-500, 500]
}  // namespace

OpenTxTransmitter::OpenTxTransmitter(const OpenTxTransmitterConfiguration& config,
                                     std::shared_ptr<ClockInterface> clock)
    : config_(config),
      logger_(DiagnosticsLogger::get_logger("opentx_transmitter")),
      clock_(std::move(clock)),
      processor_(
          make_drive_processor(config.drive_processor,
                               {
                                   .velocity_saturation_limit = config.velocity_saturation_limit,
                                   .zero_deadzone_percent = config.zero_deadzone_percent,
                                   .lifted_deadzone_percent = config.lifted_deadzone_percent,
                                   .reverse_linear = config.reverse_linear_channel,
                                   .reverse_angular = config.reverse_angular_channel,
                               },
                               logger_)) {
    // The command reaching send() is normalized ([-1, 1] == +/-max_motor_rpm), so an RPM/s limit
    // becomes a normalized rate limit by dividing out max_motor_rpm. wheel_diameter is not needed
    // here; it only enters the RPM/s default derived from the m/s^2 slip threshold.
    if (config_.max_motor_rpm_per_sec > 0.0 && config_.max_motor_rpm > 0.0) {
        max_linear_rate_per_sec_ = config_.max_motor_rpm_per_sec / config_.max_motor_rpm;
    }
}

bool OpenTxTransmitter::initialize() {
    auto device = find_opentx_device();
    if (!device) {
        logger_->warning(
            "device_not_found",
            {{"message", "No OpenTX USB serial device found (VID=0x0483, PID=0x5740)"}});
        next_reconnect_attempt_ = std::chrono::steady_clock::now() + kReconnectInterval;
        return false;
    }

    if (!serial_.open(*device)) {
        logger_->warning("open_failed", {{"path", *device}});
        next_reconnect_attempt_ = std::chrono::steady_clock::now() + kReconnectInterval;
        return false;
    }

    logger_->info("initialized", {{"device", *device}});
    next_reconnect_attempt_ = std::chrono::steady_clock::now() + kReconnectInterval;

    // Re-prime the OpenTX-side streams every time the port (re)opens. Without this, a
    // reconnect after the device was unplugged or not yet present at startup leaves the
    // radio silent on the receive side until enable() happens to be called while the port
    // is open.
    serial_.write("telemetry on\r\n");
    serial_.write("channels on\r\n");
    return true;
}

CommandFeedback OpenTxTransmitter::update() {
    reconnect_if_needed();
    if (!serial_.is_open()) return {};

    auto bytes = serial_.read_available();
    if (bytes.empty()) return {};

    // Parse CRSF telemetry frames
    auto crsf_results = crsf_parser_.parse(bytes);
    for (const auto& [packet, error] : crsf_results) {
        if (!packet) continue;

        std::visit([&](const auto& pkt) { handle_packet(pkt); }, *packet);
    }

    process_channel_updates(bytes);
    log_switch_states();

    if (!latest_channels_) return {};

    constexpr double kChannelScale = 1.0 / kChannelMax;
    const double max_linear_mps = config_.max_motor_rpm * M_PI * config_.wheel_diameter / 60.0;
    const double max_angular_radps = 2.0 * max_linear_mps / config_.wheel_track_width;

    // The channels on the wire are in whatever space the processor emits, so convert them back to
    // body axes before scaling to physical units.
    const auto body = processor_->to_body_velocity({
        .channel_a = get_channel_value(config_.linear_channel) * kChannelScale,
        .channel_b = get_channel_value(config_.angular_channel) * kChannelScale,
    });

    CommandFeedback feedback;
    feedback.commands[FrameId::OUR_ROBOT_1] = {
        .linear_x = body.linear * max_linear_mps,
        .linear_y = 0.0,
        .angular_z = body.angular * max_angular_radps,
    };
    return feedback;
}

// enable()/disable() only gate trainer command output. Telemetry and channel streams stay on
// whenever the serial port is open so that link statistics and the init button on the radio
// remain functional even when autonomy is disabled.
void OpenTxTransmitter::enable() { enabled_ = true; }

void OpenTxTransmitter::disable() {
    enabled_ = false;
    // Drop slew-limiter state so re-enabling ramps up from a standstill rather than resuming from
    // whatever command was last sent before autonomy was cut.
    prev_linear_command_ = 0.0;
    last_send_time_.reset();
}

void OpenTxTransmitter::send(VelocityCommand command) {
    reconnect_if_needed();
    if (!serial_.is_open() || !enabled_) return;

    command.linear_x = limit_linear_acceleration(command.linear_x);

    // What the two channels carry depends on the configured processor: left and right motor
    // commands under tank drive, forward velocity and yaw rate under differential drive (where the
    // OpenTX-side mixer combines them into per-wheel motor outputs).
    const auto channels = processor_->process(command);
    const int channel_a_value = to_trainer_value(channels.channel_a);
    const int channel_b_value = to_trainer_value(channels.channel_b);

    logger_->debug("send", {{"channel_a", config_.linear_channel},
                            {"channel_b", config_.angular_channel},
                            {"channel_a_val", channel_a_value},
                            {"channel_b_val", channel_b_value}});

    write_trainer_channels(channel_a_value, channel_b_value);
}

double OpenTxTransmitter::limit_linear_acceleration(double linear_command) {
    if (max_linear_rate_per_sec_ <= 0.0) return linear_command;

    const double now = clock_->now();

    // Only compute slew rate when auto switch is active
    if (get_channel_value(config_.trainer_enable_channel) < 0) {
        prev_linear_command_ = 0.0;
        last_send_time_ = now;
        return linear_command;
    }

    if (!last_send_time_) {
        // First tick after (re)enable: no dt yet, so establish the baseline and pass through.
        last_send_time_ = now;
        prev_linear_command_ = linear_command;
        return linear_command;
    }

    const double dt = now - *last_send_time_;
    last_send_time_ = now;
    if (dt <= 0.0) return linear_command;  // Non-advancing clock: nothing to rate-limit against.

    const double max_delta = max_linear_rate_per_sec_ * dt;
    const double limited = std::clamp(linear_command, prev_linear_command_ - max_delta,
                                      prev_linear_command_ + max_delta);
    if (limited != linear_command) {
        logger_->debug("accel_limited", {{"requested", linear_command},
                                         {"limited", limited},
                                         {"prev", prev_linear_command_},
                                         {"dt", dt}});
    }
    prev_linear_command_ = limited;
    return limited;
}

void OpenTxTransmitter::write_trainer_channels(int channel_a_value, int channel_b_value) {
    const bool channel_a_ok = serial_.write("trainer " + std::to_string(config_.linear_channel) +
                                            " " + std::to_string(channel_a_value) + "\r\n");
    const bool channel_b_ok = serial_.write("trainer " + std::to_string(config_.angular_channel) +
                                            " " + std::to_string(channel_b_value) + "\r\n");

    if (!channel_a_ok || !channel_b_ok) {
        logger_->warning("write_failed_reconnecting", {{"channel_a_write_ok", channel_a_ok},
                                                       {"channel_b_write_ok", channel_b_ok}});
        serial_.close();
        next_reconnect_attempt_ = std::chrono::steady_clock::now();
    }
}

int OpenTxTransmitter::get_channel_value(int channel_idx) const {
    if (!latest_channels_ || channel_idx < 0 || channel_idx >= kMaxChannels) return 0;
    return std::clamp(static_cast<int>((*latest_channels_)[channel_idx]), -kChannelMax,
                      kChannelMax);
}

BehaviorMode OpenTxTransmitter::behavior_mode() const {
    return get_channel_value(config_.behavior_mode_channel) > config_.behavior_mode_threshold
               ? BehaviorMode::RUN_AWAY
               : BehaviorMode::ATTACK;
}

void OpenTxTransmitter::log_switch_states() {
    if (!latest_channels_) return;
    const int mode_raw = get_channel_value(config_.behavior_mode_channel);
    const int trainer_raw = get_channel_value(config_.trainer_enable_channel);
    const int init_raw = get_channel_value(config_.init_button_channel);
    const bool run_away = mode_raw > config_.behavior_mode_threshold;
    // Trainer enable is active while the channel reads negative, matching the comparison in
    // limit_linear_acceleration(). Do not normalize the sign; it is a property of the OpenTX
    // model.
    const bool trainer_enabled = trainer_raw < 0;
    const bool init_pressed = init_raw > config_.init_button_threshold;
    const auto states = std::make_tuple(run_away, trainer_enabled, init_pressed);
    if (last_logged_switch_states_ == states) return;
    last_logged_switch_states_ = states;
    // The raw value next to each flag separates "switch on the wrong channel" from "threshold
    // set wrong": both look like a flag that never changes. These three flags decide whether
    // the robot moves at all, so they log at info and survive the default log level.
    logger_->info("switch_states", {{"behavior_mode", run_away ? "run_away" : "attack"},
                                    {"behavior_mode_raw", mode_raw},
                                    {"trainer_enabled", static_cast<int>(trainer_enabled)},
                                    {"trainer_raw", trainer_raw},
                                    {"init_button", static_cast<int>(init_pressed)},
                                    {"init_button_raw", init_raw}});
}

bool OpenTxTransmitter::did_init_button_press() {
    if (!latest_channels_) return false;

    int channel_idx = config_.init_button_channel;
    if (channel_idx < 0 || channel_idx >= kMaxChannels) return false;

    bool is_pressed = (*latest_channels_)[channel_idx] > config_.init_button_threshold;

    if (is_pressed && !init_button_was_pressed_) {
        init_button_was_pressed_ = true;
        return true;
    }
    if (!is_pressed) {
        init_button_was_pressed_ = false;
    }
    return false;
}

int OpenTxTransmitter::to_trainer_value(double normalized) {
    int value = static_cast<int>(normalized * kTrainerMax);
    return std::clamp(value, -kTrainerMax, kTrainerMax);
}

void OpenTxTransmitter::handle_packet(const CrsfLinkStatistics& pkt) {
    logger_->debug("link_statistics", {{"up_lq", pkt.up_link_quality},
                                       {"up_rssi", -static_cast<int>(pkt.up_rssi_ant1)},
                                       {"down_lq", pkt.down_link_quality},
                                       {"down_rssi", -static_cast<int>(pkt.down_rssi)}});
}

void OpenTxTransmitter::handle_packet(const CrsfBattery& pkt) {
    logger_->debug("battery", {{"voltage", pkt.voltage}, {"current", pkt.current}});
}

void OpenTxTransmitter::handle_packet(const CrsfAttitude& pkt) {
    logger_->debug("attitude", {{"roll", pkt.roll}, {"pitch", pkt.pitch}, {"yaw", pkt.yaw}});
}

void OpenTxTransmitter::handle_packet(const CrsfFlightMode& pkt) {
    logger_->debug("flight_mode", {{"mode", pkt.flight_mode}});
}

void OpenTxTransmitter::process_channel_updates(const std::vector<uint8_t>& bytes) {
    auto channel_updates = channels_parser_.process(bytes);
    if (channel_updates.empty()) return;

    const auto& newest_channels = channel_updates.back();
    const bool channels_changed =
        !latest_channels_.has_value() || newest_channels != latest_channels_.value();
    latest_channels_ = newest_channels;

    if (channels_changed) {
        std::vector<int> channel_values(latest_channels_->begin(), latest_channels_->end());
        logger_->debug("channels", {{"values", channel_values}});
    }
}

bool OpenTxTransmitter::reconnect_if_needed() {
    if (serial_.is_open()) return true;

    auto now = std::chrono::steady_clock::now();
    if (now < next_reconnect_attempt_) return false;

    next_reconnect_attempt_ = now + kReconnectInterval;
    return initialize();
}

}  // namespace auto_battlebot
