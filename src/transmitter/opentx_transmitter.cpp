#include "transmitter/opentx_transmitter.hpp"

#include <algorithm>
#include <chrono>
#include <string>

namespace auto_battlebot {
static bool trainer_mixed_in(int trainer_raw) { return trainer_raw >= 0; }

namespace {
constexpr auto kReconnectInterval = std::chrono::seconds(1);
constexpr auto kChannelStreamStallTimeout = std::chrono::milliseconds(500);
constexpr int kOpenTxRawChannelMax = 1000;
constexpr int kOpenTxTrainerOutputMax = 500;
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
    if (config_.max_motor_rpm_per_sec > 0.0 && config_.max_motor_rpm > 0.0) {
        max_normalized_linear_rate_per_s_ = config_.max_motor_rpm_per_sec / config_.max_motor_rpm;
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
    // Give the radio a grace window to start streaming before the channel stream reads as
    // stalled, so a healthy open does not flash a trainer-mode fault on the UI.
    last_channel_time_ = std::chrono::steady_clock::now();
    return true;
}

CommandFeedback OpenTxTransmitter::update() {
    reconnect_if_needed();
    if (!serial_.is_open()) return {};

    auto bytes = serial_.read_available();
    if (bytes.empty()) return {};

    auto crsf_results = crsf_parser_.parse(bytes);
    for (const auto& [packet, error] : crsf_results) {
        if (!packet) continue;

        std::visit([&](const auto& pkt) { handle_packet(pkt); }, *packet);
    }

    process_channel_updates(bytes);
    log_switch_states();

    if (!latest_channels_) return {};

    constexpr double kChannelScale = 1.0 / kOpenTxRawChannelMax;
    const double angular_readback_sign = config_.invert_angular_channel_readback ? -1.0 : 1.0;

    const auto body = processor_->to_body_velocity({
        .channel_a = get_channel_value(config_.linear_channel) * kChannelScale,
        .channel_b =
            get_channel_value(config_.angular_channel) * kChannelScale * angular_readback_sign,
    });

    CommandFeedback feedback;
    feedback.stick_commands[FrameId::OUR_ROBOT_1] = {
        .linear_x = body.linear,
        .linear_y = 0.0,
        .angular_z = body.angular,
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
    if (max_normalized_linear_rate_per_s_ <= 0.0) return linear_command;

    const double now = clock_->now();

    if (!trainer_mixed_in(get_channel_value(config_.trainer_enable_channel))) {
        prev_linear_command_ = 0.0;
        last_send_time_ = now;
        return linear_command;
    }

    if (!last_send_time_) {
        last_send_time_ = now;
        prev_linear_command_ = linear_command;
        return linear_command;
    }

    const double dt = now - *last_send_time_;
    last_send_time_ = now;
    if (dt <= 0.0) return linear_command;

    const double max_delta = max_normalized_linear_rate_per_s_ * dt;
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
    return std::clamp(static_cast<int>((*latest_channels_)[channel_idx]), -kOpenTxRawChannelMax,
                      kOpenTxRawChannelMax);
}

void OpenTxTransmitter::log_switch_states() {
    if (!latest_channels_) {
        logger_->info("switch_states", {{"has_switch_states", false}});
        return;
    }
    const int trainer_raw = get_channel_value(config_.trainer_enable_channel);
    const int init_raw = get_channel_value(config_.init_button_channel);
    const bool autonomy_enabled = trainer_mixed_in(trainer_raw);
    const bool init_pressed = init_raw > config_.init_button_threshold;
    const auto states = std::make_tuple(autonomy_enabled, init_pressed);
    if (last_logged_switch_states_ == states) return;
    last_logged_switch_states_ = states;
    logger_->info("switch_states", {{"has_switch_states", true},
                                    {"autonomy_enabled", static_cast<int>(autonomy_enabled)},
                                    {"autonomy_raw", trainer_raw},
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
    int value = static_cast<int>(normalized * kOpenTxTrainerOutputMax);
    return std::clamp(value, -kOpenTxTrainerOutputMax, kOpenTxTrainerOutputMax);
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

    latest_channels_ = channel_updates.back();
    last_channel_time_ = std::chrono::steady_clock::now();

    // Every frame, not only on change, so the section's age is the age of the last frame read.
    std::vector<int> channel_values(latest_channels_->begin(), latest_channels_->end());
    logger_->debug("channels", {{"values", channel_values}});
}

bool OpenTxTransmitter::channels_fresh() const {
    return (std::chrono::steady_clock::now() - last_channel_time_) < kChannelStreamStallTimeout;
}

TransmitterStatus OpenTxTransmitter::get_status() const {
    const bool connected = serial_.is_open();
    const bool receiving = connected && channels_fresh();
    return {
        .connected = connected,
        .receiving_channels = receiving,
        .has_autonomy_switch = true,
        .autonomy_switch_on = receiving && latest_channels_.has_value() &&
                              trainer_mixed_in(get_channel_value(config_.trainer_enable_channel))};
}

bool OpenTxTransmitter::reconnect_if_needed() {
    const auto now = std::chrono::steady_clock::now();

    if (serial_.is_open()) {
        if (channels_fresh()) return true;
        // Port still open but the radio has gone quiet: trainer mode dropped out, or the
        // `channels on` stream never started. Re-priming it needs a fresh open, so drop the
        // port and fall through to the reconnect path below. Rate-limited by
        // next_reconnect_attempt_ so a radio that never streams retries once per interval
        // instead of thrashing the port every tick.
        if (now < next_reconnect_attempt_) return true;
        const auto stalled_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - last_channel_time_).count();
        logger_->warning("channels_stalled_reconnecting",
                         {{"stalled_ms", static_cast<int>(stalled_ms)}});
        serial_.close();
    }

    if (now < next_reconnect_attempt_) return false;

    next_reconnect_attempt_ = now + kReconnectInterval;
    return initialize();
}

}  // namespace auto_battlebot
