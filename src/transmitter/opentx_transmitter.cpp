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

OpenTxTransmitter::OpenTxTransmitter(const OpenTxTransmitterConfiguration& config)
    : config_(config),
      logger_(DiagnosticsLogger::get_logger("opentx_transmitter")),
      processor_(
          {
              .velocity_saturation_limit = config.velocity_saturation_limit,
              .zero_deadzone_percent = config.zero_deadzone_percent,
              .lifted_deadzone_percent = config.lifted_deadzone_percent,
              .reverse_linear = config.reverse_linear_channel,
              .reverse_angular = config.reverse_angular_channel,
          },
          logger_) {}

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

    if (!latest_channels_) return {};

    constexpr double kChannelScale = 1.0 / kChannelMax;
    const double max_linear_mps = config_.max_motor_rpm * M_PI * config_.wheel_diameter / 60.0;
    const double max_angular_radps = 2.0 * max_linear_mps / config_.wheel_track_width;

    CommandFeedback feedback;
    feedback.commands[FrameId::OUR_ROBOT_1] = {
        .linear_x = get_channel_value(config_.linear_channel) * kChannelScale * max_linear_mps,
        .linear_y = 0.0,
        .angular_z = get_channel_value(config_.angular_channel) * kChannelScale * max_angular_radps,
    };
    return feedback;
}

void OpenTxTransmitter::enable() {
    enabled_ = true;
    if (serial_.is_open()) {
        serial_.write("telemetry on\r\n");
        serial_.write("channels on\r\n");
    }
}

void OpenTxTransmitter::disable() {
    enabled_ = false;
    if (serial_.is_open()) {
        serial_.write("telemetry off\r\n");
        serial_.write("channels off\r\n");
    }
}

void OpenTxTransmitter::send(VelocityCommand command) {
    reconnect_if_needed();
    if (!serial_.is_open() || !enabled_) return;

    // Differential control mode: linear_channel carries forward velocity, angular_channel
    // carries yaw rate. The OpenTX-side mixer combines them into per-wheel motor outputs.
    const auto p = processor_.process(command);
    const int linear_value = to_trainer_value(p.linear);
    const int angular_value = to_trainer_value(p.angular);

    logger_->debug("send", {{"linear_channel", config_.linear_channel},
                            {"angular_channel", config_.angular_channel},
                            {"linear_channel_val", linear_value},
                            {"angular_channel_val", angular_value}});

    write_trainer_channels(linear_value, angular_value);
}

void OpenTxTransmitter::write_trainer_channels(int linear_value, int angular_value) {
    const bool linear_ok = serial_.write("trainer " + std::to_string(config_.linear_channel) + " " +
                                         std::to_string(linear_value) + "\r\n");
    const bool angular_ok = serial_.write("trainer " + std::to_string(config_.angular_channel) +
                                          " " + std::to_string(angular_value) + "\r\n");

    if (!linear_ok || !angular_ok) {
        logger_->warning("write_failed_reconnecting",
                         {{"linear_write_ok", linear_ok}, {"angular_write_ok", angular_ok}});
        serial_.close();
        next_reconnect_attempt_ = std::chrono::steady_clock::now();
    }
}

int OpenTxTransmitter::get_channel_value(int channel_idx) const {
    if (!latest_channels_ || channel_idx < 0 || channel_idx >= kMaxChannels) return 0;
    return std::clamp(static_cast<int>((*latest_channels_)[channel_idx]), -kChannelMax,
                      kChannelMax);
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
