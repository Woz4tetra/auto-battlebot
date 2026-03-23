#include "transmitter/opentx_transmitter.hpp"

#include <algorithm>
#include <chrono>
#include <string>

namespace auto_battlebot {
namespace {
constexpr auto kReconnectInterval = std::chrono::seconds(1);
}

OpenTxTransmitter::OpenTxTransmitter(const OpenTxTransmitterConfiguration& config)
    : config_(config) {
    logger_ = DiagnosticsLogger::get_logger("opentx_transmitter");
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

        std::visit(
            [&](const auto& pkt) {
                using T = std::decay_t<decltype(pkt)>;
                if constexpr (std::is_same_v<T, CrsfLinkStatistics>) {
                    logger_->debug("link_statistics",
                                   {{"up_lq", pkt.up_link_quality},
                                    {"up_rssi", -static_cast<int>(pkt.up_rssi_ant1)},
                                    {"down_lq", pkt.down_link_quality},
                                    {"down_rssi", -static_cast<int>(pkt.down_rssi)}});
                } else if constexpr (std::is_same_v<T, CrsfBattery>) {
                    logger_->debug("battery", {{"voltage", pkt.voltage}, {"current", pkt.current}});
                } else if constexpr (std::is_same_v<T, CrsfAttitude>) {
                    logger_->debug("attitude",
                                   {{"roll", pkt.roll}, {"pitch", pkt.pitch}, {"yaw", pkt.yaw}});
                } else if constexpr (std::is_same_v<T, CrsfFlightMode>) {
                    logger_->debug("flight_mode", {{"mode", pkt.flight_mode}});
                }
            },
            *packet);
    }

    // Parse OpenTX channel streaming
    auto channel_updates = channels_parser_.process(bytes);
    if (!channel_updates.empty()) {
        latest_channels_ = channel_updates.back();

        std::vector<int> channel_values(latest_channels_->begin(), latest_channels_->end());
        logger_->debug("channels", {{"values", channel_values}});
    }

    return {};
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
    if (!serial_.is_open()) return;
    if (!enabled_) return;

    int linear_val = to_trainer_value(command.linear_x);
    int angular_val = -1 * to_trainer_value(command.angular_z);

    logger_->debug("send", {{"linear", linear_val}, {"angular", angular_val}});

    bool linear_write_ok = serial_.write("trainer " + std::to_string(config_.linear_channel) + " " +
                                         std::to_string(linear_val) + "\r\n");
    bool angular_write_ok = serial_.write("trainer " + std::to_string(config_.angular_channel) + " " +
                                          std::to_string(angular_val) + "\r\n");

    if (!linear_write_ok || !angular_write_ok) {
        logger_->warning("write_failed_reconnecting",
                         {{"linear_write_ok", linear_write_ok},
                          {"angular_write_ok", angular_write_ok}});
        serial_.close();
        next_reconnect_attempt_ = std::chrono::steady_clock::now();
    }
}

int OpenTxTransmitter::get_channel_value(int channel_idx) const {
    if (!latest_channels_ || channel_idx < 0 || channel_idx >= kMaxChannels) return 0;
    return std::clamp(static_cast<int>((*latest_channels_)[channel_idx]), -1000, 1000);
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
    constexpr int kMax = 1000;
    constexpr int kMin = -1000;
    int value = static_cast<int>(normalized * kMax);
    return std::clamp(value, kMin, kMax);
}

bool OpenTxTransmitter::reconnect_if_needed() {
    if (serial_.is_open()) return true;

    auto now = std::chrono::steady_clock::now();
    if (now < next_reconnect_attempt_) return false;

    next_reconnect_attempt_ = now + kReconnectInterval;
    return initialize();
}

}  // namespace auto_battlebot
