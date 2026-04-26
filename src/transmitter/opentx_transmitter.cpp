#include "transmitter/opentx_transmitter.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
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
        const auto& newest_channels = channel_updates.back();
        const bool channels_changed =
            !latest_channels_.has_value() || newest_channels != latest_channels_.value();
        latest_channels_ = newest_channels;

        if (channels_changed) {
            std::vector<int> channel_values(latest_channels_->begin(), latest_channels_->end());
            logger_->debug("channels", {{"values", channel_values}});
        }
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

    // Differential control mode: left_channel carries linear command, right_channel carries
    // angular command.
    //
    // Velocity saturation: angular takes full budget first; linear fills remaining headroom.
    // When saturation_limit == 0 each channel is clamped independently to [-1, 1].
    double angular_normalized = std::clamp(command.angular_z, -1.0, 1.0);
    double linear_normalized;
    if (config_.velocity_saturation_limit > 0.0) {
        const double limit = std::clamp(config_.velocity_saturation_limit, 0.0, 1.0);
        const double linear_headroom =
            std::max(0.0, limit - std::abs(angular_normalized));
        linear_normalized = std::clamp(command.linear_x, -linear_headroom, linear_headroom);
    } else {
        linear_normalized = std::clamp(command.linear_x, -1.0, 1.0);
    }

    if (config_.reverse_left_channel) linear_normalized = -linear_normalized;
    if (config_.reverse_right_channel) angular_normalized = -angular_normalized;

    const double lifted_deadzone =
        std::clamp(config_.lifted_deadzone_percent, 0.0, 100.0) / 100.0;
    const double zero_deadzone = std::clamp(config_.zero_deadzone_percent, 0.0, 100.0) / 100.0;
    auto apply_lifted_deadzone = [lifted_deadzone, zero_deadzone](double normalized) {
        const double magnitude = std::abs(normalized);
        if (magnitude <= zero_deadzone) return 0.0;
        if (magnitude <= 0.0) return 0.0;
        const double denom = std::max(1e-6, 1.0 - zero_deadzone);
        const double shifted = std::clamp((magnitude - zero_deadzone) / denom, 0.0, 1.0);
        const double lifted = lifted_deadzone + (1.0 - lifted_deadzone) * shifted;
        return std::copysign(std::clamp(lifted, 0.0, 1.0), normalized);
    };
    linear_normalized = apply_lifted_deadzone(linear_normalized);
    angular_normalized = apply_lifted_deadzone(angular_normalized);

    int left_channel_val = to_trainer_value(linear_normalized);
    int right_channel_val = to_trainer_value(angular_normalized);

    logger_->debug("send", {{"linear_x_normalized_in", command.linear_x},
                            {"angular_z_normalized_in", command.angular_z},
                            {"linear_normalized_out", linear_normalized},
                            {"angular_normalized_out", angular_normalized},
                            {"velocity_saturation_limit", config_.velocity_saturation_limit},
                            {"lifted_deadzone_percent", config_.lifted_deadzone_percent},
                            {"zero_deadzone_percent", config_.zero_deadzone_percent},
                            {"linear_channel", config_.left_channel},
                            {"angular_channel", config_.right_channel},
                            {"linear_channel_val", left_channel_val},
                            {"angular_channel_val", right_channel_val}});

    bool left_write_ok = serial_.write("trainer " + std::to_string(config_.left_channel) + " " +
                                       std::to_string(left_channel_val) + "\r\n");
    bool right_write_ok = serial_.write("trainer " + std::to_string(config_.right_channel) + " " +
                                        std::to_string(right_channel_val) + "\r\n");

    if (!left_write_ok || !right_write_ok) {
        logger_->warning("write_failed_reconnecting",
                         {{"left_write_ok", left_write_ok}, {"right_write_ok", right_write_ok}});
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
    constexpr int kMax = 500;
    constexpr int kMin = -500;
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
