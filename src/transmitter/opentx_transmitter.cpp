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

struct BodyVelocity {
    double linear;
    double angular;
};
struct WheelPair {
    double left;
    double right;
};

// Clamp a body-frame command to the configured velocity budget. Angular gets priority and linear
// fills the remaining headroom; saturation_limit == 0 falls back to clamping each axis to [-1, 1].
BodyVelocity saturate_velocity(double linear, double angular, double saturation_limit) {
    const double angular_clamped = std::clamp(angular, -1.0, 1.0);
    if (saturation_limit > 0.0) {
        const double limit = std::clamp(saturation_limit, 0.0, 1.0);
        const double headroom = std::max(0.0, limit - std::abs(angular_clamped));
        return {std::clamp(linear, -headroom, headroom), angular_clamped};
    }
    return {std::clamp(linear, -1.0, 1.0), angular_clamped};
}

// Differential-drive mixing. With saturation enabled (limit <= 1) the wheel magnitudes are
// guaranteed to be in [-1, 1]; with saturation disabled they may briefly exceed 1 but the
// per-wheel deadzone clamps them.
WheelPair mix_to_wheels(BodyVelocity body) {
    return {body.linear + body.angular, body.linear - body.angular};
}

BodyVelocity inverse_mix_from_wheels(WheelPair wheels) {
    return {0.5 * (wheels.left + wheels.right), 0.5 * (wheels.left - wheels.right)};
}

// Map |value| from (zero_deadzone, 1] -> [lifted_deadzone, 1] preserving sign. Below
// zero_deadzone the output is forced to 0. Above zero_deadzone the smallest non-zero output is
// lifted_deadzone, which is the minimum throttle that overcomes static friction on a wheel.
double apply_lifted_deadzone(double value, double zero_deadzone, double lifted_deadzone) {
    const double magnitude = std::abs(value);
    if (magnitude <= zero_deadzone) return 0.0;
    const double denom = std::max(1e-6, 1.0 - zero_deadzone);
    const double shifted = std::clamp((magnitude - zero_deadzone) / denom, 0.0, 1.0);
    const double lifted = lifted_deadzone + (1.0 - lifted_deadzone) * shifted;
    return std::copysign(std::clamp(lifted, 0.0, 1.0), value);
}

}  // namespace

OpenTxTransmitter::OpenTxTransmitter(const OpenTxTransmitterConfiguration& config)
    : config_(config), logger_(DiagnosticsLogger::get_logger("opentx_transmitter")) {}

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

    const auto saturated =
        saturate_velocity(command.linear_x, command.angular_z, config_.velocity_saturation_limit);

    // The static-friction barrier exists per wheel, not per body axis, so we apply the lifted
    // deadzone in wheel space. Side-effect: when the robot is already driving forward, a small
    // steering input passes through cleanly (both wheels well past the friction barrier); same
    // for a small linear nudge while spinning in place.
    const auto wheels_in = mix_to_wheels(saturated);
    const double zero_dz = std::clamp(config_.zero_deadzone_percent, 0.0, 100.0) / 100.0;
    const double lifted_dz = std::clamp(config_.lifted_deadzone_percent, 0.0, 100.0) / 100.0;
    const WheelPair wheels_out{
        apply_lifted_deadzone(wheels_in.left, zero_dz, lifted_dz),
        apply_lifted_deadzone(wheels_in.right, zero_dz, lifted_dz),
    };

    auto out = inverse_mix_from_wheels(wheels_out);

    // Compensate for physical motor wiring on the robot. Applied last so the wheel-space math
    // operates in a consistent body frame.
    if (config_.reverse_linear_channel) out.linear = -out.linear;
    if (config_.reverse_angular_channel) out.angular = -out.angular;

    const int linear_value = to_trainer_value(out.linear);
    const int angular_value = to_trainer_value(out.angular);

    logger_->debug("send", {{"linear_x_in", command.linear_x},
                            {"angular_z_in", command.angular_z},
                            {"linear_saturated", saturated.linear},
                            {"angular_saturated", saturated.angular},
                            {"wheel_left_pre", wheels_in.left},
                            {"wheel_right_pre", wheels_in.right},
                            {"wheel_left_post", wheels_out.left},
                            {"wheel_right_post", wheels_out.right},
                            {"linear_out", out.linear},
                            {"angular_out", out.angular},
                            {"velocity_saturation_limit", config_.velocity_saturation_limit},
                            {"lifted_deadzone_percent", config_.lifted_deadzone_percent},
                            {"zero_deadzone_percent", config_.zero_deadzone_percent},
                            {"linear_channel", config_.linear_channel},
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
