#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "data_structures/velocity.hpp"

namespace auto_battlebot {

enum class MonitorLink {
    CAMERA_FRAMES,
    PERCEPTION,
    FILTER,
    TRANSMIT,
    CRSF_TELEMETRY,
    ESPNOW,
    IMU,
    RELAY_PUBLISH,
};

inline constexpr int kMonitorLinkCount = 8;

struct FirmwareInfo {
    uint16_t protocol_version = 0;
    std::string build;
};

struct TransmitterLinkReport {
    std::vector<MonitorLink> links;
    uint64_t commands_sent = 0;
    uint64_t telemetry_frames = 0;
    uint64_t imu_samples = 0;
    std::optional<double> ack_percent;
    std::optional<double> command_age_ms;
    std::optional<double> guard_scale;
    std::optional<int> guard_state;
    std::optional<double> imu_yaw_rate_ccw_radps;
    std::optional<StickCommand> last_sent;
    std::optional<FirmwareInfo> firmware;
    std::optional<uint16_t> host_protocol_version;
};

}  // namespace auto_battlebot
