#pragma once

#include <cstdint>
#include <string>
#include <variant>

namespace auto_battlebot {

enum class CrsfFrameType : uint8_t {
    BATTERY = 0x08,
    LINK_STATISTICS = 0x14,
    ATTITUDE = 0x1E,
    FLIGHT_MODE = 0x21,
};

struct CrsfBattery {
    static constexpr CrsfFrameType frame_type = CrsfFrameType::BATTERY;
    float voltage = 0.0f;    // volts
    float current = 0.0f;    // amps
    float consumption = 0.0f;  // mAh
};

struct CrsfLinkStatistics {
    static constexpr CrsfFrameType frame_type = CrsfFrameType::LINK_STATISTICS;
    uint8_t up_rssi_ant1 = 0;      // dBm * -1
    uint8_t up_rssi_ant2 = 0;      // dBm * -1
    uint8_t up_link_quality = 0;   // %
    int8_t up_snr = 0;             // dB
    uint8_t active_antenna = 0;    // 0 or 1
    uint8_t rf_profile = 0;        // 0=4fps, 1=50fps, 2=150fps
    uint8_t up_rf_power = 0;       // 0=0mW, 1=10mW, 2=25mW, ...
    uint8_t down_rssi = 0;         // dBm * -1
    uint8_t down_link_quality = 0; // %
    int8_t down_snr = 0;           // dB
};

struct CrsfAttitude {
    static constexpr CrsfFrameType frame_type = CrsfFrameType::ATTITUDE;
    float roll = 0.0f;   // radians
    float pitch = 0.0f;  // radians
    float yaw = 0.0f;    // radians
};

struct CrsfFlightMode {
    static constexpr CrsfFrameType frame_type = CrsfFrameType::FLIGHT_MODE;
    std::string flight_mode;
};

using CrsfPacket =
    std::variant<CrsfBattery, CrsfLinkStatistics, CrsfAttitude, CrsfFlightMode>;

}  // namespace auto_battlebot
