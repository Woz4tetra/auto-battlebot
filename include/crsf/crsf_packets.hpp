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
    float voltage_v = 0.0f;
    float current_a = 0.0f;
    float consumption_mah = 0.0f;
};

enum class CrsfRfProfile : uint8_t { FPS_4 = 0, FPS_50 = 1, FPS_150 = 2 };

enum class CrsfRfPower : uint8_t {
    MW_0 = 0,
    MW_10 = 1,
    MW_25 = 2,
    MW_100 = 3,
    MW_500 = 4,
    MW_1000 = 5,
    MW_2000 = 6,
    MW_250 = 7,
    MW_50 = 8
};

struct CrsfLinkStatistics {
    static constexpr CrsfFrameType frame_type = CrsfFrameType::LINK_STATISTICS;
    uint8_t up_rssi_ant1_neg_dbm = 0;
    uint8_t up_rssi_ant2_neg_dbm = 0;
    uint8_t up_link_quality_pct = 0;
    int8_t up_snr_db = 0;
    uint8_t active_antenna = 0;
    CrsfRfProfile rf_profile = CrsfRfProfile::FPS_4;
    CrsfRfPower up_rf_power = CrsfRfPower::MW_0;
    uint8_t down_rssi_neg_dbm = 0;
    uint8_t down_link_quality_pct = 0;
    int8_t down_snr_db = 0;
};

struct CrsfAttitude {
    static constexpr CrsfFrameType frame_type = CrsfFrameType::ATTITUDE;
    float roll_rad = 0.0f;
    float pitch_rad = 0.0f;
    float yaw_rad = 0.0f;
};

struct CrsfFlightMode {
    static constexpr CrsfFrameType frame_type = CrsfFrameType::FLIGHT_MODE;
    std::string flight_mode;
};

using CrsfPacket = std::variant<CrsfBattery, CrsfLinkStatistics, CrsfAttitude, CrsfFlightMode>;

}  // namespace auto_battlebot
