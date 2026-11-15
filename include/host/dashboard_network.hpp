#pragma once

#include <cstdint>

namespace auto_battlebot {

inline constexpr uint32_t kDashboardCableNet = 0x0A2A2A00;
inline constexpr uint32_t kDashboardCableMask = 0xFFFFFF00;

constexpr bool is_dashboard_cable_address(uint32_t ip) {
    return (ip & kDashboardCableMask) == kDashboardCableNet;
}

}  // namespace auto_battlebot
