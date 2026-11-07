#pragma once

#include <cstdint>

namespace auto_battlebot {

/** The subnet install/install_dashboard_network.sh puts on the dashboard cable, 10.42.42.0/24.
 *  The box is 10.42.42.1. A routable private subnet rather than 169.254/16: iOS routes
 *  link-local traffic out its primary interface, so with Wi-Fi on the iPad sent the dashboard
 *  requests over Wi-Fi instead of the cable. */
inline constexpr uint32_t kDashboardCableNet = 0x0A2A2A00;
inline constexpr uint32_t kDashboardCableMask = 0xFFFFFF00;

/** True when `ip` (host byte order) is on the dashboard cable subnet. */
constexpr bool is_dashboard_cable_address(uint32_t ip) {
    return (ip & kDashboardCableMask) == kDashboardCableNet;
}

}  // namespace auto_battlebot
