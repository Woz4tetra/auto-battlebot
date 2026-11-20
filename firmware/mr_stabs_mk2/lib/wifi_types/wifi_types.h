#pragma once

#include <stddef.h>
#include <stdint.h>

namespace wifi_manager {

const size_t MAX_NETWORKS = 8;
const size_t SSID_LEN = 32;

enum class link_mode_t : uint8_t { AP = 0, CLIENT = 1 };

enum class link_state_t : uint8_t {
    AP,
    CONNECTING,
    CONNECTED,
    RECONNECTING,
    FALLBACK_AP,
};

struct snapshot_t {
    link_mode_t mode;
    link_state_t state;
    bool forced_ap;
    char selected[SSID_LEN + 1];
    uint8_t count;
    char networks[MAX_NETWORKS][SSID_LEN + 1];
    char joining_ssid[SSID_LEN + 1];
    char ip[16];
    int8_t rssi;
    uint32_t connect_ms_left;
    uint8_t stations;
};

}  // namespace wifi_manager
