#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

namespace wifi_manager {

const size_t MAX_NETWORKS = 8;
const size_t SSID_LEN = 32;
const size_t PASSWORD_LEN = 63;
const uint32_t CONNECT_TIMEOUT_MS = 30000;

enum class link_mode_t : uint8_t { AP = 0, CLIENT = 1 };

enum class link_state_t : uint8_t {
    AP,
    CONNECTING,
    CONNECTED,
    RECONNECTING,
    FALLBACK_AP,
};

struct network_t {
    char ssid[SSID_LEN + 1];
    char password[PASSWORD_LEN + 1];
};

struct settings_t {
    link_mode_t mode;
    char selected[SSID_LEN + 1];
    uint8_t count;
    network_t networks[MAX_NETWORKS];
};

// What the diagnostics page shows. Passwords never leave the manager.
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

class WifiManager {
   public:
    void begin(const char *hostname, const char *ap_ssid, const char *ap_password,
               uint8_t ap_channel, bool force_ap);

    uint8_t station_count() const { return _stations; }

    snapshot_t snapshot();
    const char *ap_ssid() const { return _ap_ssid; }
    uint8_t ap_channel() const { return _ap_channel; }
    const char *hostname() const { return _hostname; }

    bool save_network(const char *ssid, const char *password, String &error);
    bool delete_network(const char *ssid, String &error);
    bool apply(link_mode_t mode, const char *selected, String &error);

   private:
    static void task_entry(void *arg);
    void run();
    void start_from_saved();
    void start_ap(link_state_t state);
    void load();
    void store_locked();
    const network_t *find_locked(const char *ssid) const;

    SemaphoreHandle_t _lock = nullptr;
    settings_t _settings = {};
    const char *_hostname = "";
    const char *_ap_ssid = "";
    const char *_ap_password = "";
    uint8_t _ap_channel = 1;
    bool _force_ap = false;

    // Owned by the task once begin() returns; read by snapshot() under _lock.
    link_state_t _state = link_state_t::AP;
    char _joining[SSID_LEN + 1] = "";
    uint32_t _connect_start_ms = 0;
    volatile bool _apply_requested = false;
    volatile uint8_t _stations = 0;
};

}  // namespace wifi_manager
