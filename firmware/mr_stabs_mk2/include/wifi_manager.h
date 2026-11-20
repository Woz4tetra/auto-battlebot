#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <wifi_types.h>

namespace wifi_manager {

const size_t PASSWORD_LEN = 63;
const uint32_t CONNECT_TIMEOUT_MS = 30000;

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

    link_state_t _state = link_state_t::AP;
    char _joining[SSID_LEN + 1] = "";
    uint32_t _connect_start_ms = 0;
    volatile bool _apply_requested = false;
    volatile uint8_t _stations = 0;
};

}  // namespace wifi_manager
