#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// Chooses between the robot's own access point and joining a router, from settings the
// diagnostics page saves to NVS.
//
// AP mode: the access point comes up on boot and the robot never looks for a router.
// Client mode: the robot joins the selected network on boot. With no connection after
// CONNECT_TIMEOUT_MS it gives up and brings up the access point instead, until the next boot or
// the next apply. A connection that drops later reconnects on its own.
//
// Everything after the first start runs in a task on core 0, so the timeout, the fallback, and
// a mode switch from the page never stall the control loop on core 1.
namespace wifi_manager {

const size_t MAX_NETWORKS = 8;
const size_t SSID_LEN = 32;
const size_t PASSWORD_LEN = 63;
const uint32_t CONNECT_TIMEOUT_MS = 30000;

enum class link_mode_t : uint8_t { AP = 0, CLIENT = 1 };

enum class link_state_t : uint8_t {
    AP,            // AP mode
    CONNECTING,    // client mode, waiting for the first connection
    CONNECTED,     // client mode, joined
    RECONNECTING,  // client mode, joined once and lost it; the driver retries
    FALLBACK_AP,   // client mode, gave up after CONNECT_TIMEOUT_MS
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
    char ssid[SSID_LEN + 1];  // the network joined or being joined
    char ip[16];
    int8_t rssi;
    uint32_t connect_ms_left;
    uint8_t stations;
};

class WifiManager {
   public:
    // Starts WiFi from the saved settings before returning, so ArduinoOTA and the web server can
    // bind afterwards. force_ap keeps the access point up on ap_channel whatever is saved: ESP-NOW
    // needs a fixed channel, which a router would take over.
    void begin(const char *hostname, const char *ap_ssid, const char *ap_password,
               uint8_t ap_channel, bool force_ap);

    // Station count on the access point, 0 in client mode. Refreshed by the task, so it is
    // cheap to read every loop.
    uint8_t station_count() const { return _stations; }

    snapshot_t snapshot();
    const char *ap_ssid() const { return _ap_ssid; }
    uint8_t ap_channel() const { return _ap_channel; }
    const char *hostname() const { return _hostname; }

    // Settings edits from the page. Each saves to NVS before returning; a false return carries a
    // reason in error. Writing flash stalls both cores for a few ms, so the caller refuses these
    // while the robot is armed.
    bool save_network(const char *ssid, const char *password, String &error);
    bool delete_network(const char *ssid, String &error);
    // Saves mode and the selected network, then restarts WiFi with them.
    bool apply(link_mode_t mode, const char *selected, String &error);

   private:
    static void task_entry(void *arg);
    void run();
    void start();  // from the saved settings
    void start_ap(link_state_t state);
    void load();
    void store();
    const network_t *find(const char *ssid) const;

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
