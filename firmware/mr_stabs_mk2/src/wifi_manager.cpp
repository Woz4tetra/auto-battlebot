#include "wifi_manager.h"

#include <Preferences.h>
#include <WiFi.h>

namespace wifi_manager {

static const char *PREFS_NAMESPACE = "wifi";
static const uint32_t TASK_PERIOD_MS = 250;
// Lets the page's HTTP reply leave before an apply takes its network down.
static const uint32_t APPLY_DELAY_MS = 500;

// Fields the task refreshes for snapshot(), so the web server's task never calls into WiFi.
static char cached_ip[16] = "";
static int8_t cached_rssi = 0;

static void copy_str(char *dst, const char *src, size_t size) {
    strncpy(dst, src, size - 1);
    dst[size - 1] = '\0';
}

void WifiManager::begin(const char *hostname, const char *ap_ssid, const char *ap_password,
                        uint8_t ap_channel, bool force_ap) {
    _lock = xSemaphoreCreateMutex();
    _hostname = hostname;
    _ap_ssid = ap_ssid;
    _ap_password = ap_password;
    _ap_channel = ap_channel;
    _force_ap = force_ap;
    load();

    // The core otherwise writes the station config to flash on every WiFi.begin(), and a flash
    // write stalls the control loop. The settings live in our own NVS namespace instead.
    WiFi.persistent(false);
    WiFi.setHostname(_hostname);
    start();

    xTaskCreatePinnedToCore(task_entry, "wifi_manager", 4096, this, 1, nullptr, 0);
}

void WifiManager::task_entry(void *arg) { static_cast<WifiManager *>(arg)->run(); }

void WifiManager::run() {
    for (;;) {
        if (_apply_requested) {
            vTaskDelay(pdMS_TO_TICKS(APPLY_DELAY_MS));
            _apply_requested = false;
            start();
        }

        const bool connected = WiFi.status() == WL_CONNECTED;
        const uint32_t now = millis();
        xSemaphoreTake(_lock, portMAX_DELAY);
        link_state_t state = _state;
        const uint32_t connect_start_ms = _connect_start_ms;
        xSemaphoreGive(_lock);

        link_state_t next = state;
        switch (state) {
            case link_state_t::CONNECTING:
                if (connected) next = link_state_t::CONNECTED;
                break;
            case link_state_t::CONNECTED:
                if (!connected) next = link_state_t::RECONNECTING;
                break;
            case link_state_t::RECONNECTING:
                if (connected) next = link_state_t::CONNECTED;
                break;
            default:
                break;
        }
        if (state == link_state_t::CONNECTING && !connected &&
            now - connect_start_ms >= CONNECT_TIMEOUT_MS) {
            start_ap(link_state_t::FALLBACK_AP);
        } else if (next != state) {
            xSemaphoreTake(_lock, portMAX_DELAY);
            _state = next;
            xSemaphoreGive(_lock);
        }

        const bool ap = (WiFi.getMode() & WIFI_MODE_AP) != 0;
        _stations = ap ? WiFi.softAPgetStationNum() : 0;
        const IPAddress ip = ap ? WiFi.softAPIP() : (connected ? WiFi.localIP() : IPAddress());
        const int8_t rssi = connected ? WiFi.RSSI() : 0;
        xSemaphoreTake(_lock, portMAX_DELAY);
        if (ip == IPAddress()) {
            cached_ip[0] = '\0';
        } else {
            copy_str(cached_ip, ip.toString().c_str(), sizeof(cached_ip));
        }
        cached_rssi = rssi;
        xSemaphoreGive(_lock);

        vTaskDelay(pdMS_TO_TICKS(TASK_PERIOD_MS));
    }
}

void WifiManager::start() {
    network_t net = {};
    bool client = false;
    xSemaphoreTake(_lock, portMAX_DELAY);
    if (!_force_ap && _settings.mode == link_mode_t::CLIENT) {
        const network_t *found = find(_settings.selected);
        if (found) {
            net = *found;
            client = true;
        }
    }
    xSemaphoreGive(_lock);

    if (!client) {
        start_ap(link_state_t::AP);
        return;
    }

    if (WiFi.getMode() & WIFI_MODE_AP) WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_STA);
    // Modem sleep holds incoming packets for up to 100-300 ms.
    WiFi.setSleep(false);
    WiFi.setAutoReconnect(true);
    xSemaphoreTake(_lock, portMAX_DELAY);
    _state = link_state_t::CONNECTING;
    copy_str(_joining, net.ssid, sizeof(_joining));
    _connect_start_ms = millis();
    xSemaphoreGive(_lock);
    WiFi.begin(net.ssid, net.password);
}

void WifiManager::start_ap(link_state_t state) {
    if (WiFi.getMode() & WIFI_MODE_STA) WiFi.disconnect(true);
    WiFi.mode(WIFI_AP);
    WiFi.softAP(_ap_ssid, _ap_password, _ap_channel);
    xSemaphoreTake(_lock, portMAX_DELAY);
    _state = state;
    xSemaphoreGive(_lock);
}

snapshot_t WifiManager::snapshot() {
    snapshot_t s = {};
    xSemaphoreTake(_lock, portMAX_DELAY);
    s.mode = _settings.mode;
    s.state = _state;
    s.forced_ap = _force_ap;
    copy_str(s.selected, _settings.selected, sizeof(s.selected));
    s.count = _settings.count;
    for (uint8_t i = 0; i < _settings.count; i++) {
        copy_str(s.networks[i], _settings.networks[i].ssid, sizeof(s.networks[i]));
    }
    copy_str(s.ssid, _joining, sizeof(s.ssid));
    copy_str(s.ip, cached_ip, sizeof(s.ip));
    s.rssi = cached_rssi;
    if (_state == link_state_t::CONNECTING) {
        const uint32_t elapsed = millis() - _connect_start_ms;
        s.connect_ms_left = elapsed < CONNECT_TIMEOUT_MS ? CONNECT_TIMEOUT_MS - elapsed : 0;
    }
    s.stations = _stations;
    xSemaphoreGive(_lock);
    return s;
}

bool WifiManager::save_network(const char *ssid, const char *password, String &error) {
    const size_t ssid_len = strlen(ssid), password_len = strlen(password);
    if (ssid_len == 0 || ssid_len > SSID_LEN) {
        error = "SSID must be 1 to 32 characters";
        return false;
    }
    // WPA2 takes 8 to 63 characters; empty joins an open network.
    if (password_len != 0 && (password_len < 8 || password_len > PASSWORD_LEN)) {
        error = "password must be empty (open network) or 8 to 63 characters";
        return false;
    }
    xSemaphoreTake(_lock, portMAX_DELAY);
    network_t *net = const_cast<network_t *>(find(ssid));
    if (!net) {
        if (_settings.count >= MAX_NETWORKS) {
            xSemaphoreGive(_lock);
            error = "8 networks saved already; delete one first";
            return false;
        }
        net = &_settings.networks[_settings.count++];
    }
    copy_str(net->ssid, ssid, sizeof(net->ssid));
    copy_str(net->password, password, sizeof(net->password));
    if (_settings.selected[0] == '\0')
        copy_str(_settings.selected, ssid, sizeof(_settings.selected));
    store();
    xSemaphoreGive(_lock);
    return true;
}

bool WifiManager::delete_network(const char *ssid, String &error) {
    xSemaphoreTake(_lock, portMAX_DELAY);
    const network_t *net = find(ssid);
    if (!net) {
        xSemaphoreGive(_lock);
        error = "no saved network with that SSID";
        return false;
    }
    const size_t index = net - _settings.networks;
    for (size_t i = index; i + 1 < _settings.count; i++) {
        _settings.networks[i] = _settings.networks[i + 1];
    }
    _settings.count--;
    // Leave no stale password behind in NVS.
    memset(&_settings.networks[_settings.count], 0, sizeof(network_t));
    if (strcmp(_settings.selected, ssid) == 0) {
        copy_str(_settings.selected, _settings.count ? _settings.networks[0].ssid : "",
                 sizeof(_settings.selected));
    }
    store();
    xSemaphoreGive(_lock);
    return true;
}

bool WifiManager::apply(link_mode_t mode, const char *selected, String &error) {
    xSemaphoreTake(_lock, portMAX_DELAY);
    if (mode == link_mode_t::CLIENT && !find(selected)) {
        xSemaphoreGive(_lock);
        error = "pick a saved network for client mode";
        return false;
    }
    _settings.mode = mode;
    if (find(selected)) copy_str(_settings.selected, selected, sizeof(_settings.selected));
    store();
    xSemaphoreGive(_lock);
    if (!_force_ap) _apply_requested = true;
    return true;
}

// Caller holds _lock.
const network_t *WifiManager::find(const char *ssid) const {
    if (ssid[0] == '\0') return nullptr;
    for (uint8_t i = 0; i < _settings.count; i++) {
        if (strcmp(_settings.networks[i].ssid, ssid) == 0) return &_settings.networks[i];
    }
    return nullptr;
}

void WifiManager::load() {
    _settings = {};
    Preferences prefs;
    // Fails on first boot, before the namespace exists: AP mode, nothing saved.
    if (!prefs.begin(PREFS_NAMESPACE, true)) return;
    _settings.mode = prefs.getUChar("mode", 0) == 1 ? link_mode_t::CLIENT : link_mode_t::AP;
    prefs.getString("selected", _settings.selected, sizeof(_settings.selected));
    const uint8_t count = prefs.getUChar("count", 0);
    if (count <= MAX_NETWORKS && prefs.getBytesLength("networks") == sizeof(_settings.networks) &&
        prefs.getBytes("networks", _settings.networks, sizeof(_settings.networks)) ==
            sizeof(_settings.networks)) {
        _settings.count = count;
    }
    prefs.end();
    for (network_t &net : _settings.networks) {
        net.ssid[SSID_LEN] = '\0';
        net.password[PASSWORD_LEN] = '\0';
    }
}

// Caller holds _lock.
void WifiManager::store() {
    Preferences prefs;
    prefs.begin(PREFS_NAMESPACE, false);
    prefs.putUChar("mode", static_cast<uint8_t>(_settings.mode));
    prefs.putString("selected", _settings.selected);
    prefs.putUChar("count", _settings.count);
    prefs.putBytes("networks", _settings.networks, sizeof(_settings.networks));
    prefs.end();
}

}  // namespace wifi_manager
