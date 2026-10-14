#pragma once

// ESP-NOW link to the Jetson's USB dongle: command shortcut, telemetry, and firmware updates.
// Holds no authority. main.cpp substitutes the drive command only after every CRSF gate passes.
// See docs/plans/espnow_link_plan.md.

#include <Arduino.h>
#include <command_tracker.h>
#include <diagnostics_server.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <espnow_protocol.h>

namespace espnow_link {

// Dongles allowed to command, tune, and flash this robot: the STA MAC each dongle prints at boot.
// Add the bench dongle and the match (u.FL) dongle here as they are built.
const uint8_t DONGLE_MACS[][6] = {
    {0xD8, 0x3B, 0xDA, 0x8E, 0xED, 0x2C},  // QT Py ESP32-S3 no-PSRAM, development dongle
};

// The sweep's candidates. Last-known-good is tried ahead of these.
const uint8_t CHANNEL_CANDIDATES[] = {1, 6, 11};
const uint32_t CHANNEL_DWELL_MS = 25;  // ~6 dongle repeats at 250 Hz, plus switch settling

// ESP-NOW transmit rate. 1 Mbps until the new PCBA: the current robot board hears the dongle at
// -85 dBm from 30 cm, and 1 Mbps buys ~5 dB of receive sensitivity over 6 Mbps for ~1 ms more
// airtime a frame. Set back to WIFI_PHY_RATE_6M, here and in the dongle, once the new board
// measures ~-40 dBm. See "Robot radio" in docs/plans/espnow_link_plan.md.
const wifi_phy_rate_t ESPNOW_PHY_RATE = WIFI_PHY_RATE_1M_L;

// The control loop runs at ~1 kHz. Telemetry at that rate would hold the channel ~35% of the
// time at 6 Mbps, so it is sent at 100 Hz, which still resolves every 10 ms control tick.
const uint32_t TELEMETRY_PERIOD_MS = 10;

// OTA transfer gives up after this long without a valid OTA frame. The running image is untouched
// until Update.end() switches the boot slot, so an abort always leaves the old firmware running.
const uint32_t OTA_SILENCE_MS = 5000;

// Every boot reports OTA_VALIDATED, with its MD5, once it has run this long and heard a dongle.
// The flasher waits for it as proof the new image runs.
const uint32_t VALIDATE_AFTER_MS = 10000;

class EspNowLink {
   public:
    /** Starts ESP-NOW on an interface that is already up. ifidx is WIFI_IF_AP while the SoftAP
     *  exists and WIFI_IF_STA without it. channel is the pinned channel, or 0 to sweep. */
    bool begin(wifi_interface_t ifidx, uint8_t channel, const tunable_ptrs_t &tunables);

    /** Drains received frames and runs the OTA, sweep, and validation state machines. Call once per
     *  loop(), after crsf->update() so armed is current. Arming aborts an OTA transfer. */
    void update(uint32_t now_ms, bool armed);

    /** True while an OTA transfer owns the loop. main.cpp stops the ESCs and skips control. */
    bool ota_in_progress() const { return ota_.active(); }

    bool fresh(uint32_t now_ms) const { return tracker_.fresh(now_ms); }
    const espnow_protocol::CommandPayload &latest() const { return tracker_.latest(); }

    /** Fills in the link fields and sends at most every TELEMETRY_PERIOD_MS. */
    void send_telemetry(espnow_protocol::TelemetryPayload &telemetry, uint32_t now_ms);

   private:
    static void on_recv(const uint8_t *mac, const uint8_t *data, int length);
    static void on_sent(const uint8_t *mac, esp_now_send_status_t status);

    void handle_frame(const uint8_t *mac, uint8_t type, const uint8_t *payload, size_t length,
                      uint32_t now_ms, bool armed);
    void ota_begin(const uint8_t *mac, const uint8_t *payload, size_t length, uint32_t now_ms,
                   bool armed);
    void ota_chunk(const uint8_t *payload, size_t length, uint32_t now_ms);
    void ota_end();
    void ota_abort(uint8_t error);
    void send_ota_status(uint8_t state, uint8_t error);
    void service_sweep(uint32_t now_ms);
    void set_channel(uint8_t channel);
    void service_validation(uint32_t now_ms);
    void send_frame(const uint8_t *mac, uint8_t type, const void *payload, size_t length);

    wifi_interface_t ifidx_ = WIFI_IF_STA;
    tunable_ptrs_t tunables_;
    espnow_protocol::CommandTracker tracker_;
    espnow_protocol::OtaSequencer ota_;

    uint8_t reply_mac_[6] = {0};
    bool heard_dongle_ = false;
    uint32_t last_valid_frame_ms_ = 0;
    uint32_t last_telemetry_ms_ = 0;

    // Channel: pinned, or swept until a dongle frame arrives.
    bool sweeping_ = false;
    uint8_t channel_ = 0;
    uint8_t last_good_channel_ = 0;
    size_t sweep_index_ = 0;
    uint32_t sweep_dwell_start_ms_ = 0;

    uint8_t ota_mac_[6] = {0};
    uint8_t ota_state_ = espnow_protocol::OTA_IDLE;
    uint32_t last_ota_frame_ms_ = 0;
    uint32_t last_drop_status_ms_ = 0;
    uint8_t running_md5_[16] = {0};

    bool validated_ = false;
    uint8_t validated_reports_left_ = 0;
    uint32_t last_validated_report_ms_ = 0;
};

}  // namespace espnow_link
