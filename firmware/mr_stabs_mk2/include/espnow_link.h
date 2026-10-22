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

// The sweep's candidates. Last-known-good is tried ahead of these.
const uint8_t CHANNEL_CANDIDATES[] = {1, 6, 11};
const uint32_t CHANNEL_DWELL_MS = 25;  // ~6 dongle repeats at 250 Hz, plus switch settling

// The control loop runs at ~1 kHz. Telemetry at that rate would hold the channel ~35% of the
// time at 6 Mbps, so it is sent at 100 Hz, which still resolves every 10 ms control tick.
const uint32_t TELEMETRY_PERIOD_MS = 10;

// Dongles allowed to command, tune, and flash this robot, by the STA MAC each dongle prints at
// boot. Paired from the diagnostics page and kept in NVS, so none are built into the firmware.
// ESP-NOW allows 20 unencrypted peers.
const uint8_t MAX_PAIRED_DONGLES = 8;

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

    /** Link health for the diagnostics page. Leaves enabled, substitution, and driving to the
     *  caller, which owns those flags. */
    void fill_status(espnow_status_t &out, uint32_t now_ms) const;

    /** Adds a dongle to the paired list in NVS. Disarmed only: the NVS write stalls both cores
     *  for a few ms. Returns a pair_result_t. */
    uint8_t pair(const uint8_t *mac, bool armed);
    /** Removes a paired dongle. */
    uint8_t unpair(const uint8_t *mac, bool armed);

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
    void load_paired();
    void store_paired();
    void publish_allowed();

    uint8_t paired_[MAX_PAIRED_DONGLES][6] = {};
    uint8_t paired_count_ = 0;
    uint8_t last_pair_result_ = 0;

    wifi_interface_t ifidx_ = WIFI_IF_STA;
    tunable_ptrs_t tunables_;
    espnow_protocol::CommandTracker tracker_;
    espnow_protocol::OtaSequencer ota_;

    bool begin_ok_ = false;
    uint8_t reply_mac_[6] = {0};
    bool heard_dongle_ = false;

    // Receive counters for fill_status(), from loop() context.
    uint32_t rx_frames_ = 0;
    uint32_t rx_commands_new_ = 0;
    uint32_t rx_commands_repeat_ = 0;
    uint32_t rx_commands_old_ = 0;
    uint32_t rx_pings_ = 0;
    uint32_t rx_tunables_ = 0;
    uint32_t rx_ota_ = 0;
    uint32_t rx_size_errors_ = 0;
    uint32_t tx_telemetry_ = 0;
    uint32_t last_valid_frame_ms_ = 0;
    uint32_t last_telemetry_ms_ = 0;
    uint32_t telemetry_seq_ = 0;

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
