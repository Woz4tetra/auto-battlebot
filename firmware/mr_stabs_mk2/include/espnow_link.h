#pragma once

// ESP-NOW link to the Jetson's USB dongle: command shortcut, telemetry, and firmware updates.
// Holds no authority. main.cpp substitutes the drive command only after every CRSF gate passes.
// See firmware/mr_stabs_mk2/README.md.

#include <Arduino.h>
#include <command_tracker.h>
#include <diagnostics_server.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <espnow_protocol.h>

namespace espnow_link {

const uint8_t CHANNEL_CANDIDATES[] = {1, 6, 11};
const uint32_t CHANNEL_DWELL_MS = 25;

const uint8_t SWEEP_CHANNEL = 0;
const uint32_t TELEMETRY_PERIOD_MS = 10;

const uint8_t MAX_PAIRED_DONGLES = 8;
static_assert(MAX_PAIRED_DONGLES + 1 <= ESP_NOW_MAX_TOTAL_PEER_NUM,
              "paired dongles plus broadcast must fit the ESP-NOW peer table");

const uint32_t OTA_SILENCE_MS = 5000;

const uint32_t VALIDATE_AFTER_MS = 10000;

class EspNowLink {
   public:
    bool begin(wifi_interface_t ifidx, uint8_t pinned_channel, TuningStore *tuning);

    void update(uint32_t now_ms, bool armed);

    bool ota_in_progress() const { return ota_.active(); }

    bool fresh(uint32_t now_ms) const { return tracker_.fresh(now_ms); }
    const espnow_protocol::CommandPayload &latest() const { return tracker_.latest(); }

    void send_telemetry(espnow_protocol::TelemetryPayload &telemetry, uint32_t now_ms);

    /** Link health for the diagnostics page. Leaves enabled, substitution, and driving to the
     *  caller, which owns those flags. */
    void fill_status(espnow_status_t &out, uint32_t now_ms) const;

    /** Adds a dongle to the paired list in NVS. Disarmed only: the NVS write stalls both cores
     *  for a few ms. Returns a pair_result_t. */
    uint8_t pair(const uint8_t *mac, bool armed);
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
    void ota_abort(espnow_protocol::OtaError error);
    void send_ota_status(espnow_protocol::OtaState state, espnow_protocol::OtaError error);
    void service_sweep(uint32_t now_ms);
    void set_channel(uint8_t channel);
    void service_validation(uint32_t now_ms);
    uint8_t next_sweep_channel() const;
    void send_frame(const uint8_t *mac, uint8_t type, const void *payload, size_t length);
    void load_paired();
    void store_paired();
    void publish_allowed();

    uint8_t paired_[MAX_PAIRED_DONGLES][6] = {};
    uint8_t paired_count_ = 0;
    uint8_t last_pair_result_ = 0;

    wifi_interface_t ifidx_ = WIFI_IF_STA;
    TuningStore *tuning_ = nullptr;
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

    bool sweeping_ = false;
    uint8_t channel_ = 0;
    uint8_t last_good_channel_ = 0;
    size_t sweep_index_ = 0;
    uint32_t sweep_dwell_start_ms_ = 0;

    uint8_t ota_mac_[6] = {0};
    espnow_protocol::OtaState ota_state_ = espnow_protocol::OTA_IDLE;
    uint32_t last_ota_frame_ms_ = 0;
    uint32_t last_drop_status_ms_ = 0;
    uint8_t running_md5_[16] = {0};

    bool validated_ = false;
    uint8_t validated_reports_left_ = 0;
    uint32_t last_validated_report_ms_ = 0;
};

}  // namespace espnow_link
