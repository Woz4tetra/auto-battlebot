#pragma once
#include <Arduino.h>
#include <crash_report.h>
#include <esc_config.h>
#include <espnow_protocol.h>
#include <i2c_bus.h>
#include <telemetry_ring.h>
#include <updown_sensor.h>
#include <vbat_sensor.h>
#include <wifi_manager.h>

typedef struct {
    uint32_t timestamp_ms;
    bool radio_connected;
    bool armed;
    float a_percent;
    float b_percent;
    bool button_state;
    uint8_t flip_switch;
    float left_cmd;
    float right_cmd;
    float accel_x;
    float accel_y;
    float accel_z;
    bool is_upside_down;
    uint32_t loop_us;
    uint8_t wifi_clients;
    float orientation_x;
    float orientation_y;
    float orientation_z;
    float pid_setpoint;
    float pid_output;
    float vbat;          // pack volts from the INA238, NaN when absent
    float ibat;          // pack amps from the INA238, positive discharging, NaN when absent
    float yaw_rate;      // deg/s clockwise from the gyro
    float yaw_rate_cmd;  // deg/s clockwise the yaw loop asks for, NaN when it is not running
    // Motor stall guard, filled by publish_diag() in main.cpp.
    float motor_rise_left;  // C above ambient, thermal model
    float motor_rise_right;
    float motor_speed_left;  // rad/s at the motor, signed, NaN without eRPM
    float motor_speed_right;
    float motor_amps_left;  // estimated winding current I_hat
    float motor_amps_right;
    float guard_scale;         // 1 passes commands through
    float guard_gain;          // shunt correction g
    uint8_t guard_state;       // motor_guard::State
    uint8_t guard_sensors;     // motor_guard::Sensors
    uint8_t stall_flags;       // bit 0 left, bit 1 right
    uint16_t erpm_crc_errors;  // both ESCs summed, from boot
} diag_data_t;

struct espnow_status_t {
    bool enabled;
    bool command_substitution;
    bool esp_now_init_ok;
    bool driving;
    uint8_t channel;
    bool sweeping;
    uint8_t last_good_channel;
    bool heard_dongle;
    uint8_t dongle_mac[6];
    uint32_t last_allowed_frame_ms;
    uint32_t rx_allowed_frames;
    uint32_t rx_commands_new;
    uint32_t rx_commands_repeat;
    uint32_t rx_commands_late_dropped;
    uint32_t rx_pings;
    uint32_t rx_tunables;
    uint32_t rx_ota;
    uint32_t rx_wrong_length;
    uint32_t rx_unpaired_mac;
    uint8_t rejected_mac[6];
    uint32_t rx_queue_full_drops;
    uint32_t tx_telemetry;
    uint32_t tx_acked;
    uint32_t tx_no_ack;
    uint32_t tx_send_refused;
    uint8_t lq;
    bool command_fresh;
    uint32_t command_seq;
    uint32_t command_age_ms;  // UINT32_MAX before the first command
    float command_a;
    float command_b;
    uint8_t ota_state;  // espnow_protocol::OtaState of the last status sent
    bool validated;     // OTA_VALIDATED reported for this boot
    // Paired dongles, from NVS.
    uint8_t allowed[12][6];
    uint8_t allowed_count;
    uint8_t pair_result;  // pair_result_t of the last pair or unpair
};

enum pair_result_t : uint8_t {
    PAIR_NONE = 0,
    PAIR_OK = 1,
    PAIR_ARMED = 2,
    PAIR_FULL = 3,
    PAIR_EXISTS = 4,
    PAIR_NOT_FOUND = 6,
    PAIR_BAD_MAC = 7,
};

// A pair or unpair from the page, applied by loop(), which owns ESP-NOW and NVS writes.
struct pair_request_t {
    uint8_t op;  // 0 none, 1 pair, 2 unpair
    uint8_t mac[6];
};

struct sensor_status_t {
    updown_sensor::status_t imu;
    vbat_sensor::status_t ina;
    i2c_bus::scan_t scan;
    i2c_bus::lines_t lines;
    uint8_t sda_pin;
    uint8_t scl_pin;
    uint32_t snapshot_taken_ms;
    float yaw_rate;
    float heading_rate;
    bool gyro_sign_suspect;
    espnow_status_t espnow;
    crash_report::report_t boot;
};

class TuningStore;

using telemetry_ring_t = telemetry_ring::TelemetryRing<espnow_protocol::TelemetryPayload>;

class DiagnosticsServer {
   public:
    void begin(wifi_manager::WifiManager *wifi, const telemetry_ring_t *ring, uint32_t boot_id,
               TuningStore *tuning = nullptr, esc_config::EscConfig *esc_tuning = nullptr);
    void update(const diag_data_t *data);
    bool page_open();
    // Copies the snapshot for the /status handler, which runs on the network task.
    void set_status(const sensor_status_t &status);
    // Set by the page's Rescan button. The loop runs the scan, since it owns the bus.
    bool scan_requested() const { return _scan_requested; }
    void clear_scan_request() { _scan_requested = false; }
    // Set by the page's ALLOW and REMOVE buttons on the ESP-NOW panel. The loop applies it,
    // since it owns ESP-NOW and the NVS write. False when nothing is pending.
    bool take_pair_request(pair_request_t &out);
    // Set by the page's ERASE DUMP button. The loop erases it, since it owns flash writes.
    bool take_erase_dump_request() {
        const bool requested = _erase_dump_requested;
        _erase_dump_requested = false;
        return requested;
    }

   private:
    bool _recording = false;
    uint32_t _last_send_ms = 0;
    uint32_t _seq = 0;
    TuningStore *_tuning = nullptr;
    volatile bool _scan_requested = false;
    pair_request_t _pair_request = {};
    volatile bool _erase_dump_requested = false;
    // From the latest update(). WiFi settings writes go to flash, which stalls the loop, so
    // the page can only change them disarmed.
    volatile bool _armed = false;
    wifi_manager::WifiManager *_wifi = nullptr;
    const telemetry_ring_t *_ring = nullptr;
    uint32_t _boot_id = 0;
    esc_config::EscConfig *_esc = nullptr;
};
