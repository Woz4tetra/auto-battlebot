#pragma once
#include <Arduino.h>
#include <crash_report.h>
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

// ESP-NOW link health for the page's ESP-NOW panel. Counters run from boot; the page turns them
// into rates. Filled by espnow_link::EspNowLink::fill_status().
struct espnow_status_t {
    bool enabled;       // ESPNOW_ENABLED
    bool substitution;  // ESPNOW_COMMAND_SUBSTITUTION: drives from ESP-NOW, not shadow mode
    bool begin_ok;      // esp_now_init() succeeded
    bool driving;       // this tick's drive command came from ESP-NOW
    uint8_t channel;
    bool sweeping;
    uint8_t last_good_channel;
    bool heard_dongle;
    uint8_t dongle_mac[6];
    uint32_t last_frame_ms;  // millis() of the last frame from an allowed dongle
    uint32_t rx_frames;      // from allowed dongles, any type
    uint32_t rx_commands_new;
    uint32_t rx_commands_repeat;
    uint32_t rx_commands_old;  // late frames the tracker dropped
    uint32_t rx_pings;
    uint32_t rx_tunables;
    uint32_t rx_ota;
    uint32_t rx_size_errors;  // known type, wrong length: mismatched builds
    uint32_t rx_rejected;     // session magic from a MAC not in the allowed list
    uint8_t rejected_mac[6];
    uint32_t rx_queue_full;  // dropped because loop() fell behind the WiFi task
    uint32_t tx_telemetry;
    uint32_t tx_ok;      // send callback ACKed, all frame types
    uint32_t tx_fail;    // send callback reported no ACK after retries
    uint32_t tx_errors;  // esp_now_send() refused the frame
    uint8_t lq;          // the telemetry espnow_lq: ACK percent over the last 32 sends
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

// Sensor and I2C bus health, served as JSON at /status for the page's sensor panel.
struct sensor_status_t {
    updown_sensor::status_t imu;
    vbat_sensor::status_t ina;
    i2c_bus::scan_t scan;
    i2c_bus::lines_t lines;
    uint8_t sda_pin;
    uint8_t scl_pin;
    // millis() when the loop took this snapshot. Ages on the page are measured to this moment,
    // not to when the request arrives, since the snapshot is refreshed only once a second.
    uint32_t snapshot_ms;
    float yaw_rate;          // deg/s clockwise from the gyro, at snapshot time
    float heading_rate;      // deg/s clockwise from the heading change, at snapshot time
    bool gyro_sign_suspect;  // the yaw loop is locked out
    espnow_status_t espnow;
    crash_report::report_t boot;
};

struct tunable_ptrs_t {
    float *left_esc_deadzone = nullptr;
    float *right_esc_deadzone = nullptr;
};

using telemetry_ring_t = telemetry_ring::TelemetryRing<espnow_protocol::TelemetryPayload>;

class DiagnosticsServer {
   public:
    // Serves the page from web/ (embedded at build time by scripts/embed_web.py), the WiFi
    // settings endpoints, which edit wifi's saved networks, and GET /telemetry, which reads ring.
    // boot_id goes out with every /telemetry reply so a client can tell a reboot restarted seqs.
    void begin(wifi_manager::WifiManager *wifi, const telemetry_ring_t *ring, uint32_t boot_id,
               tunable_ptrs_t tunables = {});
    void update(const diag_data_t *data);
    // True while a browser has the page open. Gate optional diagnostic work on this.
    bool has_clients();
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
    uint32_t _seq = 0;  // +1 per row sent, so a gap in a recording shows dropped rows
    tunable_ptrs_t _tunables;
    volatile bool _scan_requested = false;
    pair_request_t _pair_request = {};
    volatile bool _erase_dump_requested = false;
    // From the latest update(). WiFi settings writes go to flash, which stalls the loop, so
    // the page can only change them disarmed.
    volatile bool _armed = false;
    wifi_manager::WifiManager *_wifi = nullptr;
    const telemetry_ring_t *_ring = nullptr;
    uint32_t _boot_id = 0;
};
