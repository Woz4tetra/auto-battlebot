#pragma once
#include <Arduino.h>
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
} diag_data_t;

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

   private:
    bool _recording = false;
    uint32_t _last_send_ms = 0;
    uint32_t _seq = 0;  // +1 per row sent, so a gap in a recording shows dropped rows
    tunable_ptrs_t _tunables;
    volatile bool _scan_requested = false;
    // From the latest update(). WiFi settings writes go to flash, which stalls the loop, so
    // the page can only change them disarmed.
    volatile bool _armed = false;
    wifi_manager::WifiManager *_wifi = nullptr;
    const telemetry_ring_t *_ring = nullptr;
    uint32_t _boot_id = 0;
};
