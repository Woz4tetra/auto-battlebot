#pragma once
#include <Arduino.h>
#include <i2c_bus.h>
#include <updown_sensor.h>
#include <vbat_sensor.h>

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
    float vbat;  // pack volts from the INA228, NaN when absent
    float ibat;  // pack amps from the INA228, positive discharging, NaN when absent
} diag_data_t;

// Sensor and I2C bus health, served as JSON at /status for the page's sensor panel.
struct sensor_status_t {
    updown_sensor::status_t imu;
    vbat_sensor::status_t ina;
    i2c_bus::scan_t scan;
    i2c_bus::lines_t lines;
    uint8_t sda_pin;
    uint8_t scl_pin;
};

struct tunable_ptrs_t {
    float *left_esc_deadzone = nullptr;
    float *right_esc_deadzone = nullptr;
};

class DiagnosticsServer {
   public:
    void begin(tunable_ptrs_t tunables = {});
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
    tunable_ptrs_t _tunables;
    volatile bool _scan_requested = false;
};
