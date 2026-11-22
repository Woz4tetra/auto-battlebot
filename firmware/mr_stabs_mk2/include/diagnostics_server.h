#pragma once
#include <Arduino.h>
#include <crash_report.h>
#include <esc_config.h>
#include <espnow_protocol.h>
#include <i2c_bus.h>
#include <motor_guard.h>
#include <updown_sensor.h>
#include <vbat_sensor.h>
#include <wifi_manager.h>

#include <atomic>

constexpr uint8_t STALL_FLAG_LEFT = 1;
constexpr uint8_t STALL_FLAG_RIGHT = 2;

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
    float pack_volts;
    float pack_amps_discharge;
    float yaw_rate_cw_dps;
    float yaw_rate_cmd_cw_dps;
    float motor_rise_left_c;
    float motor_rise_right_c;
    float motor_speed_left_rad_s;
    float motor_speed_right_rad_s;
    float motor_amps_est_left;
    float motor_amps_est_right;
    float guard_scale;
    float guard_shunt_gain;
    motor_guard::State guard_state;
    motor_guard::Sensors guard_sensors;
    uint8_t stall_flags;
    uint16_t erpm_crc_errors;
} diag_data_t;

enum pair_result_t : uint8_t {
    PAIR_NONE = 0,
    PAIR_OK = 1,
    PAIR_ARMED = 2,
    PAIR_FULL = 3,
    PAIR_EXISTS = 4,
    PAIR_NOT_FOUND = 6,
    PAIR_BAD_MAC = 7,
};

constexpr uint32_t COMMAND_AGE_NONE = UINT32_MAX;

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
    uint32_t command_age_ms;
    float command_a;
    float command_b;
    espnow_protocol::OtaState ota_state;
    bool validated;
    uint8_t allowed[12][6];
    uint8_t allowed_count;
    pair_result_t pair_result;
};

enum class PairOp : uint8_t { NONE, PAIR, UNPAIR };

struct pair_request_t {
    PairOp op;
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

class LoopFlag {
   public:
    void post() { _flag = true; }
    bool pending() const { return _flag; }
    bool take() { return _flag.exchange(false); }

   private:
    std::atomic<bool> _flag{false};
};

class TuningStore;

class DiagnosticsServer {
   public:
    void begin(wifi_manager::WifiManager *wifi, TuningStore *tuning = nullptr,
               esc_config::EscConfig *esc_tuning = nullptr);
    void update(const diag_data_t *data);
    bool page_open();
    void set_status(const sensor_status_t &status);
    bool scan_requested() const { return _scan_request.pending(); }
    void clear_scan_request() { _scan_request.take(); }
    bool take_pair_request(pair_request_t &out);
    bool take_erase_dump_request() { return _erase_dump_request.take(); }

   private:
    bool _recording = false;
    uint32_t _last_send_ms = 0;
    uint32_t _seq = 0;
    TuningStore *_tuning = nullptr;
    LoopFlag _scan_request;
    pair_request_t _pair_request = {};
    LoopFlag _erase_dump_request;
    volatile bool _armed = false;
    wifi_manager::WifiManager *_wifi = nullptr;
    esc_config::EscConfig *_esc = nullptr;
};
