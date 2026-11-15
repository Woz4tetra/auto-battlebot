#pragma once
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <Wire.h>

namespace updown_sensor {
typedef struct {
    float x;
    float y;
    float z;
} vector3_t;

const float RIGHT_SIDE_UP_THRESHOLD = -1.0;
const float UPSIDE_DOWN_THRESHOLD = 1.0;
const uint32_t RECONNECT_INTERVAL = 1000;
const uint8_t BNO055_ADDRESS = 0x28;
const uint8_t BNO055_CHIP_ID_VALUE = 0xA0;
const uint32_t DETAILS_INTERVAL = 1000;
// Error codes in status_t are Wire endTransmission() codes: 0 ok, 1 data too long,
// 2 address NACK, 3 data NACK, 4 other bus error, 5 timeout. These two are added here.
const uint8_t I2C_SHORT_READ = 6;
const uint8_t I2C_NOT_TRIED = 255;

const float YAW_RATE_SIGN = -1.0f;
const float MIN_GRAVITY_FOR_UP_MPS2 = 3.0f;
const float SIGN_CHECK_MIN_RATE_DPS = 60.0f;
const uint32_t SIGN_CHECK_MIN_DISAGREE = 10;

typedef struct {
    bool initialized;
    uint32_t begin_attempts;
    uint32_t begin_failures;
    uint8_t last_chip_id;
    uint8_t chip_id_read_error;
    uint32_t samples_lost_after_begin;
    uint32_t samples;
    uint32_t last_sample_ms;
    uint32_t details_ms;
    uint8_t details_error;
    uint8_t operation_mode;
    uint8_t sys_status;
    uint8_t self_test;
    uint8_t sys_error;
    uint8_t calibration;
    uint32_t gyro_agree;
    uint32_t gyro_disagree;
} status_t;
const uint32_t SAMPLE_INTERVAL = 10;

class UpdownSensor {
   private:
    Adafruit_BNO055 *sensor;
    TwoWire *wire;
    bool initialized = false;
    vector3_t *grav_vec;
    vector3_t *max_grav_vec;
    vector3_t *min_grav_vec;
    vector3_t *orientation;
    vector3_t *gyro_vec;
    vector3_t *linear_accel;
    bool is_upside_down = false;
    uint32_t reconnect_timer = 0;
    uint32_t sample_timer = 0;
    uint32_t sample_us = 0;
    status_t status = {};
    float yaw_rate_cw_dps = 0.0f;
    float heading_rate_cw_dps = 0.0f;
    float up_sign_fallback = 1.0f;
    float prev_heading = 0.0f;
    uint32_t prev_heading_us = 0;
    bool has_prev_heading = false;
    vector3_t *make_unit_vector(float x, float y, float z);
    bool update_sensor(bool allow_blocking_reconnect);
    vector3_t *init_vector3(float x, float y, float z);
    uint8_t read_registers(uint8_t reg, uint8_t *buffer, uint8_t length);
    bool check_chip_id();
    void check_gyro_sign(float heading, uint32_t now_us);

   public:
    UpdownSensor();
    bool begin();
    bool get_is_upside_down(bool allow_blocking_reconnect);
    vector3_t *get() { return grav_vec; }
    vector3_t *get_max() { return max_grav_vec; }
    vector3_t *get_min() { return min_grav_vec; }
    vector3_t *get_orientation() { return orientation; }
    vector3_t *get_gyro_rad_s_chip() { return gyro_vec; }
    vector3_t *get_linear_accel_mps2_chip() { return linear_accel; }
    uint32_t get_sample_us() { return sample_us; }
    void refresh_details();
    const status_t &get_status() { return status; }
    float get_yaw_rate_cw_dps() { return yaw_rate_cw_dps; }
    float get_heading_rate_cw_dps() { return heading_rate_cw_dps; }
    // True once the gyro and the heading change have disagreed in sign more often than they
    // agreed, at least SIGN_CHECK_MIN_DISAGREE times: YAW_RATE_SIGN is wrong for this
    // mounting, and closing a loop on the gyro would spin the robot.
    bool gyro_sign_suspect() {
        return status.gyro_disagree >= SIGN_CHECK_MIN_DISAGREE &&
               status.gyro_disagree > status.gyro_agree;
    }
};
}  // namespace updown_sensor
