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
// How often refresh_details() rereads the status registers for the diagnostics page.
const uint32_t DETAILS_INTERVAL = 1000;
// Error codes in status_t are Wire endTransmission() codes: 0 ok, 1 data too long,
// 2 address NACK, 3 data NACK, 4 other bus error, 5 timeout. These two are added here.
const uint8_t I2C_SHORT_READ = 6;
const uint8_t I2C_NOT_TRIED = 255;

// The yaw rate is the gyro projected onto the measured up direction (the fused gravity vector,
// which reads +z with the chip level and face up), so it stays the world yaw rate tilted or
// upside down. The gyro is right-handed, counterclockwise-positive about up; the Euler heading
// grows clockwise, hence -1. On the 2026-10-06 run, gyro z alone agreed with the heading change
// on 100% of fast samples upright and on 0% inverted. check_gyro_sign() verifies it live.
const float YAW_RATE_SIGN = -1.0f;
// Below this gravity magnitude (m/s^2) the up direction is unreliable; assume the last one.
const float MIN_GRAVITY_FOR_UP = 3.0f;
// Both rates must exceed this for a sample to vote on whether their signs agree.
const float SIGN_CHECK_MIN_RATE = 60.0f;
// Votes against the sign before the yaw-rate loop is locked out.
const uint32_t SIGN_CHECK_MIN_DISAGREE = 10;

// BNO055 health for the diagnostics page.
typedef struct {
    bool initialized;
    uint32_t begin_attempts;
    uint32_t begin_failures;
    uint8_t chip_id;      // last CHIP_ID read: BNO055_CHIP_ID_VALUE when healthy, 0 if unread
    uint8_t last_error;   // error code of the last CHIP_ID read
    uint32_t lost_count;  // samples whose CHIP_ID check failed after a good begin()
    uint32_t samples;
    uint32_t last_sample_ms;
    // Status registers, read by refresh_details(). details_ms is 0 until the first read.
    uint32_t details_ms;
    uint8_t details_error;
    uint8_t operation_mode;  // OPR_MODE, 0x08 for IMUPLUS
    uint8_t sys_status;      // SYS_STATUS, 5 when fusion is running
    uint8_t self_test;       // ST_RESULT, 0x0F when accel, mag, gyro and MCU all passed
    uint8_t sys_error;       // SYS_ERR, 0 when there is no error
    uint8_t calibration;     // CALIB_STAT: sys, gyro, accel, mag, two bits each
    // Gyro sign check: samples where the gyro and the heading change agreed or disagreed.
    uint32_t gyro_agree;
    uint32_t gyro_disagree;
} status_t;
// The BNO055 fusion output updates at 100 Hz, so reading faster returns repeated values.
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
    bool is_upside_down = false;
    uint32_t reconnect_timer = 0;
    uint32_t sample_timer = 0;
    uint32_t sample_us = 0;
    status_t status = {};
    float yaw_rate = 0.0f;      // deg/s clockwise, from the gyro
    float heading_rate = 0.0f;  // deg/s clockwise, from consecutive headings
    float up_sign = 1.0f;       // +1 chip face up, -1 face down, for when gravity is unreliable
    float prev_heading = 0.0f;
    uint32_t prev_heading_us = 0;
    bool has_prev_heading = false;
    vector3_t *make_unit_vector(float x, float y, float z);
    bool update_sensor(bool radio_connected);
    vector3_t *init_vector3(float x, float y, float z);
    uint8_t read_registers(uint8_t reg, uint8_t *buffer, uint8_t length);
    bool check_chip_id();
    void check_gyro_sign(float heading, uint32_t now_us);

   public:
    UpdownSensor();
    bool begin();
    bool get_is_upside_down(bool radio_connected);
    vector3_t *get() { return grav_vec; }
    vector3_t *get_max() { return max_grav_vec; }
    vector3_t *get_min() { return min_grav_vec; }
    vector3_t *get_orientation() { return orientation; }
    vector3_t *get_gyro() { return gyro_vec; }
    // micros() when the last sample was read, 0 before the first. A change means new data.
    uint32_t get_sample_us() { return sample_us; }
    // Rereads the status registers if DETAILS_INTERVAL has passed and the sensor is up. Two
    // short I2C reads; meant for when the diagnostics page is open.
    void refresh_details();
    const status_t &get_status() { return status; }
    // Yaw rate in deg/s, clockwise-positive like the heading, from the last sample's gyro.
    float get_yaw_rate() { return yaw_rate; }
    // The same rate from the change in heading between the last two samples. Laggier and
    // coarser than the gyro; kept to check the gyro's sign.
    float get_heading_rate() { return heading_rate; }
    // True once the gyro and the heading change have disagreed in sign more often than they
    // agreed, at least SIGN_CHECK_MIN_DISAGREE times: YAW_RATE_SIGN is wrong for this
    // mounting, and closing a loop on the gyro would spin the robot.
    bool gyro_sign_suspect() {
        return status.gyro_disagree >= SIGN_CHECK_MIN_DISAGREE &&
               status.gyro_disagree > status.gyro_agree;
    }
};
}  // namespace updown_sensor
