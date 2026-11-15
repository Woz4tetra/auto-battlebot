#pragma once

namespace yaw_control {
struct Config {
    float max_rate = 2000.0f;          // deg/s at full stick, the BNO055 gyro range
    float feedforward = 1.0f / 27.0f;  // % per deg/s: the sim's open-loop gain at rest
    float kp = 0.02f;                  // % per deg/s of rate error
    float ki = 0.02f;                  // % per degree of integrated rate error
    float i_max = 20.0f;               // % cap on the integral term
    float k_heading = 6.0f;            // deg/s of rate command per degree of heading error
    float hold_rate_max = 360.0f;      // deg/s cap on the heading-hold rate command
    float reverse_kp_scale = 4.0f;     // kp multiplier at full reverse throttle
    float reverse_ff_scale = 0.25f;    // feedforward multiplier at full reverse throttle
    float turn_threshold = 1.0f;       // % of stick above which the driver is turning
    float capture_rate = 45.0f;        // deg/s below which a released turn has stopped
    float capture_timeout = 0.3f;      // s after release to take the heading regardless
    float idle_throttle = 5.0f;        // % of throttle below which a centered stick idles
};

class YawController {
   public:
    explicit YawController(const Config &config);

    void set_config(const Config &config) { _config = config; }

    void reset(float heading);

    float update(float turn_pct, float throttle_pct_tail_first_positive, float heading_deg,
                 float rate_cw_dps, float dt_s);

    float output() const { return _output; }
    float rate_command() const { return _rate_command; }
    float setpoint() const { return _setpoint; }
    bool capturing() const { return _capturing; }

   private:
    void update_capture(float rate_cw_dps, float heading_deg, float dt_s);
    Config _config;
    float _setpoint = 0.0f;
    bool _capturing = false;
    float _capture_time = 0.0f;
    float _integral = 0.0f;
    float _output = 0.0f;
    float _rate_command = 0.0f;
};

float wrap_degrees(float angle);

constexpr float GYRO_WRAP_RAD_S = 4096.0f * 3.14159265358979f / 180.0f;

/** Undo the gyro wrap past its +/-2000 deg/s range. A spin faster than that reads back with the
 *  opposite sign, which turns the yaw-rate loop into positive feedback: it drives the motors
 *  harder to "brake" a spin it sees going the wrong way. Picks the reading's alias nearest the
 *  previous unwrapped rate, which is exact while the rate changes by under half a wrap
 *  (2048 deg/s) between samples. */
float unwrap_gyro_rate(float raw, float previous);
}  // namespace yaw_control
