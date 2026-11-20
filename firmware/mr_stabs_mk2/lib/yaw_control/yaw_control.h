#pragma once
#include <stdint.h>

namespace yaw_control {
struct Config {
    float max_rate = 2000.0f;
    float feedforward = 1.0f / 27.0f;
    float kp = 0.02f;
    float ki = 0.02f;
    float i_max = 20.0f;
    float k_heading = 6.0f;
    float hold_rate_max = 360.0f;
    float reverse_kp_scale = 4.0f;
    float reverse_ff_scale = 0.25f;
    float turn_threshold = 1.0f;
    float capture_rate = 45.0f;
    float capture_timeout = 0.3f;
    float idle_throttle = 5.0f;
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

float unwrap_gyro_rate(float raw, float previous);

float heading_dt_s(uint32_t sample_us, uint32_t last_sample_us, uint32_t max_gap_us);

bool gyro_sign_suspect(uint32_t agree, uint32_t disagree, uint32_t min_disagree);
}  // namespace yaw_control
