#include "yaw_control.h"

#include <cmath>

namespace yaw_control {
namespace {
float clamp(float value, float low, float high) { return fmin(fmax(value, low), high); }
}  // namespace

float wrap_degrees(float angle) { return angle - 360.0f * floor((angle + 180.0f) / 360.0f); }

float unwrap_gyro_rate(float raw, float previous) {
    return raw + GYRO_WRAP_RAD_S * roundf((previous - raw) / GYRO_WRAP_RAD_S);
}

YawController::YawController(const Config &config) : _config(config) {}

void YawController::reset(float heading) {
    _setpoint = heading;
    _capturing = false;
    _capture_time = 0.0f;
    _integral = 0.0f;
    _output = 0.0f;
    _rate_command = 0.0f;
}

void YawController::update_capture(float rate_cw_dps, float heading_deg, float dt_s) {
    _capture_time += dt_s;
    if (fabs(rate_cw_dps) < _config.capture_rate || _capture_time >= _config.capture_timeout) {
        _capturing = false;
        _setpoint = heading_deg;
    }
}

float YawController::update(float turn_pct, float throttle_pct_tail_first_positive,
                            float heading_deg, float rate_cw_dps, float dt_s) {
    bool turning = fabs(turn_pct) > _config.turn_threshold;
    if (!turning && fabs(throttle_pct_tail_first_positive) < _config.idle_throttle) {
        _capturing = false;
        _setpoint = heading_deg;
        _rate_command = 0.0f;
        _output = 0.0f;
        return _output;
    }

    float feedforward_command = 0.0f;
    if (turning) {
        _rate_command = turn_pct / 100.0f * _config.max_rate;
        feedforward_command = _rate_command;
        _capturing = true;
        _capture_time = 0.0f;
    } else {
        if (_capturing) update_capture(rate_cw_dps, heading_deg, dt_s);
        _rate_command = _capturing
                            ? 0.0f
                            : clamp(_config.k_heading * wrap_degrees(_setpoint - heading_deg),
                                    -_config.hold_rate_max, _config.hold_rate_max);
    }

    float reverse = clamp(throttle_pct_tail_first_positive / 100.0f, 0.0f, 1.0f);
    float kp = _config.kp * (1.0f + (_config.reverse_kp_scale - 1.0f) * reverse);
    float feedforward = _config.feedforward * (1.0f + (_config.reverse_ff_scale - 1.0f) * reverse);
    float error = _rate_command - rate_cw_dps;
    float unsaturated = feedforward * feedforward_command + kp * error + _integral;
    // Integrate only while the output has room, or when the error would pull it back in.
    if (fabs(unsaturated) < 100.0f || (unsaturated > 0.0f) != (error > 0.0f)) {
        _integral = clamp(_integral + _config.ki * error * dt_s, -_config.i_max, _config.i_max);
    }
    _output = clamp(feedforward * feedforward_command + kp * error + _integral, -100.0f, 100.0f);
    return _output;
}
}  // namespace yaw_control
