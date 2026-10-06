#include "yaw_control.h"

#include <cmath>

namespace yaw_control {
namespace {
float clamp(float value, float low, float high) { return fmin(fmax(value, low), high); }
}  // namespace

float wrap_degrees(float angle) { return angle - 360.0f * floor((angle + 180.0f) / 360.0f); }

YawController::YawController(const Config &config) : _config(config) {}

void YawController::reset(float heading) {
    _setpoint = heading;
    _capturing = false;
    _capture_time = 0.0f;
    _integral = 0.0f;
    _output = 0.0f;
    _rate_command = 0.0f;
}

float YawController::update(float turn_percent, float heading, float rate, float dt,
                            float reverse) {
    if (fabs(turn_percent) > _config.turn_threshold) {
        _rate_command = turn_percent / 100.0f * _config.max_rate;
        _capturing = true;
        _capture_time = 0.0f;
    } else {
        if (_capturing) {
            // Released: brake the turn at zero rate, then hold wherever it stopped.
            _capture_time += dt;
            if (fabs(rate) < _config.capture_rate || _capture_time >= _config.capture_timeout) {
                _capturing = false;
                _setpoint = heading;
            }
        }
        _rate_command = _capturing ? 0.0f
                                   : clamp(_config.k_heading * wrap_degrees(_setpoint - heading),
                                           -_config.hold_rate_max, _config.hold_rate_max);
    }

    float kp = _config.kp * (1.0f + (_config.reverse_kp_scale - 1.0f) * reverse);
    float feedforward = _config.feedforward * (1.0f + (_config.reverse_ff_scale - 1.0f) * reverse);
    float error = _rate_command - rate;
    float unsaturated = feedforward * _rate_command + kp * error + _integral;
    // Integrate only while the output has room, or when the error would pull it back in.
    if (fabs(unsaturated) < 100.0f || (unsaturated > 0.0f) != (error > 0.0f)) {
        _integral = clamp(_integral + _config.ki * error * dt, -_config.i_max, _config.i_max);
    }
    _output = clamp(feedforward * _rate_command + kp * error + _integral, -100.0f, 100.0f);
    return _output;
}
}  // namespace yaw_control
