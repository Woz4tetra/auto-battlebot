#include "motor_guard.h"

#include <cmath>

namespace motor_guard {

namespace {
const float TWO_PI = 6.2831853f;

float clampf(float value, float low, float high) {
    return value < low ? low : (value > high ? high : value);
}

float signf(float value) { return value < 0.0f ? -1.0f : 1.0f; }
}  // namespace

MotorGuard::MotorGuard(const Config &config) : _config(config) {
    _continuous_amps_at_rise_max =
        std::sqrt(config.rise_max_c / (config.k_c_per_a2s * config.tau_s *
                                       (1.0f + config.alpha_per_c * config.rise_max_c)));
    for (int k = 0; k < NUM_MOTORS; k++) _allowed[k] = config.peak_amps;
}

void MotorGuard::restore(float left_rise_c, float right_rise_c) {
    const float rises[NUM_MOTORS] = {left_rise_c, right_rise_c};
    for (int k = 0; k < NUM_MOTORS; k++) {
        _rise_c[k] =
            std::isfinite(rises[k]) ? clampf(rises[k], 0.0f, 2.0f * _config.rise_max_c) : 0.0f;
    }
}

State MotorGuard::state() const {
    if (_tripped) return State::TRIPPED;
    if (_sensors == Sensors::NONE) return State::OFF;
    if (_scale < 1.0f) return State::LIMITING;
    for (int k = 0; k < NUM_MOTORS; k++) {
        if (_rise_c[k] > _config.rise_soft_c) return State::HOT;
    }
    return State::NOMINAL;
}

void MotorGuard::apply(float &left_percent, float &right_percent) const {
    if (_tripped) {
        left_percent = 0.0f;
        right_percent = 0.0f;
        return;
    }
    if (_scale >= 1.0f) return;
    left_percent *= _scale;
    right_percent *= _scale;
}

void MotorGuard::track_sign(int motor, float duty, float erpm) {
    const float margin_erpm = _config.reversal_margin_rad_s * 60.0f * _config.pole_pairs / TWO_PI;
    const bool rising = erpm > _last_erpm[motor] + margin_erpm;
    if (duty != 0.0f && (erpm == 0.0f || (signf(duty) != _rotation_sign[motor] && rising))) {
        _rotation_sign[motor] = signf(duty);
    }
    _last_erpm[motor] = erpm;
}

void MotorGuard::select_sensors(bool shunt_ok, const bool (&speed_ok)[NUM_MOTORS]) {
    const bool both_speeds = speed_ok[0] && speed_ok[1];
    if (shunt_ok) {
        _sensors = both_speeds ? Sensors::FUSED : Sensors::SHUNT_ONLY;
    } else {
        _sensors = both_speeds ? Sensors::ERPM_ONLY : Sensors::NONE;
    }
}

void MotorGuard::model_currents_from_speed(const bool (&speed_ok)[NUM_MOTORS],
                                           const float (&duty)[NUM_MOTORS],
                                           const float (&erpm)[NUM_MOTORS], float v,
                                           float (&model)[NUM_MOTORS]) {
    for (int k = 0; k < NUM_MOTORS; k++) {
        if (speed_ok[k]) {
            track_sign(k, duty[k], erpm[k]);
            _speed[k] = _rotation_sign[k] * TWO_PI * erpm[k] / (60.0f * _config.pole_pairs);
        } else {
            _speed[k] = NAN;
        }
        if (!speed_ok[k] || duty[k] == 0.0f) continue;
        const float r = _config.r_cold_ohm * (1.0f + _config.alpha_per_c * _rise_c[k]);
        model[k] = (duty[k] * v - _config.ke_v_s_per_rad * _speed[k]) / r;
    }
}

void MotorGuard::resolve_contested_reversal(const float (&duty)[NUM_MOTORS],
                                            const float (&erpm)[NUM_MOTORS], float amps, float v,
                                            float (&model)[NUM_MOTORS]) {
    if (_sensors == Sensors::FUSED) {
        bool contested = false;
        for (int k = 0; k < NUM_MOTORS; k++) {
            contested |= duty[k] != 0.0f && erpm[k] > 0.0f && signf(duty[k]) != _rotation_sign[k];
        }
        const float predicted =
            _gain * (duty[0] * model[0] + duty[1] * model[1]) + _config.logic_amps;
        if (contested && predicted > _config.gain_gate_amps && predicted > 2.0f * amps) {
            if (++_shunt_sign_rejections >= 2) {
                for (int k = 0; k < NUM_MOTORS; k++) {
                    if (duty[k] != 0.0f && erpm[k] > 0.0f) {
                        _rotation_sign[k] = signf(duty[k]);
                        _speed[k] = std::fabs(_speed[k]) * _rotation_sign[k];
                        const float r =
                            _config.r_cold_ohm * (1.0f + _config.alpha_per_c * _rise_c[k]);
                        model[k] = (duty[k] * v - _config.ke_v_s_per_rad * _speed[k]) / r;
                    }
                }
                _shunt_sign_rejections = 0;
            }
        } else {
            _shunt_sign_rejections = 0;
        }
    }
}

void MotorGuard::bound_motors_without_erpm(const bool (&speed_ok)[NUM_MOTORS], bool shunt_ok,
                                           const float (&duty)[NUM_MOTORS],
                                           float measured_remainder, float v) {
    for (int k = 0; k < NUM_MOTORS; k++) {
        if (speed_ok[k]) continue;
        if (!shunt_ok || duty[k] == 0.0f) {
            _amps[k] = 0.0f;
            continue;
        }
        const float magnitude = std::fabs(duty[k]);
        const float measured =
            std::fmax(measured_remainder, 0.0f) / std::fmax(magnitude, _config.min_duty);
        const float bound = magnitude * v / _config.r_cold_ohm;
        _amps[k] = signf(duty[k]) * std::fmin(measured, bound);
    }
}

void MotorGuard::update_stall_flags(const bool (&speed_ok)[NUM_MOTORS],
                                    const float (&erpm)[NUM_MOTORS],
                                    const float (&duty)[NUM_MOTORS], float dt_s) {
    for (int k = 0; k < NUM_MOTORS; k++) {
        const bool locked =
            speed_ok[k] && erpm[k] == 0.0f && std::fabs(duty[k]) > _config.stall_duty;
        _stall_s[k] = locked ? _stall_s[k] + dt_s : 0.0f;
        _stalled[k] = _stall_s[k] >= _config.stall_hold_s;
    }
}

void MotorGuard::update_thermal_model(float dt_s) {
    const float decay = std::exp(-dt_s / _config.tau_s);
    for (int k = 0; k < NUM_MOTORS; k++) {
        const float amps_k = _sensors == Sensors::NONE ? 0.0f : _amps[k];
        if (_sensors == Sensors::NONE) _amps[k] = 0.0f;
        _rise_c[k] = decay * _rise_c[k] + (1.0f - decay) * _config.k_c_per_a2s * _config.tau_s *
                                              (1.0f + _config.alpha_per_c * _rise_c[k]) * amps_k *
                                              amps_k;
        if (_rise_c[k] >= _config.trip_rise_c) _tripped = true;
    }
}

void MotorGuard::update(float dt_s, float volts, float amps, float left_duty, float right_duty,
                        float left_erpm, float right_erpm) {
    dt_s = clampf(dt_s, 0.0f, _config.max_dt_s);
    const float duty[NUM_MOTORS] = {clampf(left_duty, -1.0f, 1.0f),
                                    clampf(right_duty, -1.0f, 1.0f)};
    const float erpm[NUM_MOTORS] = {left_erpm, right_erpm};
    bool speed_ok[NUM_MOTORS];
    for (int k = 0; k < NUM_MOTORS; k++) speed_ok[k] = std::isfinite(erpm[k]) && erpm[k] >= 0.0f;

    const bool shunt_ok = std::isfinite(volts) && std::isfinite(amps);
    if (std::isfinite(volts)) {
        _last_ina238_volts = volts;
        _volts_age_s = 0.0f;
    } else {
        _volts_age_s += dt_s;
    }
    const float v =
        _volts_age_s < _config.volts_hold_s ? _last_ina238_volts : _config.fallback_volts;

    select_sensors(shunt_ok, speed_ok);

    float model[NUM_MOTORS] = {0.0f, 0.0f};
    model_currents_from_speed(speed_ok, duty, erpm, v, model);

    resolve_contested_reversal(duty, erpm, amps, v, model);

    const float motor_draw = duty[0] * model[0] + duty[1] * model[1];
    float stalled_draw = 0.0f;
    for (int k = 0; k < NUM_MOTORS; k++) {
        stalled_draw += duty[k] * duty[k] * v /
                        (_config.r_cold_ohm * (1.0f + _config.alpha_per_c * _rise_c[k]));
    }
    bool steady = true;
    for (int k = 0; k < NUM_MOTORS; k++) {
        steady &= std::fabs(_speed[k] - _last_speed[k]) < _config.gain_steady_rad_s;
        steady &= std::fabs(duty[k] - _last_duty[k]) < _config.gain_steady_duty;
        _last_speed[k] = speed_ok[k] ? _speed[k] : 0.0f;
        _last_duty[k] = duty[k];
    }
    _steady_updates = steady ? _steady_updates + 1 : 0;
    if (_sensors == Sensors::FUSED && _steady_updates >= 2 && motor_draw > _config.gain_gate_amps &&
        motor_draw > _config.gain_min_load * stalled_draw) {
        const float raw = (amps - _config.logic_amps) / motor_draw;
        _gain += (1.0f - std::exp(-dt_s / _config.gain_tau_s)) * (raw - _gain);
        _gain = clampf(_gain, _config.gain_min, _config.gain_max);
    }

    float measured_remainder = shunt_ok ? amps - _config.logic_amps : 0.0f;
    for (int k = 0; k < NUM_MOTORS; k++) {
        if (speed_ok[k]) _amps[k] = _gain * model[k];
    }
    bool motoring = shunt_ok;
    for (int k = 0; k < NUM_MOTORS; k++) {
        motoring &= !speed_ok[k] || duty[k] * _amps[k] >= 0.0f;
    }
    for (int k = 0; k < NUM_MOTORS; k++) {
        if (!speed_ok[k]) continue;
        if (motoring && duty[k] != 0.0f) {
            const float bound = std::fmax(measured_remainder, 0.0f) /
                                std::fmax(std::fabs(duty[k]), _config.min_duty);
            _amps[k] = signf(_amps[k]) * std::fmin(std::fabs(_amps[k]), bound);
        }
    }
    for (int k = 0; k < NUM_MOTORS; k++) {
        if (speed_ok[k]) measured_remainder -= duty[k] * _amps[k];
    }
    bound_motors_without_erpm(speed_ok, shunt_ok, duty, measured_remainder, v);

    update_stall_flags(speed_ok, erpm, duty, dt_s);

    update_thermal_model(dt_s);

    update_limiter(dt_s);
}

void MotorGuard::update_limiter(float dt_s) {
    if (_sensors == Sensors::NONE) {
        _log_scale = 0.0f;
        _scale = 1.0f;
        return;
    }
    float error = -INFINITY;
    const float smoothing = 1.0f - std::exp(-dt_s / _config.limit_filter_s);
    for (int k = 0; k < NUM_MOTORS; k++) {
        _limiter_amps_filtered[k] += smoothing * (std::fabs(_amps[k]) - _limiter_amps_filtered[k]);
        const float fold =
            clampf((_rise_c[k] - _config.rise_soft_c) / (_config.rise_max_c - _config.rise_soft_c),
                   0.0f, 1.0f);
        _allowed[k] = _config.peak_amps - (_config.peak_amps - _continuous_amps_at_rise_max) * fold;
        const float magnitude = _limiter_amps_filtered[k];
        if (magnitude < _config.min_amps) continue;
        error = std::fmax(error, std::log(magnitude / _allowed[k]));
    }
    if (error > 0.0f) {
        _log_scale -= _config.loop_rad_s * error * dt_s;
    } else {
        _log_scale += std::fmin(_config.loop_rad_s * -error, _config.recover_per_s) * dt_s;
    }
    _log_scale = clampf(_log_scale, std::log(_config.min_scale), 0.0f);
    _scale = _log_scale >= 0.0f ? 1.0f : std::exp(_log_scale);
}

}  // namespace motor_guard
