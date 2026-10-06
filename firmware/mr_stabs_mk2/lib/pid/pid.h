#pragma once
#include <cmath>

namespace pid {
struct PidConfig {
    float kp = 0.0;
    float ki = 0.0;
    float kd = 0.0;
    float kf = 0.0;
    float i_zone = -1.0;  // -1 means no i_zone limit
    float i_max = 0.0;
    float tolerance = 0.0;
    bool continuous = false;   // Enable continuous angle wrapping
    float min_input = -180.0;  // Minimum input value for continuous mode
    float max_input = 180.0;   // Maximum input value for continuous mode
};

class Pid {
   public:
    /**
     * Implements a PID controller with optional feedforward term and integral zone.
     *
     * Call update() once per new measurement, with dt the time since the previous measurement.
     * Calling it again with a repeated measurement makes the D term read zero.
     *
     * Config arguments:
     * kp: Proportional gain.
     * ki: Integral gain, per second.
     * kd: Derivative gain, in seconds.
     * kf: Feedforward gain.
     * i_zone: Integral zone. If the error is within this zone, the integral term is
     *         accumulated. If set to -1, the integral term is always accumulated.
     * i_max: Maximum magnitude of the integral term's output. If set to 0, the integral term
     *        is not limited.
     * tolerance: Error tolerance for which the controller is considered to have reached the
     *           setpoint. Output is zero while the error is within this tolerance, but the
     *           integral and derivative state keep updating so leaving the band does not kick.
     * continuous: Enable continuous input wrapping (for angles)
     * min_input: Minimum input value for continuous mode (typically -180)
     * max_input: Maximum input value for continuous mode (typically 180)
     */
    Pid(const PidConfig &config);

    /**
     * Reset the integral term and previous error.
     */
    void reset();

    /**
     * Update the PID controller with the current setpoint and measurement.
     *
     * Args:
     *     setpoint: Desired value.
     *     measurement: Current value.
     *     dt: Seconds since the previous measurement.
     *     pd_scale: Multiplies the P and D terms only, for gain scheduling. The integral and
     *               feedforward are left alone, so changing the scale never jumps the output
     *               by a multiple of the accumulated integral.
     *
     * Returns:
     *     Output value.
     */
    float update(float setpoint, float measurement, float dt, float pd_scale = 1.0f);

    float get_error();

   private:
    float _calculate_p(float error);
    float _calculate_i(float error, float dt);
    float _calculate_d(float error, float dt);
    float _calculate_f(float setpoint);
    float _wrap_error(float error);
    float _repeat(float value, float length);
    float _input_modulus(float value, float min_value, float max_value);

    // Configuration parameters
    float kp;
    float ki;
    float kd;
    float kf;
    float i_zone;
    float i_max;
    float tolerance;
    bool continuous;
    float min_input;
    float max_input;

    // State variables
    float i_accum;  // integral of error over time, error * seconds
    float prev_error;
    bool has_prev_error;
    float error;
};
}  // namespace pid
