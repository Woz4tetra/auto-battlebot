#pragma once
#include <cmath>

namespace pid {
struct PidConfig {
    float kp = 0.0;
    float ki = 0.0;
    float kd = 0.0;
    float kf = 0.0;
    float i_zone = -1.0;
    float i_max = 0.0;
    float tolerance = 0.0;
    bool continuous = false;
    float min_input = -180.0;
    float max_input = 180.0;
};

class Pid {
   public:
    Pid(const PidConfig &config);

    void reset();

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

    float i_accum_error_s;
    float prev_error;
    bool has_prev_error;
    float error;
};
}  // namespace pid
