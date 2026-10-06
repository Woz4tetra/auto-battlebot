#pragma once

namespace yaw_control {
// Cascaded heading and yaw-rate control for the flip-switch-DOWN drive mode.
//
// The turn stick commands a yaw rate. With the stick centered, heading hold commands the rate
// that steers back to the held heading. An inner loop on the BNO055 gyro turns the rate command
// into the left/right differential the mixer adds to the throttle: feedforward from the command
// plus proportional and integral feedback on the rate error.
//
// The rate loop exists for reverse. Mr Stabs' center of mass sits 32 mm ahead of the axle, so
// driving tail-first is directionally unstable and a turn there builds ~1000 deg/s within
// 0.2 s. Heading hold alone could not help because it was off while the stick turned. In
// reverse the plant also turns several times harder per percent of differential, so the
// feedback gain goes up and the feedforward down as reverse throttle rises.
//
// Units: headings in degrees [0, 360) growing clockwise like the BNO055 Euler heading, rates in
// degrees per second clockwise-positive, outputs in percent of drive command. A positive output
// speeds up the left wheel and slows the right, which turns the robot clockwise.
//
// Defaults come from the closed-loop sim (auto_battlebot/mujoco_sim), whose plant has not been
// fit to recordings yet; the Python mirror in auto_battlebot/mujoco_sim/firmware.py must match.
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
};

class YawController {
   public:
    explicit YawController(const Config &config);

    // Hold `heading` from now on: clears the integral and any turn in progress.
    void reset(float heading);

    // One BNO055 sample. turn_percent is the turn stick (-100..100), dt the seconds since the
    // previous sample, reverse 0 (forward or stopped) to 1 (full reverse throttle). Returns
    // the differential in percent, clamped to +-100.
    float update(float turn_percent, float heading, float rate, float dt, float reverse);

    float output() const { return _output; }
    float rate_command() const { return _rate_command; }
    float setpoint() const { return _setpoint; }
    bool capturing() const { return _capturing; }

   private:
    Config _config;
    float _setpoint = 0.0f;
    bool _capturing = false;
    float _capture_time = 0.0f;
    float _integral = 0.0f;
    float _output = 0.0f;
    float _rate_command = 0.0f;
};

// Wraps an angle difference to [-180, 180).
float wrap_degrees(float angle);
}  // namespace yaw_control
