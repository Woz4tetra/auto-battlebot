#pragma once
#include <cstdint>

// Keeps a stalled drive motor from cooking itself. Estimates each motor's winding current from
// eRPM and the INA238 pack current, runs a one-node thermal model per motor, and scales both drive
// commands down when a motor's current exceeds what its temperature allows.
// docs/plans/motor_stall_protection_plan.md has the derivation; section numbers below refer to it.
namespace motor_guard {

// Step 0 of the plan was deferred, so every value here is a placeholder chosen to err toward
// limiting early: R_cold below the MuJoCo fit (overestimates current) and dT_max 20 C under the
// plan's 70 C.
struct Config {
    float r_cold_ohm = 0.40f;          // effective circuit resistance, cold (fit says 0.453)
    float ke_v_s_per_rad = 4.152e-3f;  // back-EMF constant, 2300 KV
    float k_c_per_a2s = 0.036f;        // thermal gain
    float tau_s = 40.0f;               // thermal time constant
    float alpha_per_c = 0.00393f;      // copper tempco
    float rise_soft_c = 35.0f;         // limiting starts folding back here
    float rise_max_c = 50.0f;          // and holds the motor here
    float peak_amps = 15.0f;           // I_peak, allowed below rise_soft_c
    float loop_rad_s = 10.0f;          // k_c, limiter crossover
    float recover_per_s = 3.0f;        // r_up, in ln(scale) per second
    float min_scale = 0.05f;
    float min_duty = 0.05f;               // floor on |d| in the measured bound
    float max_dt_s = 0.1f;                // clamp on dt after a stall in the loop
    float pole_pairs = 7.0f;              // P
    float logic_amps = 0.3f;              // I_logic
    float gain_gate_amps = 2.0f;          // g only updates above this summed motor draw
    float reversal_margin_rad_s = 50.0f;  // speed rise that proves a reversed motor crossed 0
    float gain_steady_rad_s = 150.0f;
    float gain_steady_duty = 0.02f;  // and while neither duty moved more, two updates running //
                                     // and while neither motor's speed moved more per update
    float gain_min_load = 0.4f;      // and above this fraction of the draw with no back-EMF
    float gain_tau_s = 0.5f;
    // g only raises the model. Over-estimates (a high R, or BLHeli holding duty back while it
    // restarts a motor) are capped by the shunt bound instead: a duty clamp drove g to 0.8 on
    // the 2026-10-23 bench run, which would have under-read every later stall.
    float gain_min = 1.0f;
    float gain_max = 1.5f;
    float stall_duty = 0.3f;  // stall flag: |d| above this with eRPM 0
    float stall_hold_s = 0.1f;
    float fallback_volts = 15.2f;  // V once the last INA238 voltage is volts_hold_s old
    float volts_hold_s = 1.0f;
    float min_amps = 0.1f;  // estimates below this do not drive the limiter
    // The limiter acts on |I_hat| low-passed at this time constant. Launches and full-stick
    // reversals draw stall to plugging current (up to about 47 A) for about 30 ms while back-EMF
    // builds and eRPM catches up a loop late. Acting on the raw estimate cut a full-stick launch
    // to 76% on the 2026-10-23 bench run. At 0.1 s a dual stall at full stick still crosses
    // I_peak within about 60 ms. The thermal model integrates the raw estimate either way.
    float limit_filter_s = 0.1f;
};

enum class State : uint8_t { OFF, NOMINAL, HOT, LIMITING };
enum class Sensors : uint8_t { FUSED, SHUNT_ONLY, ERPM_ONLY, NONE };

const int NUM_MOTORS = 2;

class MotorGuard {
   public:
    explicit MotorGuard(const Config &config);
    // Call once per new INA238 sample, or every 10 ms without one. Duties are what the ESCs ran
    // during that sample (previous scaled commands, 0 inside the stop threshold), in [-1, 1].
    // eRPM is the latest reading per motor, negative when invalid. NaN volts or amps means no
    // INA238 sample. With no sensor for a motor the guard is OFF, passes commands through, and
    // still cools the model.
    void update(float dt_s, float volts, float amps, float left_duty, float right_duty,
                float left_erpm, float right_erpm);
    // Call every control loop. Scales both percent commands by the held scale.
    void apply(float &left_percent, float &right_percent) const;
    float scale() const { return _scale; }
    float rise_c(int motor) const { return _rise_c[motor]; }  // 0 left, 1 right
    float amps_estimate(int motor) const { return _amps[motor]; }
    float amps_allowed(int motor) const { return _allowed[motor]; }
    float speed_rad_s(int motor) const { return _speed[motor]; }  // signed, motor shaft
    bool stalled(int motor) const { return _stalled[motor]; }
    float gain() const { return _gain; }  // g
    State state() const;
    Sensors sensors() const { return _sensors; }
    float continuous_amps() const { return _continuous_amps; }
    // Restores heat after a warm reset (RTC_NOINIT values); clamps to [0, rise_max_c * 2].
    void restore(float left_rise_c, float right_rise_c);

   private:
    Config _config;
    float _continuous_amps;  // I_cont: holds a motor at rise_max_c forever
    float _rise_c[NUM_MOTORS] = {0.0f, 0.0f};
    float _amps[NUM_MOTORS] = {0.0f, 0.0f};
    float _limit_amps[NUM_MOTORS] = {0.0f, 0.0f};  // |I_hat| low-passed for the limiter
    float _last_speed[NUM_MOTORS] = {0.0f, 0.0f};
    float _last_duty[NUM_MOTORS] = {0.0f, 0.0f};
    int _steady_samples = 0;  // updates in a row with speed and duty both steady
    float _allowed[NUM_MOTORS];
    float _speed[NUM_MOTORS] = {0.0f, 0.0f};
    float _sign[NUM_MOTORS] = {1.0f, 1.0f};  // sigma_k, direction of rotation
    float _last_erpm[NUM_MOTORS] = {0.0f, 0.0f};
    int _contradictions = 0;  // samples the shunt rejected the held sign
    bool _stalled[NUM_MOTORS] = {false, false};
    float _stall_s[NUM_MOTORS] = {0.0f, 0.0f};
    float _gain = 1.0f;
    float _log_scale = 0.0f;  // u = ln(scale)
    float _scale = 1.0f;
    float _volts = 0.0f;  // last INA238 voltage
    float _volts_age_s = 1e9f;
    Sensors _sensors = Sensors::NONE;

    void track_sign(int motor, float duty, float erpm);
    void update_limiter(float dt_s);
};

}  // namespace motor_guard
