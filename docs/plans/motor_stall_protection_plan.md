# Mr Stabs Mk2 motor stall protection

## Goal

Keep a jammed or stalled drive motor from cooking itself, without costing pushing force or
acceleration in normal driving. The fix is firmware only: the Mk2 board's INA238 already measures
pack current every 10 ms, and nothing acts on it.

## What happened, 2026-10-21

I held the robot in a stall for 5 s. The dashboard showed a 70 A peak pack current. Both ESCs and
the board survived. One motor smoked a little.

Nothing in the current setup limits motor current or motor temperature:

- The ESC's temperature protection (`Eep_Pgm_Temp_Prot_Enable=140`,
  `firmware/config/blheli/mr_stabs_mk2_esc.ini`) watches the ESC's own sensor, not the motor.
- Low RPM Power Protect (`Eep_Pgm_Enable_Power_Prot=1`) is on and still let 70 A through. It
  exists to keep startup in sync, not to limit heat.
- The Flycolor Raptor 5 layout has no current sensor, so BLHeli_32 current protection is not
  available.
- `main.cpp` reads `ibat` and only logs it.

## Known quantities

| Quantity | Value | Source |
| --- | --- | --- |
| Motor | Repeat Compact 1806, 2300 KV | `docs/plans/mujoco_warp_mr_stabs_plan.md` |
| Kt = Ke | 60 / (2 pi KV) = 4.152e-3 N m/A (V s/rad) | same |
| Gear ratio N | 22.6 | same |
| Gearbox efficiency eta | 0.8 | `auto_battlebot/mujoco_sim/actuator.py` default |
| Wheel radius r | 25 mm | CAD |
| Robot mass m | about 0.5 kg | mujoco plan |
| Reflected rotor inertia per wheel | N^2 J_r = 4.77e-4 kg m^2 | mujoco plan |
| Effective circuit resistance | 0.453 ohm | MuJoCo fit, `playground/calibration/out/mujoco_sysid/fit_2026:10:15_tag/fit.json` |
| Pack | 4S, 16.8 V full, about 15.2 V under load | `NOMINAL_PACK_V` in actuator.py |
| Shunt | 0.5 mohm, high side, ahead of both ESCs | `hardware/mr_stabs_mk2_pcb/pcba/design.py` |
| INA238 conversion | ADC_CONFIG 0xBB69: 1052 us bus + 1052 us shunt, 4 averages, 8.4 ms per result | `vbat_sensor.cpp` |
| INA238 sample period | 10 ms | `vbat_sensor.h` |
| INA238 ALERT pin | not connected (`ina["~{Alert}"] += NC`) | design.py |

The 70 A reading lines up with the MuJoCo fit. If both motors were stalled at full throttle, each
carried 35 A, which implies a circuit resistance of 15.2 V / 35 A = 0.43 ohm per motor. The fit
says 0.453 ohm. That resistance covers the winding, ESC FETs, and wiring, so the winding's own share
is smaller and unmeasured.

## The math

### 1. Motor current and pack current are not the same

Averaged over a PWM period, a BLHeli_32 ESC with synchronous rectification is a voltage source
`d V` behind the winding, where `d` is duty in [-1, 1] and `V` is the ESC supply voltage. For motor
k:

```
I_k = (d_k V - Ke w_k) / R          w_k = motor shaft speed, rad/s
```

The ESC is a buck converter, so by power balance (ESC losses ignored):

```
I_pack = sum_k d_k I_k  (+ about 0.3 A of logic and radio)
```

Two consequences:

- **At partial throttle, the motor carries more than the pack.** A stalled motor at `d = 0.5`
  draws `0.5 V / R` in its winding but only `0.25 V / R` from the pack. Winding heat goes as
  `I_k^2 R`, so a pack current limit alone under-protects at low duty.
- **The shunt sees both motors summed.** One equation, two unknown currents.

`V` in these equations is the INA238's VBUS at the board, not the open-circuit pack voltage, so
pack sag drops out. Use the measured `vbat` every sample.

### 2. Estimating each motor's current

Two upper bounds hold whenever motor k is motoring (drawing power from the pack, `d_k I_k >= 0`):

```
(a) measured bound:  |I_k| <= I_pack / |d_k|          since d_k I_k is one non-negative term of I_pack
(b) model bound:     |I_k| <= |d_k| V / R_cold         since back-EMF only reduces current while
                                                       the shaft turns the commanded way
```

The estimate is the tighter of the two:

```
I_hat_k = min(I_pack / max(|d_k|, 0.05), |d_k| V / R_cold)
```

How it behaves in the cases that matter:

| Case | (a) | (b) | Estimate |
| --- | --- | --- | --- |
| Both stalled, equal d | 2 I (sum of both) | I | I, exact |
| One stalled, other free, equal d | about I_stalled | I_stalled for both | I_stalled for both: exact for the stalled one, high for the free one |
| Both free-running, straight | small | stall value | small, about right |
| One motor regenerating (throttle dropped below speed) | low | high | low: misses the regen current |
| Plugging (full reverse at speed) | high | low | low: misses up to 2x stall current |

The two misses are transient. Once traction breaks, the rotor's inertia sets the timescale: the
motor time constant is `J_r R / Kt^2 = 9.3e-7 x 0.43 / 1.72e-5 = 23 ms`, and the robot's mass only
couples in through traction, which is about 1 g. A full stop from 2 m/s dissipates the robot's
kinetic energy, about 4 J total with the reflected rotor mass included, which heats a motor by well
under 1 C. Sustained heat comes from stalls and jams, and the estimator is exact or conservative
there.

`R_cold` must be the lowest resistance the circuit can have (cold, effective), so bound (b) stays
an upper bound. Step 0 measures it.

`d_k` is the duty the ESC applied for the current that was measured. Use the command written on the
previous INA238 sample, after the guard's scaling. A command inside `Esc::stop_threshold` sends
DShot stop, so its `d_k` is 0. Step 0 also checks whether duty is linear in the command during a
stall (see "Duty curve" there); if Low RPM Power Protect clamps duty, replace `d = cmd / 100` with
the measured curve.

### 3. How much current is useful

Most of the stall current produces no useful work, which is why a low cap is cheap.

Pushing force on flat ground is traction-limited at `mu m g`. At `mu = 1`, that is 4.9 N for the
whole robot, 2.45 N per wheel. The motor current that produces it:

```
I = F r / (N eta Kt) = 2.45 x 0.025 / (22.6 x 0.8 x 4.152e-3) = 0.82 A per motor
```

Acceleration at the traction limit (1 g) also has to spin up the reflected rotor inertia:

```
wheel torque per wheel = (N^2 J_r / r + m_share r) a
                       = (4.77e-4 / 0.025 + 0.25 x 0.025) x 9.81 = 0.248 N m
I = 0.248 / (N eta Kt) = 0.248 / 0.0751 = 3.3 A per motor
```

A full-throttle stall at 35 A per motor puts 105 N on each wheel, about 20x what traction allows. A
wheel only stalls at that current when something holds it mechanically: a jam, debris, a pin, or a
hand. A wedge under a 3 lb opponent adds at most about 13 N of normal force, which raises the useful
current to roughly 3 A per motor.

Launch spikes are higher. A wheel that breaks traction from rest briefly draws stall current until
back-EMF rises, about 23 ms. The limiter below runs at 10 rad/s and does not react to spikes that
short, so they pass through.

So a sustained per-motor cap `I_peak` around 15 A should cost nothing measurable. Step 0 sets the
real value from logs.

### 4. Thermal model

One thermal node per motor: winding plus stator, lumped. `dT` is the rise over ambient.

```
C d(dT)/dt = I^2 R(T) - dT / R_th
R(T) = R_ref (1 + alpha dT),  alpha = 0.00393 / C (copper)
```

Rewrite with `k = R_ref / C` (C per A^2 s) and `tau = R_th C` (s):

```
d(dT)/dt = k (1 + alpha dT) I^2 - dT / tau
```

Steady state at constant current, ignoring the alpha term:

```
dT_ss = k tau I^2
I_cont = sqrt(dT_max / (k tau))          the current the motor holds forever at dT_max
```

Time to reach a rise `dT` from cold at constant current:

```
t = -tau ln(1 - dT / (k tau I^2))
```

Cooling from `dT_a` to `dT_b` at zero current:

```
t = tau ln(dT_a / dT_b)
```

Discrete update, run once per INA238 sample (exact for constant current over `dt`, so a long or
irregular `dt` does not destabilise it):

```
a = exp(-dt / tau)
dT <- a dT + (1 - a) k tau (1 + alpha dT) I_hat^2
```

`k` and `R` only appear as the product `k I^2`. If the thermal fit in step 0 uses the same
effective `R` that sets `I_hat`, the model is self-consistent even though the effective `R`
includes ESC and wiring losses. I fit `k` and `tau` directly and never need `C` or the winding's
own resistance.

The lumped node underestimates the winding hot spot, which leads the stator by a few seconds in a
stall. `dT_max` carries that margin.

#### Provisional numbers

Placeholders until step 0 measures them. The stator is about 12.6 g (22.4 g motor minus the 9.8 g
rotor), of which roughly 3.5 g is copper:

```
C ~ 3.5 g x 0.385 J/(g K) + 9.1 g x 0.45 J/(g K) = 5.5 J/K
R_winding ~ 0.20 ohm (guess; typical for a 2300 KV 1806)
k ~ 0.20 / 5.5 = 0.036 C / (A^2 s)
tau ~ 40 s (guess; no airflow inside the chassis)
```

Check against the event: 35 A for 5 s gives `0.036 x 35^2 x 5 = 220 C` adiabatic rise. Enamel and
varnish start to smoke in the 150 to 250 C range, so "smoked a little" fits.

With `dT_soft = 50 C` and `dT_max = 70 C` (about 100 C absolute at 30 C ambient inside the robot):

| Quantity | Value |
| --- | --- |
| `I_cont` | sqrt(70 / (0.036 x 40)) = 7.0 A |
| Time to soft limit at a 35 A stall, from cold | 1.2 s |
| Time to `dT_max` at a 35 A stall, from cold | 1.6 s |
| Time to soft limit at the 15 A cap, from cold | 6.7 s |
| Time to `dT_max` at the 15 A cap, from cold | 9.7 s |
| Cooling 70 C to 50 C | 13 s |
| Cooling 70 C to 25 C | 41 s |

At `I_cont` = 7 A the motor still makes 21 N per wheel, about 8x what traction allows on flat
ground, so a fully folded-back robot pushes as hard as a healthy one. It only loses launch
acceleration above about 2 g, which traction caps at 1 g anyway.

### 5. Allowed current

Each motor's allowed current falls linearly from `I_peak` to `I_cont` between the soft and hard
temperature thresholds:

```
f_k = clamp((dT_k - dT_soft) / (dT_max - dT_soft), 0, 1)
I_allow_k = I_peak - (I_peak - I_cont) f_k
```

At `dT_max`, `I_allow = I_cont` holds the motor exactly at `dT_max`, a stable equilibrium. The
motor never runs above `I_cont` once it reaches `dT_max`, so it cannot climb further.

### 6. Limiter

One scale `s` in [s_min, 1] multiplies both commands. One shared scale keeps the left/right ratio,
so the robot keeps the curvature the pilot or yaw loop asked for. Scaling only the stalled wheel
would turn the robot.

Run the integrator on `u = ln s`. In a stall `I_hat` is proportional to `s`, so `ln I_hat = u +
const` and the loop gain is exactly 1 at every operating point:

```
e = max_k ln(I_hat_k / I_allow_k)          (I_hat_k below 0.1 A counts as e_k = -inf)
if e > 0:  u <- u - k_c e dt
else:      u <- u + min(k_c (-e), r_up) dt
u <- clamp(u, ln s_min, 0)
```

Loop transfer function `L = (k_c / s_laplace) exp(-s_laplace T_d)`, where `T_d` is the measurement
delay: up to 10 ms of sample hold, about 4 ms of averaging group delay, and one control loop. Call
it 25 ms worst case. Crossover is `k_c`, and phase margin is `90 deg - k_c T_d`:

| `k_c` | Phase margin at T_d = 25 ms | Time constant |
| --- | --- | --- |
| 10 rad/s | 76 deg | 100 ms |
| 20 rad/s | 61 deg | 50 ms |

`k_c = 10 rad/s` to start. When a motor is moving, `I_hat` grows less than proportionally with `s`,
so the loop gain falls below 1 and the loop gets slower, never less stable.

- `r_up = 3 /s` caps recovery so a sudden drop in current (the jam clears) brings power back over
  about 1 s instead of one step. From `s_min` to 1 is `ln(1/0.05) / 3 = 1.0 s`.
- `s_min = 0.05` keeps `u` from winding to minus infinity if the reading sticks high. At a full
  stick stall that is 0.05 x 35 A = 1.75 A, already under `I_cont`.

### 7. Sensor missing

If the INA238 is absent or failing, `ibat` is NaN. The only alternative is the model bound alone,
which treats every full-throttle moment as a stall and would fold a free-running robot back after
about 1.6 s. That is worse in a match than no guard, so **the guard passes commands through and
reports itself off**. Board 1 logged 9096 reads with 0 failures, so this is a fault state, not a
normal one. The dashboard and radio both show it, and a pre-match check catches it.

### 8. Reset behavior

- The heat state keeps updating while disarmed, in failsafe, and during OTA, with `I_hat = 0`, so
  the motor model cools. Disarming and re-arming must not hand back a fresh budget.
- At power-on, `dT = 0`.
- A watchdog or brownout reset mid-match would also zero it. Keep `dT_k` and a magic word in an
  `RTC_NOINIT_ATTR` struct so a software or brownout reset resumes the estimate; power-on
  initialises the magic and starts cold. Step 5 verifies it on hardware, since I have not confirmed
  that RTC memory survives a brownout reset on the S3.

## Step 0: measurements before writing code

All drive inputs come from the transmitter (sysid tools are human-driven only). Log with the
dashboard's CSV recording.

1. **The stall CSV from 2026-10-21.** During the stall, read `left_cmd`, `right_cmd`, `vbat`, and
   `ibat`. If both commands sat near 100%, `R_eff = vbat / (ibat / 2)`. That answers whether one or
   both motors were stalled, whether the 70 A held for 5 s or spiked and sagged, and gives
   `R_cold`.
2. **Winding resistance.** Unplug one motor from its ESC. Push 2 A from a current-limited bench
   supply through two phase leads and read the voltage at the motor leads with a DMM. `R = V / I`,
   phase to phase, cold. Repeat on the other motor. This separates the winding's share of `R_eff`.
3. **Duty curve.** Lock one wheel (clamp the wheel, not the hand). Step the stick by hand through
   about 10, 20, 40, 60% for about 0.3 s each with 3 s gaps. Expect `I_pack = d^2 V / R`. A
   shortfall at low command means Low RPM Power Protect or Rampup Power (`Eep_Pgm_Rampup_Pwr=50`)
   is clamping duty, so replace `d = cmd / 100` with the measured curve.
4. **Thermal constants.** Bond a K-type thermocouple to the stator (Kapton tape plus thermal paste)
   with the wheel locked. Hold a command that draws about 15 A per motor for 10 s, then release.
   Read the temperature every 5 s during heating and every 15 s for 3 minutes of cooling (a phone
   video of the meter is fine).
   - `tau` from the cooling curve: fit `dT(t) = dT_0 exp(-t / tau)`.
   - `k` from the heating slope at the start: `k = (d dT / dt) / I_hat^2`, using `I_hat` computed
     from the log exactly as the firmware will, so `k` absorbs whatever `R` the estimator uses.
   - Pick `dT_max` with margin for the winding hot spot: at most 70 C, lower if the stator reading
     lags the smell of hot varnish.
5. **Traction current and `I_peak`.** On the arena floor, drive into a wall and raise the stick
   slowly until the wheels spin. The highest `I_hat` before spin is the traction current. Then
   replay the estimator over a normal driving CSV and take the 99.5th percentile of per-motor
   `I_hat` over 100 ms windows. Set `I_peak` to about 1.5x the larger of the two, but no more than
   20 A.

`playground/motor_guard/replay_guard.py` (new, one CLI) does the arithmetic for items 1, 4, and
5. It takes a dashboard CSV, reconstructs `I_hat_k`, runs the thermal model open loop, and prints
`R_eff`, the would-be `dT` trace and scale, and a plot. It mirrors the C++ math with the same
constants as CLI defaults. It is a tuning tool, not a golden fixture: the native test below is the
source of truth.

## Firmware changes

### New library: `firmware/mr_stabs_mk2/lib/motor_guard/`

Pure C++ with `<cmath>` and no Arduino headers, like `lib/yaw_control/`, so the native env can test
it.

```cpp
namespace motor_guard {

struct Config {
    float r_cold_ohm = 0.43f;       // effective circuit resistance, cold (step 0.1)
    float k_c_per_a2s = 0.036f;     // thermal gain (step 0.4)
    float tau_s = 40.0f;            // thermal time constant (step 0.4)
    float alpha_per_c = 0.00393f;   // copper tempco
    float rise_soft_c = 50.0f;
    float rise_max_c = 70.0f;
    float peak_amps = 15.0f;        // I_peak (step 0.5)
    float loop_rad_s = 10.0f;       // k_c
    float recover_per_s = 3.0f;     // r_up, in ln(scale) per second
    float min_scale = 0.05f;
    float min_duty = 0.05f;         // floor on |d| in the measured bound
    float max_dt_s = 0.1f;          // clamp on dt after a stall in the loop
};

enum class State : uint8_t { OFF_NO_SENSOR, NOMINAL, HOT, LIMITING };

class MotorGuard {
   public:
    explicit MotorGuard(const Config &config);
    // Call once per new INA238 sample. Duties are what the ESCs ran during that sample
    // (previous scaled commands, 0 inside the stop threshold), in [-1, 1]. NaN volts or amps
    // puts the guard in OFF_NO_SENSOR, passes commands through, and still cools the model.
    void update(float dt_s, float volts, float amps, float left_duty, float right_duty);
    // Call every control loop. Scales both percent commands by the held scale.
    void apply(float &left_percent, float &right_percent) const;
    float scale() const;
    float rise_c(int motor) const;      // 0 left, 1 right
    float amps_estimate(int motor) const;
    float amps_allowed(int motor) const;
    State state() const;
    // Restores heat after a warm reset (RTC_NOINIT values); clamps to [0, rise_max_c * 2].
    void restore(float left_rise_c, float right_rise_c);
};

}
```

`State` meanings: `HOT` is `dT` past `rise_soft_c` on either motor with `s = 1`; `LIMITING` is
`s < 1`.

Constants live in `Config` defaults and are set once from step 0. No dashboard tunables: the robot
has one owner and one correct value per constant.

### `include/vbat_sensor.h`

`get_status().reads` already counts successful reads. `main.cpp` compares it to the last count it
saw to detect a new sample. No change needed beyond that.

### `src/main.cpp`

1. Construct `motor_guard::MotorGuard` next to `vbat_sensor_ina238`. Restore from the
   `RTC_NOINIT_ATTR` struct in `setup()` when its magic word matches.
2. After `vbat_sensor_ina238.update()`, on every loop, armed or not: if `reads` advanced, call
   `guard.update()` with `dt` since the previous sample, `vbat`, `ibat`, and the duties last written
   (`last_left_duty`, `last_right_duty`, zeroed by every `stop_escs()` path). Write the two rises to
   the RTC struct.
3. In the armed path, between `mix_motor_outputs()` and the `esc->write()` calls (lines 511 and
   512), call `guard.apply(left_command, right_command)`. Record the scaled commands as the next
   duties: `|cmd| < stop_threshold` gives 0, else `cmd / 100` (or the step 0.3 curve).
4. The ESP-NOW OTA early return and the disarmed and failsafe paths already call `stop_escs()`; set
   the stored duties to 0 there, so the model cools while the board is idle. The OTA path returns
   before the INA238 update today. Move `vbat_sensor_ina238.update()` and the guard update above
   that return.

Placement after the mixer means the guard covers both the trainer and ESP-NOW command sources.

The yaw loop runs upstream of the guard. During a straight jam the heading holds, so its error and
integrator stay near zero. A jam while turning hard could wind up the integrator against the
scaled differential. I am leaving `yaw_control` alone in v1, and step 5 checks it; if it winds up,
add a hold input to `YawController::update` gated on `guard.state() == LIMITING`.

### Diagnostics and telemetry

- `include/diagnostics_server.h` `diag_data_t`: add `float motor_rise_left`,
  `float motor_rise_right` (C), `float guard_scale`, `uint8_t guard_state`. Fill them in all three
  `diag` literals in `main.cpp` (failsafe, disarmed, armed).
- `firmware/shared/espnow_protocol/src/espnow_protocol.h` `TelemetryPayload`: add the same four
  fields. 118 B grows to 131 B, under `AIR_MAX_PAYLOAD` (245 B). Update the
  `static_assert(sizeof(TelemetryPayload) == ...)` and bump `TELEMETRY_BATCH_VERSION` to 2.
- Jetson decoder `src/esp32_diagnostics/esp32_telemetry_batch.cpp` rejects any `record_size`
  mismatch, so update it in the same commit: copy the new fields into its entry struct and whatever
  Foxglove channel it writes. Update `tests/esp32_diagnostics/test_esp32_wifi_telemetry.cpp` and
  `tests/test_espnow_transmitter.cpp` fixtures.
- `web/app.js` and `web/index.html`: a motor heat panel showing both rises against `rise_max_c`,
  the scale as a percent, and the state, with `NO SENSOR` in red for `OFF_NO_SENSOR`. Append the
  four CSV columns after `yaw_rate_cmd` and before `seq`. Grep for readers that index CSV columns
  by number (the README cites columns 21 and 22) and switch them to names.
- `README.md`: a "Stall protection" section with the thresholds, what each state means, and how to
  read the panel.

### Radio telemetry (optional, after step 5)

The pilot needs to know the robot is folded back, or a weak push reads as a broken robot. Send a
CRSF flight mode frame (type 0x21) carrying `OK`, `HOT`, `LIMIT`, or `NOSENSE` from
`CrsfBridge::send_telemetry`. EdgeTX can announce a change in `FM`. Check whether AlfredoCRSF has
a helper for 0x21; otherwise pack the frame by hand.

## Tests

New `firmware/mr_stabs_mk2/test/test_motor_guard/test_motor_guard.cpp`, run with
`pio test -e native`. The test drives a simulated plant: per-motor
`I_k = (d_k V - Ke w_k) / R`, `I_pack = sum d_k I_k`, with a two-sample delay between command and
measurement.

- Cold and lightly loaded: scale stays exactly 1.0, commands pass through bit-for-bit.
- Dual stall at full command: soft limit and `dT_max` times match the closed-form `t` from section 4
  within 5%; after that, `dT` never exceeds `rise_max_c + 1 C` over 60 s.
- Limiter settles to `I_allow` with under 10% overshoot and no sustained oscillation with the delay
  in the loop.
- Single stall with the other wheel free: the stalled motor's estimate is within 10% of its true
  current.
- Cooling: from `rise_max_c` at zero current, `dT` follows `tau ln(dT_a / dT_b)` within 2%.
- Recovery: after the jam clears, scale returns to 1.0 no faster than `recover_per_s` allows.
- NaN amps: state `OFF_NO_SENSOR`, scale 1.0, model still cools.
- A 2 s gap between updates is clamped by `max_dt_s` and does not jump the state.
- Zero and stop-threshold commands give `I_hat = 0`.
- `restore()` clamps out-of-range values.

## Step 5: bench and floor validation

1. Locked wheel, thermocouple on the stator, full stick for 30 s. Pass: the guard starts limiting
   within 30% of the predicted time, the stator stays under `30 C + rise_max_c` (ambient plus
   rise), and full power returns after cooling.
2. Same with the stick held at 30% to exercise the low-duty case where motor current exceeds pack
   current.
3. Jam while turning hard: watch `pid_output` and the yaw integrator for windup.
4. A normal 5-minute driving session with hard launches, reversals, and wall pushes. Pass: the
   state never reaches `LIMITING`.
5. Unplug the INA238 cable: `NO SENSOR` on the dashboard, robot drives normally.
6. Trigger a watchdog reset while warm (`esp_restart()` from the dashboard or a deliberate hang)
   and check that the rises survive. Then try a brownout, by dragging the pack down on a bench
   supply, and record whether RTC memory survives it.

## Later: bidirectional DShot

BLHeli_32 can return eRPM on the DShot line. With measured shaft speed per motor, the estimator
becomes exact: `I_k = (d_k V - Ke w_k) / R`, no apportioning, no blind spots during regen or
plugging, and a real stall flag (high command, zero speed). It means teaching
`lib/DShot/DShot.cpp` to switch the RMT channel to receive after each frame, and inverting the
signal. I would do it once v1 has run a few matches and shown whether the estimator's misses
matter.

## Open questions

- How was the robot stalled on 2026-10-21: both wheels held, or one? Step 0.1 answers it from the
  CSV.
- Which motor smoked, and does it still measure the same winding resistance as the other one?
  Shorted turns would show up as a lower `R` and justify replacing it before the next match.
- Is the ESC really the Flycolor Raptor 5 layout running a ReadyToSky 35 A ESC (the comment in
  actuator.py)? Its continuous rating caps `I_peak` too.

## Next steps

1. Pull the 2026-10-21 stall CSV and run step 0.1.
2. Measure winding resistance on both motors (step 0.2) and compare them.
3. Run the thermocouple test (step 0.4) and set `k`, `tau`, `rise_max_c`.
4. Write `lib/motor_guard` and its native test, then wire `main.cpp`.
5. Update the payload, Jetson decoder, and dashboard in one commit.
6. Bench and floor validation (step 5).
