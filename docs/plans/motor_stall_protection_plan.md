# Mr Stabs Mk2 motor stall protection

## Goal

Keep a jammed or stalled drive motor from cooking itself, without costing pushing force or
acceleration in normal driving. The fix is firmware only and uses two sensors the robot already
has:

- The Mk2 board's INA238 measures pack current every 10 ms, and nothing acts on it.
- The BLHeli_32 ESCs (rev 32.90) can report eRPM back on the DShot wire (bidirectional DShot).
  The DShot pads already connect each ESC to its own GPIO (IO8 left, IO9 right, through 100 ohm),
  so no board change is needed. The driver is the missing piece.

The shunt measures current but sums both motors after the ESCs' duty conversion. eRPM measures
each motor's speed but not its current. The guard uses eRPM to split the current between motors
and the shunt to set its magnitude, and either one alone still protects the motors.

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
| ESC DShot | DShot300, 3D mode, one frame per control loop, RMT TX channels 3 (left) and 2 (right) | `main.cpp`, `esc.cpp` |
| Motor pole pairs P | 7 assumed (14 poles, typical 1806); not recorded anywhere | step 0.6 measures it |
| eRPM floor | about 917 eRPM (longest period the 12-bit reply encodes, 65408 us); slower reads as 0 | bidirectional DShot spec |
| eRPM floor at the wheel, P = 7 | 917 / 7 = 131 motor RPM = 13.7 rad/s motor, 0.61 rad/s wheel, 1.5 cm/s | derived |

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

eRPM supplies the missing unknowns: with `w_k` measured, the first equation gives each `I_k`
directly, and the second becomes a check on it.

`V` in these equations is the INA238's VBUS at the board, not the open-circuit pack voltage, so
pack sag drops out. Use the measured `vbat` every sample.

### 2. Estimating each motor's current

#### 2a. Speed from eRPM

```
w_k = sigma_k x 2 pi x eRPM_k / (60 P)          rad/s at the motor shaft
```

eRPM has no sign, so `sigma_k` in {-1, +1} is tracked separately (2c). A reading of 0 means below
the floor, about 13.7 rad/s at the motor, which is under 1% of free speed and changes `I_k` by
`Ke x 13.7 / R = 0.13 A`. Treat it as 0.

A reading is valid if its GCR decode and CRC pass. If a motor has no valid reading for 50 ms, its
eRPM is invalid and that motor drops to the shunt-only estimate (2e).

#### 2b. Model current, corrected by the shunt

```
R_k       = R_cold (1 + alpha dT_k)                 winding warms with the thermal model's rise
I_model_k = (d_k V - Ke w_k) / R_k
I_pred    = sum_k d_k I_model_k + I_logic           I_logic about 0.3 A
```

The model's errors are in `R`: `R_cold` is an estimate, the lumped `dT_k` lags the real winding,
and ESC FET and wiring losses are not in the buck power balance. All of them scale both motors'
currents the same way, so one shared gain `g` absorbs them:

```
if sum_k d_k I_model_k > 2 A:                       only when the motors draw enough to measure
    g_raw = (I_pack - I_logic) / sum_k d_k I_model_k
    g    <- g + (1 - exp(-dt / 0.5 s)) (g_raw - g)  low pass, 0.5 s
g = clamp(g, 0.8, 1.5)
I_hat_k = g I_model_k
```

eRPM sets the split between the motors and the shunt sets the total. The gate at 2 A keeps `g`
from chasing noise at idle and during regen, where the sum is near zero or negative. `g` holds its
last value while gated, and starts at 1.

The lower clamp bounds the damage of a wrong `g`. At 0.8, the guard underestimates heat by at most
36% (`0.8^2 = 0.64`). A healthy fit sits near 1. Telemetry reports `g`, and a `g` that stays
outside 0.9 to 1.25 for a whole session means a constant is wrong (`R_cold`, `P`, or the duty
curve), which step 0 should catch first.

#### 2c. Direction of rotation

The motor cannot reverse without passing through zero, so `sigma_k` only changes there:

```
if eRPM_k reads 0 and |d_k| > 0:   sigma_k <- sign(d_k)
otherwise:                         sigma_k holds
```

This depends on the ESC reporting a 0 while the motor reverses. The rotor alone crosses the
+-13.7 rad/s floor in well under 1 ms at stall torque, faster than one frame per loop. BLHeli's 3D
reversal brakes to a stop and restarts, which should hold the reading at 0 for longer, but that is
unmeasured. Step 0.6 checks it on a full-stick reversal.

If step 0.6 shows the 0 can be missed, add the shunt as a tiebreak: after a reversal command,
assume the motor still spins the old way (plugging, the high-current case) until either a 0 reads
or that assumption predicts `I_pred` above twice the measured `I_pack` for two INA238 samples in a
row. Assuming plugging overestimates for a few tens of ms, which errs safe.

#### 2d. Stall flag

```
stalled_k = |d_k| > 0.3 and eRPM_k == 0, held for 100 ms
```

The flag goes to telemetry and the radio so the pilot knows which wheel is jammed. It does not cut
power: heat comes from current over time, and the thermal model already acts on that. A wheel
pinned at 30% duty for 20 s is safe, and the same wheel at full stick is not.

#### 2e. Shunt-only fallback

When a motor's eRPM is invalid, its current falls back to two upper bounds. They hold whenever
motor k is motoring (drawing power from the pack, `d_k I_k >= 0`):

```
(a) measured bound:  |I_k| <= I_pack / |d_k|          since d_k I_k is one non-negative term of I_pack
(b) model bound:     |I_k| <= |d_k| V / R_cold         since back-EMF only reduces current while
                                                       the shaft turns the commanded way
```

The estimate is the tighter of the two:

```
I_hat_k = min(I_pack / max(|d_k|, 0.05), |d_k| V / R_cold)
```

If only one motor's eRPM is valid, subtract that motor's `d_k I_hat_k` from `I_pack` before
applying (a) to the other.

How the fused estimate (2b) and the fallback compare in the cases that matter:

| Case | Fused, eRPM + shunt | Fallback (a) | Fallback (b) | Fallback estimate |
| --- | --- | --- | --- | --- |
| Both stalled, equal d | exact | 2 I (sum of both) | I | I, exact |
| One stalled, other free, equal d | exact for each motor | about I_stalled | I_stalled for both | I_stalled for both: exact for the stalled one, high for the free one |
| Both free-running, straight | exact | small | stall value | small, about right |
| One motor regenerating (throttle dropped below speed) | exact | low | high | low: misses the regen current |
| Plugging (full reverse at speed) | exact if 2c tracks the sign | high | low | low: misses up to 2x stall current |

The fallback's bounds in detail:

| Case | (a) | (b) | Estimate |
| --- | --- | --- | --- |
| Both stalled, equal d | 2 I (sum of both) | I | I, exact |
| One stalled, other free, equal d | about I_stalled | I_stalled for both | I_stalled for both: exact for the stalled one, high for the free one |
| Both free-running, straight | small | stall value | small, about right |
| One motor regenerating (throttle dropped below speed) | low | high | low: misses the regen current |
| Plugging (full reverse at speed) | high | low | low: misses up to 2x stall current |

The fallback's two misses are transient. Once traction breaks, the rotor's inertia sets the timescale: the
motor time constant is `J_r R / Kt^2 = 9.3e-7 x 0.43 / 1.72e-5 = 23 ms`, and the robot's mass only
couples in through traction, which is about 1 g. A full stop from 2 m/s dissipates the robot's
kinetic energy, about 4 J total with the reflected rotor mass included, which heats a motor by well
under 1 C. Sustained heat comes from stalls and jams, and the estimator is exact or conservative
there.

`R_cold` must be the lowest resistance the circuit can have (cold, effective), so bound (b) stays
an upper bound and the fused model starts conservative. Step 0 measures it.

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

Each sensor covers for the other, so the guard only turns off when both are gone:

| Sensors | Mode | Current estimate | Notes |
| --- | --- | --- | --- |
| eRPM and INA238 | `FUSED` | 2b, `g` updating | normal |
| INA238 only | `SHUNT_ONLY` | 2e bounds, per motor that lost eRPM | misses regen and plugging, both transient |
| eRPM only | `ERPM_ONLY` | 2b with `g` frozen at its last value | `V` comes from the last good `vbat`, then 15.2 V after 1 s |
| Neither | `OFF` | none, commands pass through | dashboard and radio show it |

`ERPM_ONLY` stays useful because measured speed tells a free-running motor from a stalled one.
The model bound alone, without speed, would treat every full-throttle moment as a stall and fold a
free-running robot back after about 1.6 s, which is worse in a match than no guard. That is why the
guard only turns off when it has neither sensor.

Board 1 logged 9096 INA238 reads with 0 failures, so a missing INA238 is a fault state. The eRPM
valid rate is unknown until step 0.6. The dashboard shows the mode and the eRPM CRC error count,
and a pre-match check catches anything other than `FUSED`.

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

Item 6 needs the bidirectional DShot driver (see "Bidirectional DShot" under firmware changes)
and should run before item 4, so the thermal fit uses the fused `I_hat`.

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
6. **eRPM.** Flash the bidirectional driver with the guard not yet wired, and log eRPM for both
   motors in the dashboard CSV.
   - **Valid rate.** Drive normally for 2 minutes. Count valid replies against frames sent, per
     motor. Above 95% is fine. Lower than that points at signal integrity on the 100 ohm pads or the
     receive timing.
   - **Pole pairs `P`.** Wheels off the ground, hold the stick steady at about 30%, and count wheel
     revolutions over 10 s on video. `P = eRPM / (22.6 x wheel RPM)`. Expect 7.
   - **Free-speed check.** At full stick in the air, `w = 2 pi eRPM / (60 P)` should land near
     `d V / Ke`, less friction. A large gap means `P` or the gear ratio is wrong.
   - **Locked rotor.** Clamp a wheel and repeat item 3's stick steps. eRPM should read 0, or jitter
     near the floor while BLHeli forces commutation. Record what it reads so the 2d stall test
     matches it.
   - **Reversal.** Wheels on the floor, full forward to full reverse and back, ten times. Check that
     eRPM reads 0 for at least one frame each time. If it ever misses, implement the shunt tiebreak
     in 2c.
   - **Fused gain.** Replay 2b over items 3 and 5. `g` should settle between 0.9 and 1.25. Outside
     that, fix `R_cold`, `P`, or the duty curve before the thermal fit.

`playground/motor_guard/replay_guard.py` (new, one CLI) does the arithmetic for items 1, 4, 5, and
6. It takes a dashboard CSV, reconstructs `I_hat_k` the way the firmware would (fused when the CSV
has eRPM columns, shunt-only otherwise, so the 2026-10-21 CSV still works), runs the thermal model
open loop, and prints `R_eff`, `P`, `g`, the would-be `dT` trace and scale, and a plot. It mirrors
the C++ math with the same constants as CLI defaults. It is a tuning tool, not a golden fixture:
the native test below is the source of truth.

## Firmware changes

### Bidirectional DShot: `firmware/mr_stabs_mk2/lib/DShot/`

`DShot.cpp` uses the legacy IDF 4.4 RMT driver, transmit only. The platform stays pinned at
`espressif32@6.13.0` for that driver; porting to IDF 5 is a separate job.

- **Frame.** Bidirectional DShot inverts the line (idle high) and the checksum
  (`~(v ^ v >> 4 ^ v >> 8) & 0xF`). BLHeli_32 detects the inverted signal and switches modes on its
  own, with no ESC setting. Set the TX idle level high and swap the bit timings' high and low
  halves.
- **Pin.** Open-drain with input enabled and the internal pull-up. Route the TX channel's output
  and an RX channel's input to the same GPIO through the GPIO matrix. The S3 has 4 RX-only RMT
  channels (4 to 7) that nothing uses today; take 4 and 5. The NeoPixel takes a TX channel, but
  `rmt_reserved_channels` already keeps it off ours.
- **Reply.** About 30 us after the frame, the ESC sends 21 bits at 5/4 of the DShot bit rate
  (375 kbit/s at DShot300, 2.67 us per bit), GCR-encoded. Decode RMT run lengths to bits, undo the
  NRZI, map 5-bit GCR groups to 4-bit nibbles, and check the 4-bit CRC. The 12-bit payload is a
  period `m << e` in us (3-bit `e`, 9-bit `m`). `eRPM = 60e6 / period`, and the maximum period
  means stopped.
- **Timing.** Read the reply without blocking: arm RX after each `send()` and collect the result at
  the next `send()`, one loop later. That adds one loop of delay to `w_k`, well inside the 25 ms
  `T_d` the limiter already assumes.
- **API.** `DShot::read_erpm(uint32_t &erpm)` returns false on no reply or a bad CRC.
  `esc::Esc` adds `erpm()`, `erpm_valid()` (a valid reply within 50 ms), and `crc_errors()`.

Put the GCR decode and CRC in `lib/DShot/erpm_decode.h` as pure C++, so the native env can test it
against hand-built pulse trains.

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
    float pole_pairs = 7.0f;        // P (step 0.6)
    float logic_amps = 0.3f;        // I_logic
    float gain_gate_amps = 2.0f;    // g only updates above this summed motor draw
    float gain_tau_s = 0.5f;
    float gain_min = 0.8f;
    float gain_max = 1.5f;
    float stall_duty = 0.3f;        // stall flag: |d| above this with eRPM 0
    float stall_hold_s = 0.1f;
};

enum class State : uint8_t { OFF, NOMINAL, HOT, LIMITING };
enum class Sensors : uint8_t { FUSED, SHUNT_ONLY, ERPM_ONLY, NONE };

class MotorGuard {
   public:
    explicit MotorGuard(const Config &config);
    // Call once per new INA238 sample. Duties are what the ESCs ran during that sample
    // (previous scaled commands, 0 inside the stop threshold), in [-1, 1]. eRPM is the latest
    // reading per motor, negative when invalid. NaN volts or amps means no INA238 sample.
    // With neither sensor the guard is OFF, passes commands through, and still cools the model.
    void update(float dt_s, float volts, float amps, float left_duty, float right_duty,
                float left_erpm, float right_erpm);
    // Call every control loop. Scales both percent commands by the held scale.
    void apply(float &left_percent, float &right_percent) const;
    float scale() const;
    float rise_c(int motor) const;      // 0 left, 1 right
    float amps_estimate(int motor) const;
    float amps_allowed(int motor) const;
    float speed_rad_s(int motor) const; // signed, motor shaft
    bool stalled(int motor) const;
    float gain() const;                 // g
    State state() const;
    Sensors sensors() const;
    // Restores heat after a warm reset (RTC_NOINIT values); clamps to [0, rise_max_c * 2].
    void restore(float left_rise_c, float right_rise_c);
};

}
```

`State` meanings: `HOT` is `dT` past `rise_soft_c` on either motor with `s = 1`; `LIMITING` is
`s < 1`; `OFF` is `Sensors::NONE`.

The guard updates at the INA238's 10 ms rate in every mode, including `ERPM_ONLY` (it then runs on
the loop clock with NaN amps). The eRPM arrives every loop, faster than that, but the thermal model
and limiter gain nothing from sampling faster than the 25 ms delay already in the loop.

Constants live in `Config` defaults and are set once from step 0. No dashboard tunables: the robot
has one owner and one correct value per constant.

### `include/vbat_sensor.h`

`get_status().reads` already counts successful reads. `main.cpp` compares it to the last count it
saw to detect a new sample. No change needed beyond that.

### `src/main.cpp`

1. Construct `motor_guard::MotorGuard` next to `vbat_sensor_ina238`. Restore from the
   `RTC_NOINIT_ATTR` struct in `setup()` when its magic word matches.
2. After `vbat_sensor_ina238.update()`, on every loop, armed or not: if `reads` advanced, or 10 ms
   passed with no new read, call `guard.update()` with `dt` since the previous call, `vbat` and
   `ibat` (NaN when no new read), the duties last written (`last_left_duty`, `last_right_duty`,
   zeroed by every `stop_escs()` path), and each ESC's `erpm()` (-1 when `!erpm_valid()`). Write the
   two rises to the RTC struct.
3. In the armed path, between `mix_motor_outputs()` and the `esc->write()` calls (lines 547 and
   548), call `guard.apply(left_command, right_command)`. Record the scaled commands as the next
   duties: `|cmd| < stop_threshold` gives 0, else `cmd / 100` (or the step 0.3 curve).
4. The ESP-NOW OTA early return and the disarmed and failsafe paths already call `stop_escs()`; set
   the stored duties to 0 there, so the model cools while the board is idle. The OTA path returns
   before the INA238 update today. Move `vbat_sensor_ina238.update()` and the guard update above
   that return.
5. `stop()` still sends DShot frames, so eRPM keeps arriving while disarmed. A wheel turned by hand
   or coasting shows up as speed with zero duty, which is correct.

Placement after the mixer means the guard covers both the trainer and ESP-NOW command sources.

The yaw loop runs upstream of the guard. During a straight jam the heading holds, so its error and
integrator stay near zero. A jam while turning hard could wind up the integrator against the
scaled differential. I am leaving `yaw_control` alone in v1, and step 5 checks it; if it winds up,
add a hold input to `YawController::update` gated on `guard.state() == LIMITING`.

### Diagnostics and telemetry

- `include/diagnostics_server.h` `diag_data_t`: add
  - `float motor_rise_left`, `float motor_rise_right` (C)
  - `float motor_speed_left`, `float motor_speed_right` (rad/s at the motor, signed)
  - `float motor_amps_left`, `float motor_amps_right` (`I_hat`)
  - `float guard_scale`, `float guard_gain` (`g`)
  - `uint8_t guard_state`, `uint8_t guard_sensors`, `uint8_t stall_flags` (bit 0 left, bit 1 right)
  - `uint16_t erpm_crc_errors` (both ESCs summed)

  Fill them in all three `diag` literals in `main.cpp` (failsafe, disarmed, armed).
- `firmware/shared/espnow_protocol/src/espnow_protocol.h` `TelemetryPayload`: add the same fields
  (8 floats, 3 bytes, one uint16). 118 B grows to 155 B before padding, under `AIR_MAX_PAYLOAD`
  (245 B). Update the `static_assert(sizeof(TelemetryPayload) == ...)` and bump
  `TELEMETRY_BATCH_VERSION` to 2.
- Jetson decoder `src/esp32_diagnostics/esp32_telemetry_batch.cpp` rejects any `record_size`
  mismatch, so update it in the same commit: copy the new fields into its entry struct and whatever
  Foxglove channel it writes. Update `tests/esp32_diagnostics/test_esp32_wifi_telemetry.cpp` and
  `tests/test_espnow_transmitter.cpp` fixtures.
- `web/app.js` and `web/index.html`: a motor heat panel showing both rises against `rise_max_c`,
  each motor's speed and `I_hat`, a `STALL` badge per wheel, the scale as a percent, `g`, the
  state, and the sensor mode. Anything other than `FUSED` shows in amber, and `OFF` in red.
  Append the new CSV columns after `yaw_rate_cmd` and before `seq`. Grep for readers that index
  CSV columns by number (the README cites columns 21 and 22) and switch them to names.
- `README.md`: a "Stall protection" section with the thresholds, what each state means, and how to
  read the panel.

### Radio telemetry (optional, after step 5)

The pilot needs to know the robot is folded back, or a weak push reads as a broken robot. Send a
CRSF flight mode frame (type 0x21) carrying `OK`, `STALL L`, `STALL R`, `HOT`, `LIMIT`, or
`NOSENSE` (any mode other than `FUSED`) from
`CrsfBridge::send_telemetry`. EdgeTX can announce a change in `FM`. Check whether AlfredoCRSF has
a helper for 0x21; otherwise pack the frame by hand.

## Tests

New `firmware/mr_stabs_mk2/test/test_motor_guard/test_motor_guard.cpp`, run with
`pio test -e native`. The test drives a simulated plant: per-motor
`I_k = (d_k V - Ke w_k) / R`, `I_pack = sum d_k I_k`, with a two-sample delay between command and
measurement. The plant reports eRPM as `|w_k|` quantised the way the ESC reports it, 0 below the
floor, one loop late. Its `R` is set 15% above `R_cold` so `g` has something to correct.

Unless a case names a mode, it runs `FUSED`.

- Cold and lightly loaded: scale stays exactly 1.0, commands pass through bit-for-bit.
- Dual stall at full command: soft limit and `dT_max` times match the closed-form `t` from section 4
  within 5%; after that, `dT` never exceeds `rise_max_c + 1 C` over 60 s.
- Limiter settles to `I_allow` with under 10% overshoot and no sustained oscillation with the delay
  in the loop.
- Single stall with the other wheel free: each motor's estimate is within 5% of its true current
  in `FUSED`. In `SHUNT_ONLY`, the stalled motor's estimate is within 10%.
- `g` converges to the plant's `R` ratio within 5% in 2 s of driving, and holds its value through
  idle and regen.
- Regen and plugging: estimates within 10% of true current in `FUSED`, including a full-stick
  reversal at speed (sign tracking through the 0 reading).
- Stall flag: sets after 100 ms of locked rotor above 30% duty, clears when the wheel turns, never
  sets during a launch from rest.
- Cooling: from `rise_max_c` at zero current, `dT` follows `tau ln(dT_a / dT_b)` within 2%.
- Recovery: after the jam clears, scale returns to 1.0 no faster than `recover_per_s` allows.
- eRPM goes invalid on one motor mid-stall: that motor drops to the shunt bounds, the other stays
  fused, `dT` does not jump.
- NaN amps with eRPM valid: mode `ERPM_ONLY`, a full-throttle free run never limits, a dual stall
  still limits within 30% of the fused time.
- NaN amps and no eRPM: state `OFF`, scale 1.0, model still cools.
- `erpm_decode.h`: decodes hand-built GCR pulse trains for 0, the floor, and full speed; rejects a
  flipped bit through the CRC.
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
5. Unplug the INA238 cable: `ERPM_ONLY` on the dashboard, robot drives normally, and a locked
   wheel at full stick still limits.
6. Trigger a watchdog reset while warm (`esp_restart()` from the dashboard or a deliberate hang)
   and check that the rises survive. Then try a brownout, by dragging the pack down on a bench
   supply, and record whether RTC memory survives it.
7. Lock one wheel and drive the other: only the locked side shows `STALL`, and its `I_hat` matches
   `I_pack / d` within 10% while the free side reads small.
8. Over the 5-minute session in item 4, `g` stays between 0.9 and 1.25 and the mode stays `FUSED`.

`SHUNT_ONLY` cannot be forced on the robot without a debug switch, and the robot has one owner and
no tunables, so the native tests cover it.

## Later: eRPM in the robot filter

Out of scope for the guard, but the eRPM telemetry it adds is also a better velocity source for our
robot's filter than stick commands. Today `KalmanMotionEstimator` dead-reckons our robot through
camera dropouts on the stick command (`plant_stick_to_body_velocity`, no dynamics). That goes
wrong when the robot is pinned (it predicts up to 1.6 m of travel over the 0.4 s max coast at about
4 m/s free speed), on launches (traction caps acceleration at about 1 g), and wherever the
firmware changes the output after the sticks (heading hold, yaw loop, upside-down inversion, this
guard). eRPM misses wheel slip the same way commands do, and it arrives late.

In the EKF arm, commands stay the predict-step input to the plant model, and eRPM becomes a
wheel-speed measurement in the update step. Order:

1. Land bidirectional DShot and pass step 0.6.
2. Measure the eRPM latency from the robot to the Jetson through the ESP-NOW telemetry ring.
3. Record a few matches on the Jetson with telemetry in the MCAP. SVO replay cannot score this,
   since replay commands come from replay nav, not the driver.
4. A/B command dead-reckoning against eRPM speed plus gyro yaw rate, binned by our-robot dropout
   length. Restore `playground/control_stage0/prediction_eval.py` from `5af3d75` and add an eRPM
   arm.
5. If eRPM wins, use it for dropout prediction now, and add it as a measurement when the EKF arm's
   plant model passes its fit.

## Open questions

- How was the robot stalled on 2026-10-21: both wheels held, or one? Step 0.1 answers it from the
  CSV.
- Which motor smoked, and does it still measure the same winding resistance as the other one?
  Shorted turns would show up as a lower `R` and justify replacing it before the next match.
- Is the ESC really the Flycolor Raptor 5 layout running a ReadyToSky 35 A ESC (the comment in
  actuator.py)? Its continuous rating caps `I_peak` too.
- Does BLHeli_32 32.90 run bidirectional DShot together with 3D mode on this layout? It should, but
  step 0.6 is the first time it runs on this robot.
- Does the ESC report eRPM 0 during a 3D reversal (2c)? Step 0.6 answers it.
- Is the internal pull-up enough for the reply edge through the 100 ohm resistor and the ESC lead,
  or does the line need an external pull-up? Step 0.6's valid rate answers it.

## Next steps

1. Pull the 2026-10-21 stall CSV and run step 0.1.
2. Measure winding resistance on both motors (step 0.2) and compare them.
3. Write bidirectional DShot in `lib/DShot` with `erpm_decode.h` and its native test, then run
   step 0.6.
4. Run the duty curve and thermocouple tests (steps 0.3 and 0.4) and set `k`, `tau`, `rise_max_c`.
5. Write `lib/motor_guard` and its native test, then wire `main.cpp`.
6. Update the payload, Jetson decoder, and dashboard in one commit.
7. Bench and floor validation (step 5).
