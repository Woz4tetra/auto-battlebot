# Motor guard

`firmware/mr_stabs_mk2/lib/motor_guard` limits drive commands from a thermal and current model of the two drive motors.

## update() contract

- Call once per new INA238 sample, or every 10 ms without one.
- `left_duty` and `right_duty` are what the ESCs ran during that sample: the previous scaled commands, 0 inside the stop threshold, in [-1, 1].
- `left_erpm` and `right_erpm` are the latest reading per motor, negative when invalid.
- NaN `volts` or `amps` means no INA238 sample.
- With no sensor for a motor the guard is OFF: it passes commands through and still cools the model.
- `rise_c()` and the other per-motor accessors index with `LEFT_MOTOR` (0) and `RIGHT_MOTOR` (1).

## Current estimate

- The shunt sets the total current through one shared gain `g`. The gain update is gated so idle and regen, where the sum is near zero or negative, do not move it.
- It is also gated on load. Free running, the model current is the small difference of `d V` and `Ke w`, and friction, eRPM lag, and duty error swamp it.
- It is also gated on steady speed. The INA238 averages over 8.4 ms and eRPM lags a loop, so during a launch or reversal the shunt and the model describe different instants. The same applies to the duty: the INA238 sample spans up to the last two updates, and eRPM still reads the old speed on the sample a step lands in.
- `g` only raises the model. Over-estimates (a high R, or BLHeli holding duty back while it restarts a motor) are capped by the shunt bound instead.
- The measured bound (2e a) caps the model too. The model assumes the ESC applies the commanded duty, but BLHeli holds duty back while it restarts a motor at low RPM. The bound only holds while no motor regenerates, since regen adds a negative term to the pack current.

## Limiter filter

- The limiter acts on |I_hat| low-passed at a fixed time constant. Estimates below the limiter threshold do not drive it.
- Launches and full-stick reversals draw stall to plugging current (up to about 47 A) for about 30 ms while back-EMF builds and eRPM catches up a loop late.
- At the 0.1 s time constant, a dual stall at full stick still crosses I_peak within about 60 ms.
- The thermal model integrates the raw estimate either way.


## `motor_guard::Config`

`MotorGuard` reads these fields. Defaults are in `firmware/mr_stabs_mk2/lib/motor_guard/motor_guard.h`.

| Field | Meaning |
| --- | --- |
| `r_cold_ohm` | Effective circuit resistance, cold. Default 0.40. The MuJoCo fit says 0.453 (`docs/plans/motor_stall_protection_plan.md`). |
| `ke_v_s_per_rad` | Back-EMF constant, 2300 KV. |
| `k_c_per_a2s` | Thermal gain. |
| `tau_s` | Thermal time constant. |
| `alpha_per_c` | Copper tempco. |
| `rise_soft_c` | Limiting starts folding back at this temperature rise. |
| `rise_max_c` | Limiting holds the motor at this rise. |
| `peak_amps` | `I_peak`, allowed below `rise_soft_c`. |
| `loop_rad_s` | `k_c`, the limiter crossover. |
| `recover_per_s` | `r_up`, in ln(scale) per second. |
| `min_duty` | Floor on `|d|` in the measured bound. |
| `max_dt_s` | Clamp on dt after a stall in the loop. |
| `pole_pairs` | `P`. |
| `logic_amps` | `I_logic`. |
| `gain_gate_amps` | `g` only updates above this summed motor draw. |
| `reversal_margin_rad_s` | Speed rise that proves a reversed motor crossed 0. |
| `gain_steady_duty` | Largest per-update change in either motor's duty that still counts as steady. `g` adapts only after two steady updates in a row. |
| `gain_steady_rad_s` | Largest per-update change in either motor's speed that still counts as steady (same two-update rule). |
| `gain_min_load` | `g` updates only above this fraction of the draw with no back-EMF. |
| `stall_duty` | Stall flag: `|d|` above this with eRPM 0. |
| `fallback_volts` | Volts used once the last INA238 voltage is `volts_hold_s` old. |
