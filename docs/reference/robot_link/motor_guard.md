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

