# Motor guard

`firmware/mr_stabs_mk2/lib/motor_guard` limits drive commands from a thermal and current model of the two drive motors.

## update() contract

- Call once per new INA238 sample, or every 10 ms without one.
- `left_duty` and `right_duty` are what the ESCs ran during that sample: the previous scaled commands, 0 inside the stop threshold, in [-1, 1].
- `left_erpm` and `right_erpm` are the latest reading per motor, negative when invalid.
- NaN `volts` or `amps` means no INA238 sample.
- With no sensor for a motor the guard is OFF: it passes commands through and still cools the model.
- `rise_c()` and the other per-motor accessors index with `LEFT_MOTOR` (0) and `RIGHT_MOTOR` (1).
