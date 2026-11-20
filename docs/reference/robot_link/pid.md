# PID

`firmware/mr_stabs_mk2/lib/pid` is a PID controller with optional feedforward and integral zone.

## `update()` contract

- Call `update()` once per new measurement, with dt the time since the previous measurement.
- A repeated measurement makes the D term read zero (`Pid.DerivativeIsErrorChangeOverDt`).

## `pid::PidConfig`

| Field | Meaning |
| --- | --- |
| `kp` | Proportional gain. |
| `ki` | Integral gain, per second. |
| `kd` | Derivative gain, in seconds. |
| `kf` | Feedforward gain on the setpoint. |
| `i_zone` | The integral accumulates only while the error is inside this zone. -1 means the integral always accumulates. |
| `i_max` | Maximum magnitude of the integral term's output. 0 means no limit. |
| `tolerance` | Error band in which the controller counts as at the setpoint. Output is 0 inside it, but the integral and derivative state keep updating, so leaving the band does not kick. |
| `continuous` | Wrap the input as an angle. |
| `min_input` | Minimum input for continuous mode, typically -180. |
| `max_input` | Maximum input for continuous mode, typically 180. |
