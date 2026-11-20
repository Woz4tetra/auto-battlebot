# 2026-10-24 mk2 esc floor step

Stand sweeps on 2026-10-24: both ESCs, both directions, run every step from 1 to 65 at the speed of step 65 and respond linearly from 66 up. No hysteresis. Unchanged by sine mode, rampup power or PWM frequency.

On the robot, the ESC's eRPM reply bits run at 95 ticks, 11% fast. At 25% fast a 3-bit run rounds down against the nominal bit and fails. Measuring the bit recovers it. The test is `test_erpm_decode`.

The design is in `docs/reference/robot_link/esc_drive.md`.
