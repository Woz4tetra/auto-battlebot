# 2026-10-24 mk2 esc floor step

## Notes moved from code comments

### General

- `mr_stabs_mk2/include/esc.h`: Duty the ESC runs for a command: 0 inside stop_threshold, else esc_map's floor-offset duty, so 1% runs about 7%.
- `mr_stabs_mk2/lib/esc_map/esc_map.h`: steps above zero in each direction The ESCs run every step from 1 to 65 at the same speed, the speed of step 65, and respond linearly from 66 up: both ESCs, both directions, no hysteresis, unchanged by sine mode, rampup power or PWM frequency (stand sweeps, 2026-10-24). Commands start at 66 so the smallest one past stop_threshold already moves the motor.

### ESC eRPM reply clock

- `mr_stabs_mk2/lib/erpm_decode/erpm_decode.h`: The ESC's reply clock is its own: BLHeli_32 on Mr Stabs sends bits at 95 ticks against the nominal 107, which leaves a 3-bit run at 2.66 bits, close to rounding down. Measure the bit from the reply's single-bit runs (within 35% of nominal) and decode with that.
- `mr_stabs_mk2/test/test_erpm_decode/test_erpm_decode.cpp`: Recorded on the robot: the ESC's bits run at 95 ticks, 11% fast. At 25% fast a 3-bit run rounds down against the nominal bit and fails; measuring the bit recovers it.
