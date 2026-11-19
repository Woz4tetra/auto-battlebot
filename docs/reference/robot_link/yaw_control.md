# Yaw control

`firmware/mr_stabs_mk2/lib/yaw_control` runs the yaw-rate loop with heading hold on the Mr Stabs Mk2.

## Integrator

- The integral term accumulates only while the output has room (`|unsaturated| < 100`), or when the error would pull the output back toward range.
- A reversed error therefore unwinds a saturated integral instead of freezing it.

## Gyro wrap

- The BNO055 gyro range is +/-2000 deg/s and wraps at 4096 deg/s. A spin faster than the range reads back with the opposite sign.
- That turns the yaw-rate loop into positive feedback: it drives the motors harder to "brake" a spin it sees going the wrong way.
- `unwrap_gyro_rate()` picks the alias of the raw reading nearest the previous unwrapped rate.
- It is exact while the rate changes by under half a wrap (2048 deg/s) between samples. A larger step picks the wrong alias.
