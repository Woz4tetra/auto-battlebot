# Yaw control

`firmware/mr_stabs_mk2/lib/yaw_control` runs the yaw-rate loop with heading hold on the Mr Stabs Mk2.

## Control structure

- The mode is flip-switch DOWN. The turn stick commands a yaw rate. With the stick centered, heading hold commands the rate that steers back to the held heading.
- An inner loop on the BNO055 gyro turns the rate command into the left/right differential the mixer adds to the throttle: feedforward from the stick's command plus proportional and integral feedback on the rate error. It runs once per IMU sample in `mr_stabs_mk2/src/main.cpp`.
- Heading hold's commands get no feedforward. With feedforward on them, the loop acted as a second heading gain and shook the standing robot at 6 Hz (2026-10-06 run).
- Standing still with the turn stick centered, the loop idles: zero output, and the held heading follows the robot. There the robot is an undamped integrator (1533 deg/s^2 of yaw per percent, fit from the 2026-10-06 run) behind a 30 ms delay, so any useful hold gain hunts around the ESC deadzone, and there is no drift to correct.
- Without a usable IMU, the turn stick passes straight through, as in MIDDLE. That covers no sample for `HEADING_STALE_US`, a failed gyro sign check, and an inverted robot.
- The loop replaces the earlier heading-hold PID, which was off while the stick turned and so could not stop a tail-first spin.

## Reverse

- The rate loop exists for reverse. Mr Stabs' center of mass sits 32 mm ahead of the axle, so driving tail-first is directionally unstable. A turn there builds about 1000 deg/s within 0.2 s.
- In reverse the plant also turns several times harder per percent of differential. The feedback gain goes up and the feedforward goes down as reverse throttle rises.

## Units and sign

- Headings are degrees in [0, 360), growing clockwise like the BNO055 Euler heading. Rates are degrees per second, clockwise-positive. Outputs are percent of drive command.
- A positive output speeds up the left wheel and slows the right, which turns the robot clockwise.
- Defaults come from the closed-loop sim (`auto_battlebot/mujoco_sim`), whose plant has not been fit to recordings yet. The Python mirror in `auto_battlebot/mujoco_sim/firmware.py` must match.

## Yaw rate source

- `UpdownSensor` projects the gyro onto the measured up direction (the fused gravity vector, +z with the robot upright, after the axis remap in `begin()`). The yaw rate stays the world yaw rate tilted or upside down.
- The gyro is right-handed, counterclockwise-positive about up. The Euler heading grows clockwise, hence the -1.
- `check_gyro_sign()` verifies the sign live.
- The differential turns the robot the same way inverted, because both the wheel sides and their ground direction swap. The loop therefore runs upside down too. A flip restarts it, because the Euler heading swings while the robot goes over.

## Integrator

- The integral term accumulates only while the output has room (`|unsaturated| < 100`), or when the error would pull the output back toward range.
- A reversed error therefore unwinds a saturated integral instead of freezing it.

## Gyro wrap

- The BNO055 gyro range is +/-2000 deg/s and wraps at 4096 deg/s. A spin faster than the range reads back with the opposite sign.
- That turns the yaw-rate loop into positive feedback: it drives the motors harder to "brake" a spin it sees going the wrong way.
- `unwrap_gyro_rate()` picks the alias of the raw reading nearest the previous unwrapped rate.
- It is exact while the rate changes by under half a wrap (2048 deg/s) between samples. A larger step picks the wrong alias.

## `yaw_control::Config`

Defaults are in `firmware/mr_stabs_mk2/lib/yaw_control/yaw_control.h`. Percent means percent of drive command.

| Field | Meaning |
| --- | --- |
| `max_rate` | deg/s at full stick, the BNO055 gyro range. |
| `feedforward` | Percent per deg/s: the sim's open-loop gain at rest (1/27). |
| `kp` | Percent per deg/s of rate error. |
| `ki` | Percent per degree of integrated rate error. |
| `i_max` | Percent cap on the integral term. |
| `k_heading` | deg/s of rate command per degree of heading error. |
| `hold_rate_max` | deg/s cap on the heading-hold rate command. |
| `reverse_kp_scale` | `kp` multiplier at full reverse throttle. |
| `reverse_ff_scale` | Feedforward multiplier at full reverse throttle. |
| `turn_threshold` | Percent of stick above which the driver is turning. |
| `capture_rate` | deg/s below which a released turn has stopped. |
| `capture_timeout` | Seconds after release to take the heading regardless. |
| `idle_throttle` | Percent of throttle below which a centered stick idles. |
