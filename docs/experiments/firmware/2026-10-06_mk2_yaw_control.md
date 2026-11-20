# 2026-10-06 mk2 yaw control

Measurements from the 2026-10-06 run that set the yaw control structure. The design is in `docs/reference/robot_link/yaw_control.md`.

## Heading hold feedforward

- Feedforward on heading hold's commands acted as a second heading gain. With the robot standing still it shook it at 6 Hz.

## Standing plant

- With the turn stick centered and the robot standing still, the plant is an undamped integrator of 1533 deg/s^2 of yaw per percent, fit from this run, behind a 30 ms delay.
