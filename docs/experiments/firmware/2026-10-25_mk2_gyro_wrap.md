# 2026-10-25 mk2 gyro wrap

In the 2026-10-25 01:44 recording, `yaw_rate` went from -1950 to +1997 deg/s in one sample while the robot kept spinning the same way. The unwrapped rate stays negative. The design is in `docs/reference/robot_link/yaw_control.md`. The test is `test_yaw_control`.
