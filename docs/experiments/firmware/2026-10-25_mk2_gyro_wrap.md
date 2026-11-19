# 2026-10-25 mk2 gyro wrap

## Notes moved from code comments

### General

- `mr_stabs_mk2/test/test_yaw_control/test_yaw_control.cpp`: 2026-10-25 01:44 recording: yaw_rate went from -1950 to +1997 deg/s in one sample while the robot kept spinning the same way. The unwrapped rate stays negative.
