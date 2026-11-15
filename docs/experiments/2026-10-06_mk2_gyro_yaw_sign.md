# 2026-10-06 mk2 gyro yaw sign

## Notes moved from code comments

### General

- `mr_stabs_mk2/include/updown_sensor.h`: The yaw rate is the gyro projected onto the measured up direction (the fused gravity vector, which reads +z with the robot upright, after the axis remap in begin()), so it stays the world yaw rate tilted or upside down. The gyro is right-handed, counterclockwise-positive about up; the Euler heading grows clockwise, hence -1. On the 2026-10-06 run, gyro z alone agreed with the heading change on 100% of fast samples upright and on 0% inverted. check_gyro_sign() verifies it live.
