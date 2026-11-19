# 2026-08-20 velocity jig imu range

## Notes moved from code comments

### General

- `velocity_jig/include/config.h`: Full-scale ranges. Battlebot: fast spin + hard hits. The standard ranges are the LSM6DS enums (ISM330DHCX adds only a 4000 dps gyro extension). Both ranges were raised after the 2026-08-20 session ran out of headroom on each. The angular steps peaked at 2054 dps against the 2294 dps that +/-2000 covers, 90% of full scale, and the coupling grid pegged the accelerometer at +/-32764 on nine isolated 1 ms samples with nothing struck: tank-steer chatter through the guard plates, not an impact. The gyro extension also has to cover the raised angular_cap in waveforms.toml. Cost of the change is resolution: both channels halve, so the noise floor per LSB doubles. That is the right trade here, because a clipped sample's true value is unknown and biases every fit that reads it.
