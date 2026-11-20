# 2026-08-20 velocity jig IMU range

## Full-scale ranges

The velocity jig (`velocity_jig/include/config.h`) is meant for a battlebot: fast spin and hard hits. The standard ranges are the LSM6DS enums. The ISM330DHCX adds only a 4000 dps gyro extension.

Both ranges were raised after the 2026-08-20 session ran out of headroom on each.

- Gyro: the angular steps peaked at 2054 dps against the 2294 dps that +/-2000 covers, 90% of full scale. The gyro extension also has to cover the raised `angular_cap` in `waveforms.toml`.
- Accelerometer: the coupling grid pegged it at +/-32764 on nine isolated 1 ms samples with nothing struck. That was tank-steer chatter through the guard plates, not an impact.

The cost is resolution. Both channels halve, so the noise floor per LSB doubles. That is the right trade here, because a clipped sample's true value is unknown and biases every fit that reads it.
