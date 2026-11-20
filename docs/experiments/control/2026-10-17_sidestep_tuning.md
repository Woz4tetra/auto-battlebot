# Sidestep controller tuning

Measurements behind the defaults in `SidestepConfig` (`auto_battlebot/rl/sidestep.py`). Field meanings are in `docs/reference/control/rl_training_config.md`.

- Heading loop: tuned on a 90 degree step in the Warp plant. `gain` 0.3 settles in 0.4 s. `gain` 0.8 oscillated at +-17 rad/s through the 40 to 80 ms command latency.
- `max_angular` 0.5 is about 19 rad/s. Full angular action spins the chassis over.
- `strike_linear` inside `slow_m`: ramming the immovable opponent at 2.6 m/s stopped the robot in one step and pitched it over its nose.
- `back_off_s` is 0.0. Against the slowest pursuer, backing straight out stayed in its arc and the pursuer pinned us, 30 hits taken per episode.
- `linear_slew` 0.1: swinging linear from -0.5 to +0.6 in 0.16 s while backing at 2 m/s pitched the nose to -84 degrees and flipped the robot. 0.04 was too slow to get round a pursuer.
