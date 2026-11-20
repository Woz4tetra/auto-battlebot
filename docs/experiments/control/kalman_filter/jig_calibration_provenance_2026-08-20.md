# Velocity jig calibration provenance

Measurements behind `playground/calibration/jig_calibration.toml` and `playground/calibration/waveforms.toml`. Field meanings are in `docs/reference/calibration/velocity_jig_config.md`.

## E4: `meters_per_count = 4.183811e-05`

Measured 2026-08-20 from twelve hand-pushed out-and-back passes at four distances:

| Distance | Logs |
| --- | --- |
| 0.50 m | LOG-38, LOG-39, LOG-40 |
| 0.75 m | LOG-42, LOG-43, LOG-44 |
| 1.00 m | LOG-45, LOG-46, LOG-47 |
| 3.00 m | LOG-50, LOG-51, LOG-52 |

```bash
python playground/calibration/fit_encoder_scale.py \
    --run 0.5  LOG-38.TXT LOG-39.TXT LOG-40.TXT \
    --run 0.75 LOG-42.TXT LOG-43.TXT LOG-44.TXT \
    --run 1.0  LOG-45.TXT LOG-46.TXT LOG-47.TXT \
    --run 3.0  LOG-50.TXT LOG-51.TXT LOG-52.TXT \
    --plot playground/calibration/out/encoder_scale.png
```

- The value is the slope of counts against distance. There is a +113 count (+4.7 mm) fixed offset per pass, from coasting past the mark or backlash taken up at the start. The slope is differential, so the offset cancels. A fit through the origin gives 4.174e-05, 0.2% high.
- Uncertainty is 0.14%, under the runbook's 0.3% bar. The 3 m passes carry it. An earlier fit over 0.5 to 1.0 m alone gave 4.2547e-05 +/- 0.63% with a +17 mm offset, and both were artifacts of too short a lever arm.
- Worst closure across the twelve passes was 5.3 mm.

## E3: `[gyro] scale = 0.9987`

Measured 2026-08-20 from two ten-turn hand rotations on a pivot:

| Log | Rotation | Gyro read | Error |
| --- | --- | --- | --- |
| LOG-54 | +3600 deg | 3604.4 | +0.12% |
| LOG-55 | -3600 deg | -3605.0 | -0.14% |

```bash
python playground/calibration/fit_gyro_scale.py \
    --run 3600 LOG-54.TXT --run -3600 LOG-55.TXT \
    --plot playground/calibration/out/gyro_scale.png
```

- The two directions agree to 0.02%, so this is a scale factor and not a mounting sign problem.
- It is well inside the runbook's 1% bar. Carrying it or dropping it makes little difference to anything fitted downstream.
- Peak rate was 875 deg/s, inside the +/-2294 deg/s the header reports. Stage 2 measured a driven spin of 61.5 rad/s, which is 3524 deg/s and would clip. Switch the gyro to 4000 dps before recording angular steps.

## E2: gyro noise model

From LOG-97, 30.5 min stationary, 2026-08-21:

```bash
python playground/calibration/fit_gyro_noise.py \
    playground/calibration/out/gyro_bias/LOG-97.TXT \
    --plot playground/calibration/out/gyro_noise.png
```

The parameters are inert until the noise model is incorporated into the plant fitting script.

## Waveform choices

### Hold and coast length

- The old `hold_s` comment justified 0.6 s by floor length, not by the runbook's 2 s. At the fitted `tau_lin_a` of 0.1309 s, 0.6 s is 4.6 time constants (98.98% settled), and a full-command step costs 1.88 m including the coast to a stop. Two seconds would be 15 tau and 6.57 m, more floor than this robot has and more settling than a first-order rise can use.
- The runbook's 2 s predates the measured tau. It guarded against stage 2's 0.2 s, which is 1.5 tau and only 78% settled, so there was no plateau to read.
- `k_rev` is +/- 38% from n=4. Reps fix that where a longer hold does not.
- The old `coast_s` comment said a long tail costs 0.30 m regardless, and that 2.0 s is 22 decel constants.
- The file holds `hold_s = 2.0` and `coast_s = 3.0` today, not the 0.6 s the first comment argued for. The reasoning above is kept as measured, but the values do not match it.

### `angular_cap`

- The earlier 0.13 was 8.0 / 61.5, using stage 2's camera `k_ang`.
- The jig measures `k_ang` at 32.16 +/- 0.572 rad/s. The raw gyro counts rule out 61.5: LOG-69 peaked at 2054 dps against a 2294 dps full scale, where 61.5 rad/s would be 3524 dps and would have railed.
- So the cap doubles to 8.0 / 32.16 = 0.249. The file holds 0.24.
- This assumed `rate_limit = 8.0` was a real E5 measurement. When the comment was written E5 had not been run and `r_enc_perp` and `r_imu` were both 0.0. The file now holds `r_enc_perp = 0.156` and `r_imu = 0.022`.

### Deadzone staircase

- The 2026-08-20 fit put the forward crossing between 0.000 and 0.010, so the robot had already moved on the first rung and the crossing was an extrapolation. The staircase now starts at 0.001.
- Left angular crossed between 0.000 and 0.010 on 2026-08-20. Right angular crossed between 0.010 and 0.019.
- 30 rungs at 2 s each is 60 s per direction.
- The old comment on the linear staircase said rungs at 1.0 s cover 0.82 m per direction and 7.6 tau per rung. The file holds `hold_s = 0.5` for that entry.

### Reverse cap

- `lin_step_full` and `lin_coast` use `reverse_cap = 0.4`, where the robot breaks into an uncommanded spin at and above 0.5 reverse.
- The spin threshold was observed, not measured.

### Coupling grid

- A 1.0 linear cell at the 0.24 angular cap goes out as 0.76. The 0.24 gap between commanded and measured tripped the contamination gate at 0.05 and discarded LOG-71 as a leaning stick when the sticks were centered. Every cell over the bar had `tgt_lin = 1.0`, and the median error across all 56 held stretches was 0.0029, the same as the clean runs.

### Holdouts

- LOG-76, LOG-77 and LOG-88 were recorded with `lin_prbs` at 0.6, so the whole linear holdout set validated the model against a scrubbing encoder wheel. LOG-88 was recorded with `lin_sine` at 0.5.
- `lin_prbs` bit rate was unchanged when the amplitude moved to 0.4.
- 60 ms bits cover roughly 0.2 to 8 Hz.
