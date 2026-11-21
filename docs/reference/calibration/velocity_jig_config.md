# Velocity jig config files

`auto_battlebot/calibration/jig/jig_calibration.toml` holds the bench constants and `auto_battlebot/calibration/jig/waveforms.toml` holds the excitation waveforms. Measured values and the runs behind them are in `docs/experiments/control/kalman_filter/jig_calibration_provenance_2026-08-20.md`. Procedures are runbook block 0 in `docs/experiments/control/kalman_filter/velocity_jig_runbook.md`.

## `auto_battlebot/calibration/jig/jig_calibration.toml`

`fit_jig_plant.py`, `fit_process_noise.py` and `plot_jig_runs.py` read it, and all three default to this path. Keep one set per hardware configuration. Remounting the encoder changes the effective wheel radius, and a 2% scale error there is a 2% velocity error in everything fitted afterwards, so re-measure after any change to the mount.

### `[encoder]`

| Field | Meaning |
| --- | --- |
| `meters_per_count` | E4, metres per quadrature count. Every linear measurement depends on it. It is the slope of counts against distance, not a fit through the origin. Fit with `fit_encoder_scale.py`. |
| `r_enc_perp` | E5, the encoder wheel's offset perpendicular to the forward axis, metres. Removed from the speed measurement so combined linear-plus-angular runs are usable. Regress encoder rate against gyro yaw rate over pure spins; the slope is this. |
| `rate_limit` | E5, yaw rate in rad/s above which the encoder wheel scrubs sideways instead of rolling. Caps the amplitude of every angular waveform through `angular_cap`. Stop and lower it if the wheel chatters or leaves a scrub mark. |

### `[gyro]`

| Field | Meaning |
| --- | --- |
| `scale` | E3, integrated gyro angle against a known rotation. A scale factor, not a mounting sign. Fit with `fit_gyro_scale.py`. The runbook bar is 1%. Peak rate during the fit was 875 deg/s, inside the +/-2294 deg/s the header reports. Switch the gyro to 4000 dps before recording angular steps. |
| `bias_dps`, `arw_deg_sqrt_hr`, `bias_instability_dps`, `bias_tau_s`, `rrw_deg_per_s_sqrt_s` | E2, stationary-gyro noise model from `fit_gyro_noise.py`, per axis for `bias_dps`. Nothing reads them yet. |

### `[imu]`

| Field | Meaning |
| --- | --- |
| `r_imu` | E5, the IMU's offset from the yaw axis, metres. Regress measured lateral acceleration against yaw rate squared over pure spins; the slope is this. Kept as a check against a tape measure. The slip detector fits the lever-arm terms per run instead. |

## `auto_battlebot/calibration/jig/waveforms.toml`

`velocity_jig_drive.py --waveform <name> --name ...` drives an entry, and the fit reads the same file. Each `[waveform.<name>]` table declares `kind`, `channel` and `role`. The fit routes on those three, so a new waveform reaches the right fit without editing `fit_jig_plant.py`. `[defaults]` merges into every entry.

| Key | Values |
| --- | --- |
| `kind` | `step`, `staircase`, `coast`, `grid`, `prbs`, `chirp`, `sine`, `triangle`, `trim`, `manual` |
| `channel` | `linear`, `angular`, `combined` |
| `role` | `fit` (trains the model) or `holdout` (validates it, never fitted) |

Amplitudes are normalized to [-1, 1]. Deadzone is not compensated, because the physical deadzone is one of the things being measured.

`reverse_cap` on a step or coast waveform scales the reverse half of the ladder to that peak and leaves forward alone. Set it per waveform, not in `[defaults]`: on an angular waveform it would shrink right turns against left ones and bias `k_ang`.

### `[defaults]`

| Field | Meaning |
| --- | --- |
| `hold_s` | Step hold time. The runbook asks for 2 s. Precision on `k_fwd` and `k_rev` comes from segment count, not segment length. Angular waveforms override it. |
| `coast_s` | Coast tail length. |
| `angular_cap` | Cap on angular amplitudes. Set it to `rate_limit / k_ang`, with `k_ang` from your own fit. |

### Linear entries

| Entry | Meaning |
| --- | --- |
| `lin_deadzone` | Staircase from 0.001 to 0.030 in 0.001 steps, both directions. Rungs run back to back with no return to zero. |
| `lin_step_full` | Shuttle steps at 1.0, 0.75, 0.5, 0.25. `reverse_cap = 0.4` scales the reverse ladder to [0.1, 0.2, 0.3, 0.4]. At and above 0.5 reverse the robot breaks into an uncommanded spin, which feeds `k_rev`, `tau_lin_a` and both drift terms with bad data. |
| `lin_coast` | Coast tails from 0.5. Reverse entries drop to 0.4. The decel constant is read off the tail. |

### Angular entries

| Entry | Meaning |
| --- | --- |
| `ang_deadzone` | Same staircase as `lin_deadzone`. |
| `ang_step_full` | Shuttle steps at 0.25, 0.5, 0.75, 1.0 with `cap = 1.0`, 3 s hold and 3 s coast. |
| `ang_coast` | Dedicated excitation for the angular decel constant. Without it that constant is fitted from the drops at the end of angular steps alone. |

### Coupling entry

| Entry | Meaning |
| --- | --- |
| `coupling_grid` | Linear tops out at 0.76 because `to_channels` gives angular priority and clips linear to `1 - abs(ang)`. The angular axis spans both signs: steer-brake and angular droop flip with turn direction while straight-line drift does not, so a one-way grid folds the drift into the droop coefficient. |

### Holdout entries

Holdouts test the model and are never fitted.

| Entry | Meaning |
| --- | --- |
| `lin_prbs` | Amplitude 0.4. A PRBS is symmetric, so a larger amplitude commands the reverse bits past the uncommanded-spin threshold. `bit_ms = 60` gives roughly 0.2 to 8 Hz. |
| `lin_chirp` | Frequency-domain cross-check on a time constant fitted from steps. |
| `lin_sine` | Amplitude 0.4, same as `lin_prbs`. |
| `operator` | Emits nothing. The operator drives, and the radio's own mixer output is the command log. |
| `trim` | Low and slow, to measure how far the robot arcs before a long straight run. |
| `ang_prbs`, `combined_prbs`, `ang_chirp`, `lin_triangle` | Defined by `kind`, `channel` and `role` alone. |

## `auto_battlebot/calibration/write_plant_configs.py`

`SIM_FIELDS` maps simulation TOML keys to fit names. The unsuffixed `max_linear_speed`, `tau_linear` and `tau_angular` keys are the symmetric fallback. They map to `k_fwd`, `tau_lin_a` and `tau_ang_a` and go unused once the `_fwd`/`_rev` and `_accel`/`_decel` keys are set.
