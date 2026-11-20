# Velocity jig config files

`playground/calibration/jig_calibration.toml` holds the bench constants and `playground/calibration/waveforms.toml` holds the excitation waveforms. Measured values and the runs behind them are in `docs/experiments/control/kalman_filter/jig_calibration_provenance_2026-08-20.md`. Procedures are runbook block 0 in `docs/experiments/control/kalman_filter/velocity_jig_runbook.md`.

## `playground/calibration/jig_calibration.toml`

`fit_jig_plant.py`, `fit_process_noise.py` and `plot_jig_runs.py` read it, and all three default to this path. Keep one set per hardware configuration. Remounting the encoder changes the effective wheel radius, and a 2% scale error there is a 2% velocity error in everything fitted afterwards, so re-measure after any change to the mount.

### `[encoder]`

| Field | Meaning |
| --- | --- |
| `meters_per_count` | E4, metres per quadrature count. Every linear measurement depends on it. It is the slope of counts against distance, not a fit through the origin: each pass has a fixed offset (coasting past the mark, or backlash taken up at the start) that the slope cancels. Fit with `fit_encoder_scale.py`. |
| `r_enc_perp` | E5, the encoder wheel's offset perpendicular to the forward axis, metres. Removed from the speed measurement so combined linear-plus-angular runs are usable instead of discarded. Regress encoder rate against gyro yaw rate over pure spins; the slope is this. |
| `rate_limit` | E5, yaw rate in rad/s above which the encoder wheel scrubs sideways instead of rolling. Caps the amplitude of every angular waveform through `angular_cap`. Stop and lower it if the wheel chatters or leaves a scrub mark. |

### `[gyro]`

| Field | Meaning |
| --- | --- |
| `scale` | E3, integrated gyro angle against a known rotation. A scale factor, not a mounting sign: the two spin directions agree. Fit with `fit_gyro_scale.py`. The runbook bar is 1%. Peak rate during the fit was 875 deg/s, inside the +/-2294 deg/s the header reports, but a driven spin reaches higher rates. Switch the gyro to 4000 dps before recording angular steps. |
| `bias_dps`, `arw_deg_sqrt_hr`, `bias_instability_dps`, `bias_tau_s`, `rrw_deg_per_s_sqrt_s` | E2, stationary-gyro noise model from `fit_gyro_noise.py`, per axis for `bias_dps`. Nothing reads them until the noise model is incorporated into the plant fitting script. |

### `[imu]`

| Field | Meaning |
| --- | --- |
| `r_imu` | E5, the IMU's offset from the yaw axis, metres. Regress measured lateral acceleration against yaw rate squared over pure spins; the slope is this. Kept as a check against a tape measure. The slip detector fits the lever-arm terms per run instead, because a single scalar cannot say which of two terms an offset lands on. |

## `playground/calibration/waveforms.toml`

`velocity_jig_drive.py --waveform <name> --name ...` drives an entry, and the fit reads the same file. Each `[waveform.<name>]` table declares `kind`, `channel` and `role`. The fit routes on those three, so a new waveform reaches the right fit without editing `fit_jig_plant.py`. `[defaults]` merges into every entry.

| Key | Values |
| --- | --- |
| `kind` | `step`, `staircase`, `coast`, `grid`, `prbs`, `chirp`, `sine`, `triangle`, `trim`, `manual` |
| `channel` | `linear`, `angular`, `combined` |
| `role` | `fit` (trains the model) or `holdout` (validates it, never fitted) |

Amplitudes are normalized to [-1, 1]. Deadzone is not compensated anywhere, because the physical deadzone is one of the things being measured.

`reverse_cap` on a step or coast waveform scales the reverse half of the ladder to that peak and leaves forward alone. Use it when the robot misbehaves in one direction only. Set it per waveform, not in `[defaults]`: on an angular waveform it would shrink right turns against left ones and bias `k_ang`.

### `[defaults]`

| Field | Meaning |
| --- | --- |
| `hold_s` | Step hold time. The runbook asks for 2 s. A shorter hold is justified by floor length, since a full-command step costs distance including the coast to a stop. Precision on `k_fwd` and `k_rev` comes from segment count, not segment length, so add reps before lengthening a hold. Angular waveforms override it because a spin costs no floor. |
| `coast_s` | Coast tail length. Coasting starts after the robot has stopped travelling under command, so a long tail costs a fixed distance regardless of the command. |
| `angular_cap` | Cap on angular amplitudes. The encoder wheel scrubs sideways through every spin, so the bench `rate_limit` caps every angular run. Set it to `rate_limit / k_ang`, with `k_ang` from your own fit. Assumes `rate_limit` is a real E5 measurement. Stop the moment the encoder wheel chatters or leaves a scrub mark. |

### Linear entries

| Entry | Meaning |
| --- | --- |
| `lin_deadzone` | Staircase from 0.001 to 0.030 in 0.001 steps, both directions. It starts at 0.001, not 0.01, because resolution is the step size: 0.01 steps can only report +/- 0.0098, and a crossing below the first rung is an extrapolation. It stops at 0.030 because the crossing is under 0.01, so higher rungs measure nothing the step runs do not cover. Rungs run back to back with no return to zero, so distance accumulates across all 30. |
| `lin_step_full` | Shuttle steps at 1.0, 0.75, 0.5, 0.25. `reverse_cap = 0.4` scales the reverse ladder to [0.1, 0.2, 0.3, 0.4]. At and above 0.5 reverse the robot breaks into an uncommanded spin. A spun hold is not a straight-line steady state: the encoder wheel scrubs sideways, so the reported arc length is not the distance travelled, and the `r_enc_perp` correction assumes rolling and cannot recover it. Nothing downstream catches this, because `held_segments` gates on the commanded angular channel, which is zero through a linear run. Spun rungs would feed `k_rev`, the pooled `tau_lin_a` and both drift terms as clean data. 0.4 rather than 0.45 leaves margin, since the spin threshold was observed, not measured, and moves with floor and pack voltage. Raise it once the underlying asymmetry is fixed, because until then the reverse gain is measured over a shorter lever arm. `trim = false`: this is the run the straight-line drift is measured from, so correcting the arc would remove the signal. |
| `lin_coast` | Coast tails from 0.5. Reverse entries drop to 0.4 for the same spin threshold as `lin_step_full`. The decel constant is read off the tail, and a tail that starts from a spin decays with the scrub in it. |

### Angular entries

| Entry | Meaning |
| --- | --- |
| `ang_deadzone` | Same staircase as `lin_deadzone`. Pivoting in place costs no floor, so the runbook's 2 s hold is free. The left crossing needs the range and the right crossing needs the resolution. |
| `ang_step_full` | Shuttle steps at 0.25, 0.5, 0.75, 1.0 with `cap = 1.0`, 3 s hold and 3 s coast. |
| `ang_coast` | Dedicated excitation for the angular decel constant. Without it that constant is fitted from the drops at the end of angular steps alone. Add this run when the constant comes back weak. |

### Coupling entry

| Entry | Meaning |
| --- | --- |
| `coupling_grid` | Linear tops out at 0.76, not 1.0, so no cell asks for more authority than the mix can deliver. `to_channels` gives angular priority and clips linear to `1 - abs(ang)`, so a 1.0 cell at the 0.24 cap goes out as 0.76. The 0.24 gap between commanded and measured trips the contamination gate at 0.05 and discards the run as a leaning stick. The angular axis spans both signs, and that is load-bearing: steer-brake and angular droop flip with turn direction while straight-line drift does not. A grid that turns one way cannot separate them and folds the drift into the droop coefficient. |

### Holdout entries

Holdouts test the model and are never fitted.

| Entry | Meaning |
| --- | --- |
| `lin_prbs` | Amplitude 0.4, not 0.6. A PRBS is symmetric by construction, so 0.6 commands -0.6 on every reverse bit, past the amplitude where the robot spins uncommanded. A holdout that fails for that reason looks exactly like a model error. Capping only the reverse half is not an option: an asymmetric PRBS has different spectral content and a DC bias that walks the robot down the floor over 30 s. The cost is excitation amplitude, so prediction error is measured against a smaller signal. Raise it once the reverse asymmetry is fixed. `bit_ms = 60` sets the bandwidth, roughly 0.2 to 8 Hz, which brackets the 58-78 ms plant time constants and the 33 ms frame period. |
| `lin_chirp` | Frequency-domain cross-check on a time constant fitted from steps. A first-order model with tau 58 ms has its corner near 2.7 Hz. |
| `lin_sine` | Amplitude 0.4 for the same reason as `lin_prbs`: a sine is symmetric, so a larger amplitude drives the reverse half into the spin. |
| `operator` | Emits nothing. The operator drives, and the radio's own mixer output is the command log, which is why the readback exists. |
| `trim` | Low and slow, to measure how far the robot arcs before committing to a long straight run. |
| `ang_prbs`, `combined_prbs`, `ang_chirp`, `lin_triangle` | No rationale beyond `kind`, `channel` and `role`. |

## `playground/calibration/write_plant_configs.py`

`SIM_FIELDS` maps simulation TOML keys to fit names. The unsuffixed `max_linear_speed`, `tau_linear` and `tau_angular` keys are the symmetric fallback. They map to `k_fwd`, `tau_lin_a` and `tau_ang_a` and go unused once the `_fwd`/`_rev` and `_accel`/`_decel` keys are set.
