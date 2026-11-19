# Robot plant files

2026-11-18. The `[plant]` tables moved out of the profiles into `config/robots/`, loaded with
`[plant] from = "robots/<file>"` (`docs/config_extends.md`). Before the move, 9 files carried a
`[plant]` table. All 31 profiles resolved to byte-identical `--print-config` output before and
after.

## Files

| File | Values | Loaded by |
| --- | --- | --- |
| `config/robots/mrs_buff_mk3.toml` | Mrs Buff Mk3 Stage A jig fit, hand-edited (see below) | `_common.toml`, so every Mrs Buff profile |
| `config/robots/mr_stabs_mk2.toml` | Mr Stabs Mk2 distilled MuJoCo fit | `mr_stabs_mk2_desktop`, `mr_stabs_mk2_jetson`, `playback/mr_stabs_mk2_playback`, `simulation/mr_stabs_mk2_kinematic_sim`, `simulation/mr_stabs_mk2_kinematic_sim_sensing` |
| `config/robots/mr_stabs_mk2_geometry.toml` | Mr Stabs Mk2 drivetrain geometry estimate | `mr_stabs_mk2_aer_zed_box`, `mr_stabs_mk2_fid_zed_box` |

`simulation/mrs_buff_mk3_rl_sim.toml` still writes its own `[plant]` inline. It merges key by key
over the Mrs Buff file.

Every file also carries `[plant.process_noise]`, because a `from` table replaces the inherited
one outright. All three hold the Mrs Buff Mk3 noise fit. Mr Stabs Mk2 has no process noise fit of
its own; it ran Mrs Buff's table through the merge before the move, and the copy keeps that.

## Mrs Buff Mk3

- Source: Stage A per-phase fit, `playground/calibration/out/plant_stageA.toml`, sessions
  2026-08-19 through 2026-08-23, model M4 (drift terms disabled by the ladder, written as zero).
  Measured with guard plates on, so their ground friction is part of the fitted plant.
- `delay_s` is the onset-stack measurement (3486 edges, SNR 22.4), corroborated by the tau-pinned
  profile (54 ms) and the stage 2 camera value (59 ms).
- The table does not match the fit. `write_plant_configs.py --check` reports 11 fields drifted:
  the deadzones, `tau_*`, `c_sb` and `c_ad` are zero in the config, and `k_fwd` is 4.3546 against
  the fit's 4.88002. The fit's `tau_lin_d` is 0.123461. This drift existed in
  `config/_common.toml` before the move.

## Process noise

- Source: `fit_process_noise.py` output, `playground/calibration/out/process_noise.toml`, same
  sessions and plant as the Mrs Buff fit. Only the EKF reads it.
- Growth-law mechanisms: white-noise accel (h^3), scale-factor error on v and w (h^2, applied
  state-dependent at runtime), heading random walk (h), delay jitter (constant, the fit found
  none).
- `q_along = 10.0` is hand-set, not from the fit. The fitted 0.37125 left the EKF's forward-speed
  sigma at 0.17 m/s while its speed was wrong by 0.56 m/s, so the filter out-voted its own
  measurements and the motion profile braked on a number that read half the robot's real speed.
  It was chosen by sweeping against a 150 ms chord reference on the 2026-08-23 20-37 RUN_AWAY
  segments and taking the point where the speed innovations become chi-square consistent:
  median NIS 0.44 against the 0.455 target for 1 DOF, from 5.65 at the fitted value. Position
  jitter is unchanged (2.9 to 3.1 mm) and heading rejections fall from 4.4% to 0.1%. Replace it
  with a real fit when `fit_process_noise.py` next runs on a session with autonomous RUN_AWAY
  motion in it.

## Mr Stabs Mk2 distilled

- Source: `playground/calibration/out/mujoco_sysid/fit_2026-10-15_yolo_brake/distilled/plant.toml`.
  `write_plant_configs.py --robot mr_stabs_mk2 --fit <that file> --check` reports it up to date.
- `write_plant_configs.py --robot mr_stabs_mk2` writes this file only.
- The comments removed from `playback/mr_stabs_mk2_playback.toml` and
  `simulation/mr_stabs_mk2_kinematic_sim*.toml` described the geometry derivation below, but those
  profiles held the distilled values.

## Mr Stabs Mk2 geometry

Derived from drivetrain geometry before the 2026-10-15 fit: full stick = `max_motor_rpm` at the
wheel.

- `k_fwd = k_rev = 1500 RPM * pi * 0.05 m / 60 = 3.92699 m/s`
- `k_ang = 2 * 3.92699 / 0.131 m track = 59.9541 rad/s`
- Deadzones and coupling are zero because geometry cannot predict them. The time constants and
  delay are zero, so the navigation feedforward adds no derivative terms.
