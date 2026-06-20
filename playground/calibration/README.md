# Mrs Buff MK3 drivetrain calibration

Tooling to physically characterize the drivetrain so the headless sim
(`simulation/kinematic_sim_server.py`) becomes a faithful drivetrain testbed, not just a controller
testbed. Background and the parameter taxonomy are in
`docs/experiments/control_improvement/` (Stage 1 report) and the control plan.

The plant fit `playground/control_stage0/fit_plant.py` produces from fight recordings is rough: it runs on
noisy ZED perception poses (flat-plane bias, yaw flips) and gentle driving that sits in the ESC deadzone.
These tools fix that with a deliberate excitation run against clean AprilTag ground truth.

## What gets measured

- Per-direction linear/angular deadzone
- Forward/reverse max-speed asymmetry, max yaw rate
- Separate accel (spin-up) and decel (coast/brake) time constants
- Steer-brake coupling (forward speed lost while turning)
- Actuation lag (Crossfire + ESC + mechanical), the tail not in any recording

## Prerequisites

- `pyserial` (driver script) and `opencv-python` (AprilTag, already a project dep). Install pyserial into
  the venv if missing.
- An AprilTag (`DICT_APRILTAG_36h11`) mounted flat on the robot top; note its physical edge length
  (`--tag-size`, metres) and its heading offset vs robot forward (`--yaw-offset-deg`).
- A printed AprilTag **floor grid** (a `GridBoard`) taped flat on the floor, fully in view, plus the
  **robot tag** taped flat on the robot top. Generate both as print-accurate PDFs with `make_print_tags.py`
  (step 0 below). The grid auto-detects the world frame, so no clicking pixels by hand. Its marker ids
  (`[--floor-first-id, +cols*rows)`, default 10..21) must not collide with the robot `--tag-id` (default 0).
  Defaults are sized for the ZED at VGA: 0.16 m floor markers, a 0.13 m robot tag (~38 px / ~31 px at a
  1 m mount). Print at 100% and verify a marker edge measures the stated size.
- The overhead camera. With the ZED (`--source zed`) intrinsics come from the SDK automatically and the
  camera runs its fastest mode (VGA @ 100 fps; high fps sharpens the actuation-lag estimate). Otherwise
  pass `--intrinsics` as `{ "camera_matrix": [...], "dist_coeffs": [...] }` or `{ "fx":.., "fy":.., "cx":..,
  "cy":.. }`, matched to the running resolution.
- With the intrinsics, the floor grid and the robot tag are both solved by `solvePnP`, so the tag's height
  above the floor does not bias (x, y) (no flat-plane parallax).
- **Guard plates ON**, competition battery, surface matched to the NHRL arena floor. Plate friction is part
  of the plant. See `am32_tuning.md`.
- Driver radio sticks centered: trainer mode adds stick input to the script's command.
- Run the driver and the capture **on the same host** so they share one `CLOCK_MONOTONIC`; the fitter does
  no time alignment.

## Run order

```bash
source scripts/activate_python.sh

# 0. Generate the floor grid + robot tag PDFs, print at 100%, tape the grid on the floor (fully in view)
#    and the tag flat on the robot top.
python playground/calibration/make_print_tags.py --out-dir playground/calibration/print

# 1. Dry-run the protocol (no hardware) to review the maneuver schedule.
python playground/calibration/calibrate_drive.py --dry-run

# 2. Start the overhead ground-truth capture (own terminal). ZED intrinsics + fastest fps (VGA@100) come
#    from the SDK. The floor-grid defaults match make_print_tags.py, so only --tag-size is needed.
python playground/calibration/apriltag_track.py \
    --source zed --tag-size 0.13 --out playground/calibration/out/truth_log.csv

# 3. Run the excitation. Arms only after you type 'go'; disarms on any exit.
python playground/calibration/calibrate_drive.py --out playground/calibration/out/cmd_log.csv

# 4. Fit the plant and write the validation plot.
python playground/calibration/fit_plant_calib.py \
    playground/calibration/out/cmd_log.csv playground/calibration/out/truth_log.csv \
    --plot playground/calibration/out/fit.png
```

The fitter prints the `[our_robot]` block for `simulation/kinematic_sim.toml`, a recommended
`lifted_deadzone_percent` for `config/main.toml`, and the `command_ms` latency.

## Where the results go

- `[our_robot]` block (gains, taus, steer-brake) -> `simulation/kinematic_sim.toml`. The extended
  `Plant` (`simulation/kinematic_sim_server.py`) consumes the new fields; unset fields fall back to the old
  symmetric values, so existing configs are unaffected.
- Measured physical deadzone -> `config/main.toml [transmitter] lifted_deadzone_percent` (currently a
  guess). The sim's `deadzone_*` models only the residual after that compensation.
- `command_ms` -> `simulation/kinematic_sim.toml [latency]`.

## Validate

1. The fitter's printed values are self-consistent and the `--plot` overlay tracks the measured velocity.
2. Re-run a sweep with the new plant and confirm the overshoot magnitudes are now defensible:
   `python playground/control_stage0/sim_sweep.py --sweep playground/control_stage0/sweeps/start_positions.toml`.
3. On the robot, confirm gentle commands now produce motion after applying the measured deadzone.

## ESC tuning loop

`am32_tuning.md` is the AM32 settings guide for a rammer. Each ESC change (Startup Power, Brake on Stop,
etc.) is followed by a re-run of this calibration so its effect on deadzone / max speed / accel tau / coast
tau is measured, not guessed. Dump the ESC config to `am32_settings_<date>.txt` before each change.
