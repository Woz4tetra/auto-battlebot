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

- Install the subfolder deps into the venv: `pip install -r playground/calibration/requirements.txt`
  (DepthAI for the OAK, plus opencv/numpy/pyserial/Pillow/matplotlib/trimesh). DepthAI is pinned to the v2
  line; `apriltag_track.py` uses the v2 pipeline API.
- An AprilTag (`DICT_APRILTAG_36h11`) mounted flat on the robot top; note its physical edge length
  (`--tag-size`, metres) and its heading offset vs robot forward (`--yaw-offset-deg`).
- The manufactured AprilTag **floor grid** (a `GridBoard`, 3x5 of 65 mm markers, 15 mm gaps, ids 160..174
  numbered right-to-left per row) placed flat and fully in view for the one-time floor lock, then
  **removed** so it never obstructs the robot. Plus the **robot tag** (id 20, off the grid's range); generate
  its print-accurate PDF with `make_print_tags.py` (step 0). The grid auto-detects the world frame, so no
  clicking pixels by hand. Detection is pixels-limited: face-on, 36h11 needs ~18 px edge to decode. At
  1080p the 65 mm floor markers are ~36 px @1 m, so the one-time lock works at the mount; if it does not
  catch, place or hold the board closer (it is removed afterward anyway).
- The overhead camera: a Luxonis **OAK-1 W** (`--source oak`). Intrinsics and distortion come from the
  on-device factory calibration automatically; it runs 1080p @ 60 fps (resolution clears the ~18 px
  detection cliff at a 1-1.5 m mount, 60 fps meets the actuation-lag estimate's >=60 fps need). For any
  other camera/video, pass `--intrinsics` as `{ "camera_matrix": [...], "dist_coeffs": [...] }` or
  `{ "fx":.., "fy":.., "cx":.., "cy":.. }`, matched to the running resolution.
- Lighting. Dim arenas crush the marker contrast and detection fails (a raw frame at mean ~52/255 detected
  0 tags). Each frame is auto-brightened before detection (recovered ~10/15 on that frame), and the preview
  shows the brightened image, but good, even lighting still helps: avoid a bright background that fools the
  camera's auto-exposure into underexposing the floor.
- The OAK-1 W's wide lens is distorted, but the floor grid and the robot tag are both solved by `solvePnP`
  with the real distortion coefficients, so the tag's height above the floor does not bias (x, y) (no
  flat-plane parallax).
- **Guard plates ON**, competition battery, surface matched to the NHRL arena floor. Plate friction is part
  of the plant. See `am32_tuning.md`.
- Driver radio sticks centered: trainer mode adds stick input to the script's command.
- Run the driver and the capture **on the same host** so they share one `CLOCK_MONOTONIC`; the fitter does
  no time alignment.

## Run order

```bash
source scripts/activate_python.sh
pip install -r playground/calibration/requirements.txt   # once: DepthAI (OAK) + the rest

# 0. Print the robot tag PDF at 100% and tape it flat on the robot top. The floor grid is the manufactured
#    board, so it needs no printing (make_print_tags also writes a floor_grid.pdf for reprints only).
python playground/calibration/make_print_tags.py --out-dir playground/calibration/print

# 1. Dry-run the protocol (no hardware) to review the maneuver schedule.
python playground/calibration/calibrate_drive.py --dry-run

# 2. Start the overhead ground-truth capture (own terminal). OAK-1 W intrinsics + distortion + 1080p@60
#    come from the device; defaults match the manufactured board. It prompts you to place the floor board
#    to lock the world frame, then to REMOVE the board before driving so it does not block the robot.
#    This records the raw camera images to MCAP; AprilTag detection + the pose solve happen offline (step 4).
python playground/calibration/apriltag_track.py \
    --source oak --out playground/calibration/out/apriltag_track.mcap

# 3. Run the excitation. Arms only after you type 'go'; disarms on any exit.
python playground/calibration/calibrate_drive.py --out playground/calibration/out/cmd_log.csv

# 4. Solve the field-plane poses from the recording -> the (t, x, y, yaw, visible) truth CSV.
python playground/calibration/analyze_apriltag_mcap.py \
    playground/calibration/out/apriltag_track.mcap \
    --out playground/calibration/out/truth_log.csv --plot playground/calibration/out/track.png

# 5. Fit the plant and write the validation plot.
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
