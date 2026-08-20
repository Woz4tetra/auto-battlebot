# Mrs Buff MK3 drivetrain calibration

Tooling to physically characterize the drivetrain so the headless sim
(`simulation/kinematic_sim_server.py`) becomes a faithful drivetrain testbed, not just a controller
testbed. Background and the parameter taxonomy are in
`docs/experiments/control_improvement/` (Stage 1 report) and the control plan.

There are two ground-truth paths here. **The velocity jig is the current one**; the AprilTag path
below it came first and its analysis scripts are kept for the recordings already made with it.

## Velocity jig (current)

An RP2040 datalogger rides on the robot and records a wheel encoder plus an IMU at 1 kHz, so the
ground truth is on-robot rather than from an overhead camera. Firmware and wiring:
`firmware/velocity_jig`. Procedure: `docs/experiments/kalman_filter/velocity_jig_runbook.md`.

```bash
source scripts/activate_python.sh

# Inspect the waveform catalog and preview a program, no hardware needed.
python playground/calibration/velocity_jig_drive.py --list-waveforms
python playground/calibration/velocity_jig_drive.py --waveform lin_step_full --dry-run \
    --half-width-m 0.25 --params playground/calibration/out/plant_params.toml

# Confirm which serial port is the jig and which is the radio.
python playground/calibration/velocity_jig_drive.py --list-ports

# Record. Each run leaves LOG-N.TXT, LOG-N.toml and LOG-N.cmd.csv in the session directory,
# which defaults to out/<date>-<name>. Pass --out to put it somewhere else.
python playground/calibration/velocity_jig_drive.py \
    --waveform lin_step_full --waveform lin_coast --reps 3 \
    --name "garage floor, pack 3"

# Eyeball each run. Writes LOG-N.png beside every log that does not have one yet, so
# running it again after three more runs costs three plots.
python playground/calibration/plot_jig_runs.py

# Fit, and read the report between batteries to decide what to record next. The calibration
# defaults to playground/calibration/jig_calibration.toml.
python playground/calibration/fit_jig_plant.py \
    playground/calibration/out/2026-08-19-garage-floor-pack-3 \
    --out playground/calibration/out/plant_params.toml \
    --report playground/calibration/out/jig_fit.html

# Process noise, from the same session directories.
python playground/calibration/fit_process_noise.py \
    playground/calibration/out/2026-08-19-garage-floor-pack-3 \
    --params playground/calibration/out/plant_params.toml
```

Before any of that, `jig_calibration.toml` needs `meters_per_count`. The tools default to
that path and refuse a zero rather than fitting speeds of zero. Push the robot between marked
points with the motors disarmed, dwelling at each, over **several distances**:

```bash
python playground/calibration/fit_encoder_scale.py \
    --run 1.0 LOG-a.TXT LOG-b.TXT LOG-c.TXT \
    --run 2.0 LOG-d.TXT LOG-e.TXT LOG-f.TXT \
    --run 3.0 LOG-g.TXT LOG-h.TXT LOG-i.TXT \
    --plot playground/calibration/out/encoder_scale.png
```

Several distances rather than one repeated, because a fixed error per pass (coasting past
the mark, backlash at the start) is a constant count offset that only shows up as an
intercept when the distance varies. The current value came out 2.1% different once that
offset was separated out.

Waveforms are declared in `waveforms.toml`, each with a `kind`, a `channel` and a `role`. Those
three fields are also what the fit routes on, so adding an excitation is a catalog edit rather
than a code change. `role = "fit"` trains the model and `role = "holdout"` validates it.

The HTML report opens with a table ranking each parameter by how poorly determined it is, mapped
to the waveform that constrains it, with a run count to reach a 10% target. Pass `--bootstrap 8`
to measure the convergence slope instead of assuming the root-N law, and `--detail all` for a
measured-against-predicted figure per run.

Two things worth knowing before a session:

- **The coupling grid must span both angular signs.** Angular droop flips with the turn direction
  and straight-line drift does not, so a one-sided grid cannot separate them and the fitter will
  refuse to report `c_ad` rather than return a contaminated number.
- **The robot arcs under a pure forward command**, because the guard plates drag asymmetrically.
  That is fitted as `c_drift` and `c_drift_bias` rather than trimmed away. Any trim applied during
  a run is logged as an angular command, never as a hidden offset.

## AprilTag overhead camera (superseded)

The plant fit `playground/control_stage0/fit_plant.py` produces from fight recordings is rough: it runs on
noisy ZED perception poses (flat-plane bias, yaw flips) and gentle driving that sits in the ESC deadzone.
These tools fixed that with a deliberate excitation run against clean AprilTag ground truth.

`apriltag_track.py`, which recorded these sessions, has been removed along with the excitation
program it drove. `analyze_apriltag_mcap.py` and `fit_plant_calib.py` still run on recordings made
before that, and `make_print_tags.py` / `make_robot_tag_3d.py` still generate the physical tags.

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
- The capture and the excitation ran in one process, so the camera frames and
  the issued commands share one `CLOCK_MONOTONIC` and the fitter does no time alignment.

## Run order

```bash
source scripts/activate_python.sh
pip install -r playground/calibration/requirements.txt   # once: DepthAI (OAK) + the rest

# 0. Print the robot tag PDF at 100% and tape it flat on the robot top. The floor grid is the manufactured
#    board, so it needs no printing (make_print_tags also writes a floor_grid.pdf for reprints only).
python playground/calibration/make_print_tags.py --out-dir playground/calibration/print

# 1. Recording step removed: apriltag_track.py is gone. Steps 2 and 3 below still run on
#    MCAPs recorded before it was removed.

# 3. Solve the field-plane poses from the recording -> the truth CSV. On a --drive recording this CSV also
#    carries the issued commands (cmd_lin, cmd_ang, ...) zero-order-held onto each frame, so it is the only
#    input the fitter needs.
python playground/calibration/analyze_apriltag_mcap.py \
    playground/calibration/out/apriltag_track.mcap \
    --out playground/calibration/out/truth_log.csv --plot playground/calibration/out/track.png

# 3. Fit the plant and write the validation plot.
python playground/calibration/fit_plant_calib.py \
    playground/calibration/out/truth_log.csv \
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
