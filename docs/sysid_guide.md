# Running a sysid session

How to collect drivetrain sysid data for Mr Stabs Mk2, watch coverage live, and turn the
recording into a fitted, validated plant installed in the robot's profiles. I drive by hand the
whole time. The tools only listen and never send drive commands. Every offline step has a plot to
check before the next one; [From recording to a fitted plant](#from-recording-to-a-fitted-plant)
walks through them.

Background: `docs/plans/sysid_coverage_monitor_plan.md` (the monitor) and
`docs/plans/mujoco_warp_mr_stabs_plan.md` (session procedure, maneuver menu). As of 2026-10-13 the
monitor has only run on simulated sessions, not against the robot.

## Hardware setup

1. Put Mr Stabs Mk2 in the 1.52 m drive-test box.
2. Mount the ZED X One S on the tripod about 0.75 m above the box floor, looking straight down,
   over the middle of the box. Connect it to the ZED Box Mini over GMSL2. The 2026-10-15 mount
   ended up 1.04 m up and 23 deg off vertical; the far side of the box then sees the tag at
   about 24 px, its decode limit, and most of that session's tag dropouts came from it.
3. Plug the radio into the ZED Box over USB.
4. Connect the ZED Box to the laptop over ethernet (`ssh -i ~/.ssh/auto-battlebot-compute-2 user@192.168.50.183`).
5. The ZED Box's second WiFi dongle joins the robot's `MR-STABS` access point. The app polls the
   robot's telemetry at `192.168.4.1:80` (`esp32_host` in the profile).
6. Leave the robot's flip switch DOWN (auto-steer on), the competition setting.

## Start the app on the ZED Box

```bash
./scripts/build_and_run.sh -c config/mr_stabs_mk2_sysid_zed_box.toml
```

This profile runs the AprilTag keypoint model (robot tags 41 and 76), records the ESP32 telemetry
ring, and uses `NoopNavigation` with autonomy off, so the trainer switch cannot put zeros on the
sticks.

Then, from the dashboard:

1. Start recording (`/command/set_recording`).
2. Put the floor marker array (3x5 GridBoard, ids 160-174) in the box and press field init
   (`/command/reinit_field`). It locks after 10 frames.
3. Take the marker array out.

## Start the coverage monitor on the laptop

```bash
source scripts/activate_python.sh
python playground/calibration/sysid_coverage.py --live ws://192.168.50.183:8765
```

Open `http://localhost:8790`. The page shows:

- a confidence ring per plant parameter (full ring = pinned to its target sigma)
- suggested stick motions, ranked by expected ring gain per second
- coverage map, transition table, correlation matrix
- a floor view with 500 ms ghost predictions
- holdout error and the estimate trace over time
- a link panel: telemetry rate, tag rate, clock-fit residual, gate counts

Check the link panel first. If telemetry or tag rate is zero, fix the WiFi link or the camera
before driving.

## Drive

Drive for a few minutes. There is no start button: the monitor picks up motions whenever they
happen. Use the suggestions to fill the rings that lag. Suggestions that need more floor than the
box has are marked floor-limited; skip those.

When done:

1. Stop the recording from the dashboard.
2. Ctrl-C the monitor. It writes the session to `data/sysid_sessions/<stamp>/`:
   - `plant.toml`: the live plant estimate
   - `coverage.json`: the last snapshot
   - `events.json`: the event log

The live estimate is a progress gauge. The calibration that ships is the offline fit on the MCAP.

## From recording to a fitted plant

Everything below runs on the laptop with no robot. Each step writes files the next step reads,
and each has a plot to check before moving on. The reference numbers come from the 2026-10-15
sessions: `00-29-00` (461 s, the training session) and `00-20-58` (307 s, held out).

```bash
source scripts/activate_python.sh
REC=data/recordings/auto_battlebot_mr_stabs_mk2_sysid_zed_box_2026-10-15_00-29-00.mcap
HOLD=data/recordings/auto_battlebot_mr_stabs_mk2_sysid_zed_box_2026-10-15_00-20-58.mcap
OUT=playground/calibration/out/mujoco_sysid
```

Record at least two sessions. The fit trains on one and is judged on another, never on windows of
its own session.

| Step | Script | Writes | Check |
| --- | --- | --- | --- |
| 1 | `check_sysid_recordings.py` | report, `<stem>.png` | PASS/WARN/FAIL, maneuver menu |
| 2 | `detect_robot_yolo.py` | `yolo.csv` | detection count |
| 3 | `smooth_tag_poses.py` | the session bundle | `yolo_calibration.png`, smoother plots |
| 4 | `plot_sysid_bundle.py` | `plots/overview.png`, `floor.png`, `dropouts.png` | aliasing, tilt, coverage, gates |
| 5 | `fit_mujoco_plant.py fit` | `fit.json`, `validation.json`, `plots/` | loss, params, rollouts, errors |
| 6 | `fit_mujoco_plant.py validate` | `validation_<name>.json`, `plots_<name>/` | the same, on other holdouts |
| 7 | `distill_mujoco_plant.py` | `distilled/plant.toml`, `plant_steps.png` | step responses, ACCEPT limits |
| 8 | `write_plant_configs.py` | the Mr Stabs profiles' `[plant]` | `--check` |
| 9 | `sim_stop_test.py` | `stop_test/results.csv` | stops on target |

### 1. Check the recordings

Copy the MCAP off the ZED Box once the app has closed it, then:

```bash
python playground/calibration/check_sysid_recordings.py $REC $HOLD --plots $OUT/checks
```

Each recording gets PASS / WARN / FAIL for file integrity, camera rate, field stability, tag
coverage, the ESP32 stream, and the sticks. Exit status is 1 on any FAIL.

Open `$OUT/checks/<stem>.png`:

- Top: the robot's track over the field, coloured by time. It should cover the box, not one
  corner.
- Middle: linear and angular stick against time. Red spans are where the track was lost for more
  than 200 ms. Long red spans over the parts you care about (punches, hard stops) mean those
  maneuvers will not make it into the fit.
- Bottom: seconds of straight hold per stick level. The menu table under the report marks every
  maneuver that is short against its target with `<- short`.

What a FAIL costs:

- `sticks` FAIL (no transmitter log, as in `00-20-58`): the fit still works, since it drives the
  model with the ESP32's `left_cmd`/`right_cmd`, but the radio delay is unknown and the menu
  cannot be scored. Fine for a holdout, not for the training session.
- `field` FAIL: the tripod moved. Positions before and after the move disagree; drive again.
- `file` FAIL: copied mid-recording. Copy it again.

### 2. Find the robot with YOLO

```bash
python playground/calibration/detect_robot_yolo.py $REC --out $OUT/2026-10-15_00-29-00_yolo/yolo.csv
python playground/calibration/detect_robot_yolo.py $HOLD --out $OUT/2026-10-15_00-20-58_yolo/yolo.csv
```

This runs the deployed pose engine over every video frame (about 6 minutes for 27,645 frames on
the RTX 4080 laptop) and writes every `MR_STABS_MK2` box with its front and back keypoints. The
smoother uses these only where the tag drops out.

Why it is needed: the tag decodes down to about 24 px. With the camera tilted, a robot heading
away from it shows the 11 degree nose-down tag foreshortened below that, and the tag drops out
while the robot sits flat and in plain view. On `00-29-00` that was 18 dropouts, 22.6 s. Once
the tag is lost the app searches the whole frame and falls from 60 fps to about 12, so most
video frames in a dropout are never searched at all. YOLO finds the robot in 94.5% of those
frames.

The model also labels the 3x5 floor marker board as the robot. The next step drops those
detections; you do not need to clean anything by hand.

### 3. Smooth the session

```bash
python playground/calibration/smooth_tag_poses.py $REC --session-name 2026-10-15_00-29-00_yolo \
    --yolo $OUT/2026-10-15_00-29-00_yolo/yolo.csv --plots
python playground/calibration/smooth_tag_poses.py $HOLD --session-name 2026-10-15_00-20-58_yolo \
    --yolo $OUT/2026-10-15_00-20-58_yolo/yolo.csv --plots
```

Leave out `--yolo` to smooth on tags alone (`..._tag` bundles); keep one of those around to
compare against. The bundle lands in `$OUT/<session-name>/`: `smoothed.csv` (pose on a 5 ms
grid), `measurements.csv` (one row per tag or YOLO fix, `source` says which), `commands.csv`,
`windows.csv` (the 1 s fit windows), `session.json`.

What the smoother does, in order:

1. Turns each tag detection into an axle-centre pose and smooths it with an RTS smoother.
2. Fuses the robot's own BNO055 yaw rate, shifted by its measured lag (about 30 ms) and scaled
   by its measured gain. Without it a 1 s dropout can leave the yaw rate locked onto a whole turn
   per frame, which matches every wrapped tag fix and never recovers (2026-10-14: 78% of samples
   above 50 rad/s).
3. With `--yolo`, projects the keypoints onto the floor, calibrates that projection against the
   tag on frames where both see the robot, and adds the result as position fixes inside tag
   dropouts while the IMU says the robot is flat. It adds the IMU's pitch and roll there too, so
   the tilt state does not drift and mislabel flat windows as nose lifts.
4. Cuts the session into 1 s windows: flat or nose-lift by pitch, dropped near the rails, across
   a flip, or under 70% coverage.

The printed summary, with the 2026-10-15 values:

```text
  yolo: ... dropped_by_size_gate 151, video_frames_in_dropouts 1952, dropped_not_flat 77,
        fixes 1875, accepted 1852, rejected 23
  detections 19168, rejected 366, IPPE close calls 225
  noise floor x 2.64 mm, y 4.24 mm, yaw 0.46 deg
  windows {'flat': 227, 'nose_lift': 151}, dropped {'span_too_short': 15, 'coverage': 5}
```

Plots in the bundle directory:

- `yolo_calibration.png` (with `--yolo`). Left and middle: box size and front-back keypoint
  spacing of every confident detection, with the gates dashed. The marker board is the separate
  cluster near 380 px; it must sit outside the box-size gate. Right: YOLO minus tag on co-visible
  frames, with the fitted offset, plane height and axle offset in the title. Expect a median under
  10 mm (2026-10-15: 8.5 mm and 3.9 mm) and the same offset, plane and axle on every session
  (-25.2 and -25.4 ms, -23 and -27 mm, -46 and -48 mm). A session that disagrees has a camera
  calibration or clock problem, not a YOLO problem.
- `imu_vs_tag_yaw_rate.png`: the IMU and smoothed yaw rate on top of each other through spins.
  They should overlap. Check `session.json` `imu_yaw_rate`: `correlation` above 0.9, `lag_s`
  near 0.03, `linear_gain` near 1.
- `still_raw_vs_smoothed.png`: raw tag poses against the smoothed track on the longest still
  stretch. The smoothed line should sit in the middle of the raw scatter without following it.
- `residual_autocorrelation.png`: residuals should drop to the band within a few lags. A slow
  decay means the process noise is too low for this driving.

### 4. Look at the smoothed bundle

```bash
python playground/calibration/plot_sysid_bundle.py $OUT/2026-10-15_00-29-00_yolo \
    --compare $OUT/2026-10-15_00-29-00_tag
```

This writes three figures to `$OUT/2026-10-15_00-29-00_yolo/plots/`.

`overview.png` is the whole session on one time axis:

- A: accepted fixes by source. Gaps in both rows are dropouts nothing filled (flips, the robot
  under a hand or out of frame).
- B: smoothed and IMU yaw rate, log scale. The smoothed rate should trace the IMU. A flat shelf
  at hundreds or thousands of rad/s is the aliasing failure; the title gives the share of samples
  above 50 rad/s (2026-10-15: 2.2%, almost all inside the 12 s upside-down blackout at 77 s,
  where neither tags nor the IMU can see).
- C: pitch against the grey flat band the window gate uses. Short spikes are wheelies and
  punches. Long stretches outside the band while the robot sat flat mean the tilt state drifted
  through a dropout.
- D: x and y against the near-rail limits. Time spent past the dashed lines produces no windows.
- E: the windows, flat and nose-lift, with the drop counts.

`floor.png` is the session top-down: every tag fix, every YOLO fix, the smoothed path, and the
box the window gate assumes. If many fixes sit outside that box, the box is not where the
gate thinks it is: rerun step 3 with `--box-center X Y --box-half-extent H` read off this plot.
On 2026-10-15 fixes reach y = +1.1 and -1.0 m against the default +/-0.76 m box, which costs
57 s of driving to the near-rail gate.

`dropouts.png` has one panel per tag dropout longer than 0.3 s with the robot upright, 0.5 s
either side. Blue dots are tag fixes, yellow YOLO fixes, green the smoothed path, orange the
`--compare` bundle's. The green path should run through the fixes. A loop or a line shooting off
between them is the smoother extrapolating with nothing to hold it: look for panels with few
YOLO fixes (a hand over the robot) or a fast spin.

To put a number on how much YOLO helps, hide the tag in tracked stretches and see how close each
smoother gets back to it. Across 40 hidden 1 s spans on `00-29-00`:

| Error over the hidden span | Tag only | Tag + YOLO |
| --- | --- | --- |
| position, median | 88 mm | 8 mm |
| position, worst in span (median over spans) | 182 mm | 24 mm |
| heading, median | 0.9 deg | 0.9 deg |
| heading, worst in span (90th percentile) | 4.7 deg | 4.7 deg |

### 5. Fit the MuJoCo plant

The fit runs MuJoCo Warp on the laptop's GPU. Run it directly.

```bash
python playground/calibration/fit_mujoco_plant.py fit $OUT/2026-10-15_00-29-00_yolo \
    --holdout $OUT/2026-10-15_00-20-58_yolo \
    --max-windows 64 --popsize 64 --generations 25 --restarts 2 --zero-mode both \
    --out $OUT/fit_2026-10-15_yolo
```

The first line it prints is the batch and its cost:

```text
64 windows (32 nose-lift) from 1 sessions; 4096 worlds per generation; up to 1.64e+09 rollout-steps
```

At most half the batch is nose-lift windows. These settings
take about 27 minutes on the RTX 4080 laptop; the defaults (popsize 128, 40 generations,
3 restarts) are about five times that.

The fit runs CMA-ES in four stages, each freeing a group of parameters: `straight`, `turning`,
`nose_lift`, then `joint` (everything). Each restart is a fresh start from a random point; with
`--zero-mode both`, half the restarts assume the ESCs brake at zero command and half that they
coast. The log prints the best and median loss per generation.

With `--holdout` the fit validates at the end. `fit_2026-10-15_yolo/` then holds `fit.json`,
`validation.json` and `plots/`:

- `plots/loss.png`: best loss per generation for every restart, stage by stage (dashed lines).
  Curves still falling at a stage's end want more `--generations`. Restarts that finish at very
  different losses found different basins: trust the lowest only if `params.png` shows it is not
  sitting on a bound.
- `plots/params.png`: each restart's best parameters as a position inside its search range, the
  best run starred with its value. Parameters the restarts agree on are identified by the data. A
  parameter scattered across its range is not, and its value means nothing. A parameter stuck at
  0 or 1 is fighting its bound: either the range is wrong or the model is missing something.
- `plots/rollouts.png`: held-out windows top-down, picked at the best, median and worst MuJoCo
  error for flat and nose-lift windows. Blue and yellow dots are the measured robot (tag, YOLO),
  green the MuJoCo rollout, orange the grey-box. Both models start from the window's measured
  state and get the logged motor commands. Good: green runs through the dots for the whole second.
  A rollout that bends the wrong way points at the turning parameters, one that runs long or short
  at the top speed or the stop.
- `plots/errors.png`: position and heading error at 0.2, 0.4 and 1.0 s, as multiples of the
  measurement noise (dashed line at 1). The MuJoCo boxes should sit at or below the grey-box ones.

`validation.json` holds the plan's six checks
(`docs/plans/mujoco_warp_mr_stabs_plan.md`, "Validation"):

| Key | Passes when |
| --- | --- |
| `check1_2_errors.check1_vs_greybox` | `mujoco_beats_or_matches` at 0.4 s and 1.0 s (MuJoCo median position within 5% of the grey-box or better) |
| `check1_2_errors.*.position_median_x_noise` | read it: how many noise floors off the model is |
| `check3_plausibility` | every parameter `ok` (resistance 0.05 to 1 ohm, efficiency 0.6 to 0.95, wheel slide friction 0.5 to 2.5, skid 0.05 to 1, armature 0.8 to 1.2); `total_delay_s.near_60ms` true; `coast_tau_s` near the measured 0.078 s |
| `check4_nose_lift` | `both_lift` and `neither` dominate; `real_only` and `sim_only` small |
| `check5_firmware` | `pid_output_rms_percent` and `motor_command_rms_percent` small: the simulated firmware reproduces the logged commands |
| `check6_identification` | restart `losses` close together; `tradeoff_pairs` lists parameters that trade against each other |
| `check7_closed_loop` | read it: errors per drive direction (forward, reverse, spin, other), open loop and with the firmware yaw loop run around the sim from the logged sticks |

Score reverse driving on `check7_closed_loop.closed_loop`, not on the open-loop numbers. When
the stick is centred, the yaw loop holds heading against a reverse veer the plant does not have.
On 2026-10-15 it backed up straight with 1.12 to 1.6 times more left than right command, growing
with throttle. Open-loop replay feeds that correction to a sim that has no veer, so the sim turns
and the real robot didn't: 12.8 deg median heading error at 0.4 s on the `00-20-58` holdout.
With the loop closed it is 5.3 deg. Fitting per-motor reverse gains to chase the open-loop error
improved the training session and doubled the holdout's reverse position error, because the veer
changes between sessions.

The fit keeps the restart with the lowest training loss, even when the brake and coast restarts
tie. Look at both. On 2026-10-15 the tag + YOLO fit's best coast restart (54.30) beat its best
brake restart (54.39) by 0.09, and it was the worse model: a 0.80 s coast time constant against
the measured 0.078 s, and it lost to the grey-box on held-out position. To score one zero mode on
its own, copy `fit.json` into a new directory keeping only that mode's `runs` and run step 6 on
it.

2026-10-15 reference (trained on `00-29-00`, 64 windows, scored on `00-20-58`). Median position
error on held-out windows; heading in brackets:

| Model | Flat 0.4 s | Flat 1.0 s | Nose-lift 0.4 s | Nose-lift 1.0 s |
| --- | --- | --- | --- | --- |
| grey-box | 17.1 mm (2.8 deg) | 24.9 mm (5.2 deg) | | |
| MuJoCo, tag-only bundle, brake | 11.8 mm (3.0 deg) | 23.3 mm (6.6 deg) | 36.1 mm | 116 mm |
| MuJoCo, tag + YOLO bundle, coast (picked) | 18.3 mm (3.8 deg) | 36.1 mm (2.9 deg) | 45.0 mm | 85 mm |
| MuJoCo, tag + YOLO bundle, brake | 13.0 mm (2.4 deg) | 15.6 mm (9.6 deg) | 25.4 mm | 83 mm |

Scored on the tag-only holdout bundle, 16 to 28 flat windows per horizon and 8 to 10 nose-lift,
so differences of a few millimetres are noise. With the zero mode held to brake, the YOLO-filled
training session gives the better plant at 1 s and on nose lifts, and it is the only fit with
every parameter in its plausible range. All fits leave `delay_s` near zero and several
parameters on a bound (`params.png`), and the joint stage is still improving at generation 25:
more generations and a second training session are the next things to try.

### 6. Score the fit on other sessions

```bash
python playground/calibration/fit_mujoco_plant.py validate $OUT/fit_2026-10-15_yolo \
    $OUT/2026-10-15_00-20-58_tag --name holdout_tag
```

`--name` writes `validation_<name>.json` and `plots_<name>/` beside the default ones. Score every
fit against every holdout you have; a plant that only predicts the session it was fitted on has
learned that session's battery and floor.

### 7. Distill the grey-box table

The app cannot run MuJoCo. Its EKF and `MotionProfileNavigation` run the grey-box `[plant]`
table: deadzones, gains, accel and decel time constants, delay, steer-brake coupling. Distilling
drives the fitted MuJoCo model with stick tapes and fits that table to what it does.

```bash
python playground/calibration/distill_mujoco_plant.py $OUT/fit_2026-10-15_yolo \
    --holdout $OUT/2026-10-15_00-20-58_yolo
```

It writes `fit_2026-10-15_yolo/distilled/plant.toml`, `distill.json` and `plant_steps.png`, and
prints held-out position error per horizon for the distilled table, the current config's table,
and a grey-box fitted straight to the data. It warns `sim holdout misses ...` when the distilled
table misses any of 15 mm at 100 ms, 80 mm at 400 ms or 8 deg heading at 400 ms against MuJoCo.

`plant_steps.png` compares the distilled table (solid) with the one the profiles carry now
(dashed):

- Top left: forward speed after a linear stick step and release at 0.1, 0.25, 0.5 and 1.0. The
  flat start is the delay, the curve up is the accel time constant, the plateau the top speed at
  that stick, the fall after release the decel time constant.
- Top right: the same for yaw rate.
- Bottom: steady speed and yaw rate against stick. The flat middle is the deadzone; the ends are
  top speed each way.

Before 2026-10-15 the Mr Stabs profiles carried a placeholder table: zero deadzones, zero time
constants, zero delay. It draws as square steps. A distilled table should show a real delay, a
rise over tens of milliseconds, and a deadzone.

Also read the `pitch-gated tapes` list it prints: the share of each stick tape dropped because
the MuJoCo robot pitched up. A tape that loses most of its samples teaches the table nothing about
that command, and the table only covers what survives.

Distill clamps a negative radio delay to zero and says so. A stick move cannot reach the ESP32
before it is sent, so a negative `radio_delay` in `session.json` is a clock-fit error; on
2026-10-15 it was -9.5 ms.

2026-10-15 reference, both tables distilled with a 0 ms radio delay:

| | tag-only fit | tag + YOLO fit, brake |
| --- | --- | --- |
| sim holdout, pos at 100 ms (limit 15 mm) | 19.6 mm | 28.5 mm |
| sim holdout, pos at 400 ms (limit 80 mm) | 69.6 mm | 114 mm |
| sim holdout, heading at 400 ms (limit 8 deg) | 24.6 deg | 25.0 deg |
| held-out session, pos RMS at 1.0 s, distilled | 86.8 mm | 86.9 mm |
| held-out session, pos RMS at 1.0 s, current config | 99.6 mm | 114 mm |
| forward step tapes dropped as wheelies | 80 to 81% | 76 to 79% |
| `k_fwd`, `tau_lin_a`, `tau_lin_d`, `delay_s` | 5.60, 0.172 s, 0.061 s, 0 | 5.73, 0.183 s, 0.068 s, 0.016 s |

Both tables beat the placeholder on the held-out session and both miss the sim-holdout limits:
the grey-box form does not reproduce the MuJoCo model within them, mostly because the forward
steps the table would learn from are wheelies. Do not install a table that misses the limits.

### 8. Install the table

```bash
python playground/calibration/write_plant_configs.py --robot mr_stabs_mk2 \
    --fit $OUT/fit_2026-10-15_yolo/distilled/plant.toml --check
python playground/calibration/write_plant_configs.py --robot mr_stabs_mk2 \
    --fit $OUT/fit_2026-10-15_yolo/distilled/plant.toml
```

`--check` lists every field that would change (`path: field old -> new`) and writes nothing.
Without it the script rewrites the numbers in the six Mr Stabs profiles that restate `[plant]`
and leaves their comments alone. Rebuild afterwards.

### 9. Check the stop in simulation

```bash
./scripts/build.sh
python playground/calibration/sim_stop_test.py \
    --fit-file $OUT/fit_2026-10-15_yolo/fit.json \
    --arm current=config/mr_stabs_mk2_aer_zed_box.toml \
    --arm distilled=$OUT/fit_2026-10-15_yolo/distilled/plant.toml
```

The real binary drives `MotionProfileNavigation` from each arm's table while the sim moves the
robot with the fitted MuJoCo model, to the same set of goals. Each run prints where the robot
stopped along the approach (`stop_along`, negative is short), how far past it got (`peak_along`),
when it arrived, and how often it left the stop circle again. The per-arm summary counts stops
within `--stop-distance` (0.15 m). The distilled arm should stop on target more often and with a
smaller mean miss than the current one. `FLIPPED` means the punch flipped the robot; lower
`--max-linear-command` rather than blaming the table. On the 2026-10-15 fit every arm flips at
the default 0.8, every arm stops within 2 cm at 0.3, and 0.5 separates them: 2/6 on target for
the placeholder table, 5/6 for the distilled one.

### 10. On the robot

Drive a new session with the installed table and put it through steps 1 to 4. Then score the fit
on it with step 6 (`--name <date>`). If the errors hold up on a session the fit never saw, the
plant is done; if they grow, compare `plant_steps.png` and the battery voltage and floor of the
two days.

## Replay without the robot

```bash
# a recorded session at 8x real time
python playground/calibration/sysid_coverage.py --replay rec.mcap --speed 8

# as fast as possible, no page
python playground/calibration/sysid_coverage.py --replay rec.mcap --speed 0 --no-serve

# write a simulated session, then replay it
python playground/calibration/sysid_coverage.py --synthetic /tmp/sysid_sim.mcap --seconds 150
```

The replay page stays up after the recording ends; Ctrl-C writes the session and exits.

Drives recorded on a match profile have no tags. Replay their video through
`config/playback/mr_stabs_mk2_sysid_playback.toml` first, one recording at a time.

## Troubleshooting

- Tag size: `tag_size_m = 0.064` is the CAD value. Measure the printed tag and correct it in both
  `config/mr_stabs_mk2_sysid_zed_box.toml` and the playback profile before the first session.
- Field shifted mid-session: the checker flags more than 5 mm or 0.5 deg of field wander. That
  means a bumped tripod or a re-init. Re-init the field and drive again.
- Truncated MCAP: a missing summary means it was copied while still recording. Copy it again.
- Tag dropouts while the robot sits flat: the tag is near its decode limit. Check the tag size
  gradient in `floor.png` and the tag pixel size in step 1. Mount the camera straight down over the
  box centre, use a bigger tag, or shorten the exposure; YOLO in step 2 covers what is left.
- Smoothed yaw rate shelf in `overview.png` B: the IMU did not fuse. Check `session.json`
  `imu_fused` and `imu_yaw_rate` (correlation, lag, gain).
- Many windows labelled nose-lift while the robot sat flat: the tilt state drifted through a
  dropout. With `--yolo` the IMU's pitch holds it; check `session.json` `imu_tilt`.
