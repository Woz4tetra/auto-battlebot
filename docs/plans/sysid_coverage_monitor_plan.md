# Sysid coverage monitor plan

Goal: a live tool that shows how much of the Mr Stabs Mk2 drivetrain plant has been characterized
while I drive by hand, the way Face ID enrollment shows how much of a face it has seen or OptiTrack
shows wand coverage. I drive however I like. The tool reads the sticks, ESP-NOW telemetry and the
AprilTag poses, labels each motion, refits the grey-box plant as windows arrive, and shows which
parameters are pinned, which are stuck, and what motion would help next.

Two hard rules:

1. **The tool never sends a drive command.** Autonomous excitation is too likely to hit something.
   The tool is a read-only subscriber, and the profile it runs against transmits nothing.
2. **No interaction is required.** No start button, countdown or prompt to follow. Suggestions are
   optional and the tool picks motions up whenever they happen.

Background:

- Interactive prototype (demo data, JS port of `plant.py`):
  https://claude.ai/artifact/GDB7P9TwfMUegQcrSdrN8m. This plan builds the real version of it.
- `docs/plans/espnow_link_plan.md` builds the telemetry this tool reads.
- `docs/plans/mujoco_warp_mr_stabs_plan.md:106-124` and `:285-315` hold the sysid session
  procedure and maneuver menu this tool replaces with live feedback.
- `docs/experiments/kalman_filter/kalman_filter_plan.md` §1.6 defines the windowed open-loop
  scoring and the M0 to M6 ladder reused here.

Written 2026-10-13 against `benw/zed-box` at `d8c74c0f`.

### Status (2026-10-13)

Steps 1 to 7 are built and tested on a simulated session; nothing has run against the robot.

| Piece | Where |
| --- | --- |
| Batched mixer and yaw loop mirror, closed-loop rollout | `auto_battlebot/control/firmware_mixer.py` |
| Telemetry columns from the schema, plant inputs, live clock | `auto_battlebot/calibration/sysid/telemetry.py` |
| Profile loader (`extends` chain) | `auto_battlebot/calibration/sysid/profile.py` |
| Segmenter, plus the offline menu moved out of the checker | `auto_battlebot/calibration/sysid/maneuvers.py` |
| Causal tag poses and window start states | `auto_battlebot/calibration/sysid/pose_stream.py` |
| Residuals, Fisher, fit, holdout, suggestions | `auto_battlebot/calibration/sysid/model.py` |
| Streaming session | `auto_battlebot/calibration/sysid/coverage.py` |
| Refit engines (inline, process) | `auto_battlebot/calibration/sysid/worker.py` |
| Replay and live sources | `auto_battlebot/calibration/sysid/sources.py`, `auto_battlebot/recording/foxglove_ws.py` |
| Page and its server | `auto_battlebot/calibration/sysid/static/index.html`, `server.py` |
| Simulated session in the recorder's formats | `auto_battlebot/calibration/sysid/synthetic.py` |
| CLI | `playground/calibration/sysid_coverage.py` |
| Tests | `tests/python/test_sysid_coverage.py` |

On a 150 s simulated drive in the 1.52 m box (replayed from MCAP, `--speed 0`, 41 s on the
dev box): 47 windows used, 11 held out, 41 dropped at the rails. The ring reached 96% and the
500 ms closed-loop holdout error 0.46 cm. A 120 s run lands most parameters within 1σ of the
simulated truth; delay reads 4 ms short, which is the minimum radio latency that the clock map
cannot separate (see Clock).

Where the build differs from the design as first written:

- The telemetry loader lives in `calibration/sysid/telemetry.py`, not `sysid_io.py`, because
  another agent was editing `sysid_io.py` at the time. It reads the column list from the
  recorder's schema, so there is no separate parity list to keep in step. It falls back to
  `/robot/esp32_diagnostics` for recordings without ESP-NOW telemetry.
- No fixed-lag RTS smoother. Each 0.5 s sub-window starts from a weighted quadratic fit to the
  tag poses within 0.15 s of its start, and the fit then treats that start state (x, y,
  heading, speed) as a nuisance with a prior: one Gauss-Newton step per sub-window, and the
  Jacobian projected so the start state cannot fake information (Schur complement). Without
  that, start-state errors biased the gains and lags by 4 to 13σ on simulated data.
- Delay is a continuous fit parameter, not profiled on a grid: commands shift by a linearly
  interpolated fractional delay, which keeps the residual smooth in it.
- The wall gate uses `sysid_windows`' 0.15 m robot length, not 0.10 m, so A7 compares like
  with like.
- The floor run for suggestions is the field diagonal minus 0.6 m (a robot length and a stop
  at each end), not the side minus 0.9 m, which left nothing in a 1.52 m box.

## Requirements

1. Read-only. The tool opens no write path to the robot, the dongle or the relay command topics.
   Data recorded while the Jetson could have been commanding (`espnow_active` or
   `autonomy_enabled` true) is excluded and flagged on screen.
2. Live: the coverage map, edge table and ticker update within 0.5 s of the motion finishing; the
   ring and correlation matrix update as each fit window closes; full refits land at least every
   5 s during a 15 minute session.
3. The same pipeline runs on a recorded MCAP at faster than real time. Replay is how it is
   developed and tested.
4. The final calibration stays the offline pipeline (smoothed tag poses, `jig_fit` joint fit).
   The live fit's job is coverage and a stopping criterion, not the shipped parameters.
5. Parameters that the floor is too small to identify are marked as floor-limited instead of left
   looking unfinished.

## What exists

| Item | State | Where |
| --- | --- | --- |
| Hand-driven sysid profile, transmits nothing | Done | `config/mr_stabs_mk2_sysid_zed_box.toml` (`NoopNavigation`, `autonomy_enabled_by_default = false`) |
| Composite transmitter: OpenTX primary + ESP-NOW | Built, unit tested | `src/transmitter/composite_transmitter.cpp` |
| ESP-NOW telemetry on `/robot/espnow_telemetry`, 100 Hz | Built, tested on the robot | `src/transmitter/espnow_transmitter.cpp:184-192`, `docs/foxglove_recording_format.md:211-247` |
| Telemetry payload | `a/b_percent`, `left/right_cmd`, `yaw_rate`, gravity vector, Euler, `vbat/ibat`, arm/flip/autonomy flags, `espnow_active`, `timestamp_ms` | `firmware/shared/espnow_protocol/src/espnow_protocol.h:58-85` |
| Dongle pings every 10 ms, so telemetry flows with no Jetson commands | Done | `espnow_protocol.h:129` |
| Stick readback | OpenTX channels on `/diagnostics/opentx_transmitter`; `/status/sticks` at 20 Hz | `src/transmitter/opentx_transmitter.cpp:289-292`, `src/runner.cpp:684-689` |
| Tag poses live and recorded | `/apriltag/robot_tags`, both IPPE solutions per frame | `src/keypoint_model/apriltag_keypoint_model.cpp:20` |
| Everything above is live on the relay | ws:8765, one app, many clients | `src/viz_relay/main.cpp`, `include/publisher/output_channel.hpp` |
| Grey-box plant, windows, horizon scoring | Done | `auto_battlebot/control/plant.py` (`simulate:344`, `make_windows:611`, `predict_windows:712`, `MODEL_LADDER:201`) |
| Joint fit with delay profiling | Done | `auto_battlebot/calibration/jig/jig_fit.py` (`residual_vector:933`, `joint_fit:943`, `profile_delay:994`) |
| Firmware mixer and yaw loop mirror | Done, scalar | `auto_battlebot/mujoco_sim/firmware.py` (`YawController`, `FirmwareMixer`) |
| Robot-to-host clock map | Done | `auto_battlebot/recording/esp32_clock.py:133` (`fit_robot_clock`) |
| Tag pose smoother | Offline, non-causal RTS | `auto_battlebot/perception/tag_pose_smoother.py:1195` |
| Sysid window gates (rail, gaps, flip) | Done, reads `/robot/esp32_diagnostics` | `auto_battlebot/perception/sysid_windows.py` |
| Offline maneuver menu and session targets | Done, playground only | `playground/calibration/check_sysid_recordings.py:413` (`maneuver_menu`), `:97` (`MENU_TARGETS`) |
| Match maneuver classes | Done | `auto_battlebot/calibration/match/match_windows.py:564` |
| `/robot/espnow_telemetry` reader | **Missing** | `sysid_io.py` reads only `esp32_diagnostics` |
| Python Foxglove WS client | **Missing** | `websockets` 17.1 is in the venv |

The espnow plan's Phase 2 deletes `/robot/esp32_diagnostics` and moves the sysid loaders to
`/robot/espnow_telemetry` (`espnow_link_plan.md:835-836`). This tool is built on ESP-NOW
telemetry only, so it does that move for the loaders it touches.

## Design

```
ZED Box (sysid profile, transmits nothing)
  OpenTX readback ─┐
  ESP-NOW dongle ──┼─► app ─► viz_relay ws:8765 ─────────────┐
  camera + tags ───┘                                         │
                                                             ▼
Laptop: venv/bin/python playground/calibration/sysid_coverage.py --live ws://<box>:8765
  ┌──────────────────────────────────────────────────────────────────────────────────┐
  │ source (live WS client | MCAP replay)                                            │
  │   └─► sample assembler: clock-mapped telemetry + causal tag pose, 50 Hz grid     │
  │         ├─► maneuver segmenter ──► ticker, coverage map, edge table              │
  │         ├─► gates (wall, flip, autonomy, gaps, tag coverage)                     │
  │         └─► window builder (1.5 s) ─► info accumulator ─► ring, correlation      │
  │                                   └─► refit worker (process) ─► θ, ladder,       │
  │                                                     holdout, suggestions         │
  │   state snapshot @ 4 Hz ─► local page (http://localhost:8790, websocket JSON)    │
  └──────────────────────────────────────────────────────────────────────────────────┘
```

Running on the laptop keeps all of it off the ZED Box and out of the perception loop. The relay
already fans out to many clients, so the web dashboard keeps working alongside it.

### 1. Plant input and the firmware yaw loop

The plant models the firmware yaw loop. With the flip switch down, `mix_motor_outputs`
(`firmware/mr_stabs_mk2/src/main.cpp:219-264`) runs `yaw_control::YawController` on every BNO055
sample and replaces the steer stick with its output. The command the Jetson sends is a stick, so
the plant the Jetson needs runs from sticks through the loop to the wheels:

```
a/b_percent ─► firmware mixer (yaw loop when flip switch down) ─► left/right_cmd ─► drivetrain
                       ▲                                                             │
                       └──────────── heading, gyro yaw rate ◄────────────────────────┘
```

- **Fit input:** `left_cmd/right_cmd` from telemetry, converted to the plant's normalized linear
  and angular command. They are what the ESCs got, after the deadband and the loop, at 100 Hz on
  the robot clock. Fitting from them identifies the 16 drivetrain parameters the same way with
  the loop on or off, and keeps the loop's feedback out of the Fisher matrix.
- **Prediction input:** `a/b_percent` and the flip switch through a vectorized mirror of the
  mixer and `YawController`, closed around the simulated heading and yaw rate. The ghost
  predictions, the holdout score and the suggestion ranking all use this path, so they score the
  plant the Jetson would command. The controller gains are firmware constants
  (`yaw_control::Config`), not fit parameters.
- `auto_battlebot/mujoco_sim/firmware.py` already mirrors both line for line, but as scalar
  Python stepping one robot. The fit needs batched rollouts, so a numpy version over the batch
  dimension goes into `auto_battlebot/control/firmware_mixer.py`, with a parity test against the
  scalar mirror. The scalar one stays for the MuJoCo closed loop.
- **Parity on real data:** the telemetry carries `pid_output` and `yaw_rate_cmd`. Running the
  mirror on the recorded sticks, heading and gyro must reproduce them sample for sample, apart
  from the 10 ms sample phase. That pins the mirror before any fit depends on it.

Step 1 also cross-checks `a/b_percent` against the OpenTX channel readback on one recording, for
sign and scale.

The C++ filter mirrors `plant.py` and predicts from the command it sends, so it needs the same
loop mirror for matches run with the flip switch down. That belongs to the kalman plan; this plan
only builds the Python side.

### 2. Measurements

| Channel | Source | Rate | Used for |
| --- | --- | --- | --- |
| Position, heading | `/apriltag/robot_tags` through a causal pose filter | camera rate | windowed fit, wall gate |
| Yaw rate | telemetry `yaw_rate` (BNO055, deg/s clockwise) | 100 Hz | fit; keeps the yaw terms identified through spin blur |
| Up/down, nose lift | telemetry gravity vector | 100 Hz | flip gate, nose-lift gate |
| Battery | `vbat`, `ibat` | 100 Hz | sag annotation; a session-level gain check |
| Arm, link, autonomy | telemetry flags | 100 Hz | gates |

The prototype showed that heading has to be in the residual: with velocity and yaw rate alone the
two drift terms never separated and coverage stalled at 93%. The live fit uses position, heading
and gyro yaw rate.

The offline RTS smoother is non-causal. The live path (`pose_stream.py`) runs each frame
through the offline `build_candidates` and `select_ippe` and finalizes a detection once it is
older than `select_ippe`'s 0.25 s look-ahead. Fit windows start from a local fit to those poses,
with the start state profiled out of the fit (see Status). The session processes everything
0.5 s behind the newest data, so every window has the tags around it.

### 3. Clock

Telemetry carries `timestamp_ms` on the robot clock and is logged at `host_receive_ns`.
`esp32_clock.fit_robot_clock` builds the lower-envelope robot-to-host map offline. The live
version refits that map over a sliding 60 s buffer every 5 s; tag frames are already on the host
clock (`log_time = image_stamp_ns`). The fit's delay term absorbs any constant offset, and the
tool shows the clock-fit residual so drift is visible.

The lower envelope rests on the fastest deliveries, so the minimum radio latency stays in the
map: every telemetry stamp reads that much late. For the commands the delay term absorbs it
(the fitted delay is short by the minimum latency). For the gyro it is a small misalignment
against the tags. Commands are held onto the 10 ms grid; the gyro is interpolated, or a held
gyro reads up to one frame late against the tags.

### 4. Maneuver segmentation

The ticker labels come from plateau and transition detection on the stick stream, ported from the
prototype:

- Plateau: a maximal run where each channel stays within ±0.06 (a held thumb's jitter).
- Hold: a plateau of 0.5 s or longer. A throttle hold with steer under 0.12 is a straight hold;
  with more steer it is an arc. A steer hold with throttle under 0.12 is a spin.
- Edge: a transition between holds, or out of rest. It is clean when it finishes in 80 ms or less,
  and only clean edges count for delay and lag. Slower ones get "slewed, snap it next time."
- Creep: a monotonic climb out of zero past 0.12 that takes 0.8 s or longer. It informs the
  deadzones.

`check_sysid_recordings.maneuver_menu` and `match_windows.maneuver_classes` already label the same
things with their own thresholds. Per the "second caller moves to `auto_battlebot/`" rule, the
segmenter goes in `auto_battlebot/calibration/sysid/maneuvers.py`, and `check_sysid_recordings.py`
switches to it, keeping `MENU_TARGETS` as the offline session check.

Every label carries the flip switch state. Loop-on windows go into the fit like any other, since
the fit input is the motor command. The loop does add motions worth seeing on the coverage map:
a steer release (the capture brake), a centered-stick reverse run (heading hold on the unstable
tail-first direction) and a steer snap while reversing (feedforward cut, feedback gain raised).
They get their own rows in the transition table.

### 5. Gates

A window is dropped from the fit, shown in red on the tape and listed in the ticker, when any of:

- **Wall:** the axle center is within 0.15 m of a rail (the `sysid_windows.near_rail` gate), or,
  from the third refit on, any tag residual is over 4σ (scaled by the fit residual) against the
  current fit. In the prototype, wall contact
  came straight from the simulator; here those two signals have to do that job.
- **Flip or nose lift:** the robot or a tag reads upside down, or the tag pitch leaves the rest
  pitch by more than 4 degrees (the `sysid_windows` limit).
- **Jetson could command:** `espnow_active` or `autonomy_enabled` true. This should never fire in
  the sysid profile; if it does, a banner says so.
- **Link or tag gaps:** telemetry missing for over 50 ms, counted from `telemetry_seq` (lost
  frames) or from `timestamp_ms` (late ones), or tag coverage under 0.7 in the window. One or
  two lost frames are held over and cost nothing.
- **Disarmed:** `armed` false. The window counts as idle.

### 6. Information and refit

- Each 1.5 s window is three 0.5 s sub-windows, each run open loop on the motor commands from
  its start state. Residuals: tag position along and across the camera view and tag heading at
  each fix (at most 16 per sub-window, sigmas floored at 1 cm and 0.03 rad for model error),
  and gyro yaw rate every 20 ms over 0.3 rad/s.
- Each closed window adds `JᵀJ` to a running Fisher matrix, with J the Jacobian of those
  residuals with respect to the 16 `PlantParams` at the current estimate, from batched finite
  differences through `plant.simulate`.
- The covariance is `s² (JᵀJ + prior)⁻¹`, with the stage 2 values as the Gaussian prior and
  `s²` the residual variance per row when it exceeds 1, so a model that cannot reach the noise
  floor reports wider bars. Each
  parameter's ring fill is `min(1, target_σ / σ)`. The targets are hard-coded in the module with
  a comment per value: 4 ms for delay, 6 ms for each lag, 3% of gain for the gains, 0.008 for
  each deadzone, 0.03 for coupling, 0.12 rad/s for drift.
- Refits run in a worker process every 4 s of driving. It does a warm-started soft-L1
  `least_squares` over at most 300 used windows (thinned evenly), then recomputes the Fisher
  matrix over all of them at the new estimate. Every third refit also fits each rung of
  `MODEL_LADDER`.
- **Holdout:** every fifth usable window, interleaved in time, is held out. Holdout error is the
  500 ms closed-loop position error on those windows, from the sticks through the mixer. This replaces the prototype's three fixed
  maneuvers, which a real session would not contain.
- **Suggestions:** a catalog of stick motions (`model.CATALOG`: steps, spins, creeps, arcs,
  taps) is simulated at the current estimate through the mixer, with the flip switch where it
  is now, and each motion is scored by expected ring gain per second. A motion whose predicted
  travel exceeds the floor run (field diagonal minus 0.6 m) is not suggested. A parameter whose remaining gain only comes from motions that do not fit is marked
  floor-limited. The floor run comes from the profile, so a bigger box raises the limit with no
  code change. The 0.5 s sub-windows do not need a plateau, so on simulated data the gains
  filled even in the 1.52 m box; whether real data does the same is a step 8 question.

### 7. Page

The page is the prototype's layout, served by the tool itself:

- live stream: gimbals, an 8 s tape with window status, the event ticker, the hint line
- parameter confidence ring and parameter list
- suggestions
- coverage map
- transition table
- correlation matrix
- live floor view with 500 ms ghost predictions
- convergence: holdout error, estimate trace, model ladder

Differences from the prototype:

- No simulated driver, speed control, or drag and keyboard input. The gimbals show the telemetry
  `a/b_percent`.
- The floor view draws the field from the profile and the smoothed tag pose.
- A link panel shows telemetry rate, tag rate, clock-fit residual and gate counts.

It is a single static HTML/JS file in `auto_battlebot/calibration/sysid/static/`, fed by a
websocket that pushes a JSON snapshot at 4 Hz. It lives outside `web/` on purpose:

- The Svelte dashboard is a client of the relay, and the relay serves one app.
- This page needs the Python process's state, not the app's.
- No Vite build is needed for a tool used a few times a season.

### 8. Session output

On exit the tool writes these to a session directory outside `runs/` (default
`~/sysid_sessions/<stamp>/`):

- the live estimate as `[plant]` TOML via `PlantParams.to_toml`
- the final coverage snapshot as JSON
- the event log

The MCAP the app recorded stays the source of truth for the offline fit.

## Config

There is no new C++ config. The tool runs against the existing profile:

```bash
# on the ZED Box
./scripts/build_and_run.sh -c config/mr_stabs_mk2_sysid_zed_box.toml
# on the laptop
venv/bin/python playground/calibration/sysid_coverage.py --live ws://<zed-box>:8765
venv/bin/python playground/calibration/sysid_coverage.py --replay <recording.mcap> --speed 8
```

Field size and tag mounts come from the profile TOML (1.52 x 1.52 m in the sysid profile): the
recording's active profile on replay, `--profile` otherwise. Until a real session exists,
`--synthetic out.mcap` writes a simulated one to replay.

## Steps

Each step is one commit. The tree builds and passes `./scripts/lint` and tests after each.

0. **Prerequisites.**
   - ESP-NOW is flashed and tested on the robot.
   - `telemetry_seq u32` lands in `TelemetryPayload`, with the schema updated in
     `json_schemas.hpp` and `mcap_write.py` together (they must stay byte-identical). Another
     agent is adding it.
   - Confirm `/robot/espnow_telemetry` arrives at 100 Hz on ws:8765 under the sysid profile, with
     `espnow_active` false and no `MSG_COMMAND` traffic from the dongle.
   - Record one hand-driven sysid session of 10 minutes or more with the full maneuver menu. It
     is the replay fixture for every step below.
1. **Telemetry loader.**
   - Add `sysid_io.load_espnow_telemetry` with the schema's field list.
   - Add the fit-input mapping from `left/right_cmd` and the prediction-input mapping from
     `a/b_percent`, cross-checked against the OpenTX readback.
   - Add the batched mixer and yaw loop mirror in `auto_battlebot/control/firmware_mixer.py`,
     and a closed-loop rollout that wraps `plant.simulate`.
   - Add a live-capable wrapper around `fit_robot_clock`.
   - Tests: a parity test with the JSON schema in `mcap_write.py`; sign and scale on the
     fixture; the batched mirror against `mujoco_sim/firmware.py` on random stick traces; the
     mirror against recorded `pid_output` and `yaw_rate_cmd` on the fixture.
2. **Maneuver segmenter.**
   - Move it into `auto_battlebot/calibration/sysid/maneuvers.py` and point
     `check_sysid_recordings.py` at it.
   - Tests: synthetic stick traces from the excitation catalog give the expected holds, edges,
     slews and creeps, and the menu numbers on the fixture are unchanged within thresholds.
3. **Streaming tag pose.**
   - Add the fixed-lag smoother in `tag_pose_smoother.py`.
   - Tests: on the fixture, position and heading agree with the offline RTS output within 5 mm
     and 1 deg RMS at 0.5 s lag.
4. **Estimator core** (`auto_battlebot/calibration/sysid/coverage.py`).
   - It contains the sample assembler, gates, window builder, Fisher accumulator, coverage map,
     edge table, candidate ranking and reachability.
   - Pure and deterministic. Driven by a source iterator, with no IO.
   - Tests: on synthetic data from `plant.simulate` with known truth, the empirical spread of
     the fitted parameters over 20 noise seeds matches the reported σ within a factor of 1.5.
5. **Refit worker and MCAP replay.**
   - Add the worker process, the `--replay` source and the session output.
   - Acceptance: on the fixture, the final live estimate matches an offline `jig_fit.joint_fit`
     on the same windows within 2σ per parameter.
6. **Live source.**
   - Add a minimal Foxglove ws-protocol subscriber in `auto_battlebot/recording/foxglove_ws.py`.
     It is subscribe-only: advertise, subscribe, then decode binary message frames. It
     deliberately has no publish path.
   - Tests: replaying the fixture through a local `viz_relay` gives the same snapshot sequence
     as `--replay`.
7. **Page.**
   - Port the prototype page onto the snapshot feed, and add the link panel and banners.
8. **Arena session.**
   - Drive one session with the tool live.
   - Check the acceptance table below and write up the session under
     `docs/experiments/control_improvement/`.

## Acceptance

| # | Criterion | Threshold | Measured by |
| --- | --- | --- | --- |
| A1 | No command traffic while the tool runs | 0 `MSG_COMMAND` frames from the dongle; `espnow_active` false for the whole session | dongle link stats, telemetry |
| A2 | Ticker latency | motion labeled within 0.5 s of finishing | replay timestamps |
| A3 | Refit cadence | at most 5 s between refits at 15 min of driving, on the laptop | worker log |
| A4 | Honest uncertainty | reported σ within 1.5x of seed-to-seed spread | step 4 test |
| A5 | Live vs offline fit | every parameter within 2σ of the offline joint fit | step 5 test |
| A6 | Stopping criterion | when the ring reads 95% or more, the offline fit on that session meets kalman plan C1 to C3 on a held-out session | step 8 |
| A7 | Gates | no window with a wall contact flagged in the offline `sysid_windows` gates is used live | step 8 |

## Risks and open questions

- **The yaw loop mirror can drift from the firmware.** Any change to `yaw_control::Config` or
  `mix_motor_outputs` has to land in both mirrors. The parity test on recorded `pid_output`
  catches it on the next recording, not at build time.
- **Loop gains are tuned on an unfit plant.** The `yaw_control` defaults came from the MuJoCo sim.
  Once this tool produces a fit, the loop may want retuning, which changes the closed-loop
  prediction but not the fit.
- **The 1.52 m box limits the gains.** High-throttle steady state is unreachable in the current
  sysid box, so `k_fwd`, `k_rev` and `tau_lin_a` will read floor-limited. I am trying to get a
  larger box; if that falls through, finishing them needs match data (`match_windows`).
- **Operator feedback may bias drift.** I steer against drift without thinking, so the stick
  input is correlated with the disturbance. The fit runs on the measured motor commands, so
  this only biases drift if the drift itself is a disturbance the model lacks. The segmenter
  labels hands-off straight runs (steer under 0.03 for 1 s or longer) in the ticker; the fit
  does not yet restrict the drift terms to them.
- **Live wall gating is weaker than the simulator's.** The 0.15 m rail gate and the residual gate
  will miss glancing contacts and flag some hard maneuvers. A7 checks this against the offline
  gates.
- **Arena link loss.** ESP-NOW works on the robot, but arena 2.4 GHz traffic is untested. Dropped
  frames show as `telemetry_seq` jumps and gate the window instead of corrupting it.
- **Telemetry fields change in espnow Phase 2.** The loader reads by field name with a schema
  parity test, so a renamed field fails the test instead of silently reading NaN.
- **The ticker runs 0.5 s behind.** The session processes data 0.5 s old so the tags around it
  have arrived, and an event shows once its window has closed and been gated, which A2 may
  miss. If it reads as sluggish at the arena, label from sticks at once and mark the window
  later.

## Next steps

Remaining, in order:

1. Step 0: record a hand-driven session under the sysid profile, then replay it:
   `venv/bin/python playground/calibration/sysid_coverage.py --replay <mcap> --speed 8`.
2. On that fixture, run the checks the simulation cannot: the mirror against recorded
   `pid_output`, `a/b_percent` sign and scale against the OpenTX readback, and A5 against an
   offline joint fit.
3. Run the live source against the box's relay once (`--live ws://<zed-box>:8765`).
4. Step 8 at the arena, in the largest box available.
