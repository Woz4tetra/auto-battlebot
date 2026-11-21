# Navigation scoring and RL v2 plan

Goal: Mr Stabs Mk2, driven by an RL policy, impresses our human driver in the 1.5 m test box. It
gets lots of back wedges and pins on Mrs Buff Mk3 and stays out of her vertical spinner's front
arc. Every change is decided by a navigation scoring pipeline built the same way as the detector
eval in `auto_battlebot/eval/`: one scorer, hand-validated ground truth, and a paired bootstrap.
The lessons then carry to Mrs Buff Mk3 and Mk4.

This plan follows `docs/plans/rl_policy_navigation_plan.md` (deleted 2026-11-18, in git history). That plan built the RL stack. This one
fixes what broke when the stack met the field, and adds the measurement that was missing.

The decisions come from a grilling session on 2026-11-14. The full question and answer log is in
[Decision log](#decision-log) at the end. A review the same day changed the sequencing, the
statistics and parts of phase 5; every change is listed in
[Review amendments](#review-amendments), with the grill answer it touches.

The work is split into packages that separate agents can pick up. See
[Work packages](#work-packages).

## Status (2026-11-14)

Not started. The order of work is in [Order of work](#order-of-work): bench checks, then cheap
field fixes.

## What went wrong on 2026-10-25

Three Mr Stabs Mk2 recordings from that evening ran `RlPolicyNavigation` with bundle
`rl_nav_p4` on profile `mr_stabs_mk2_fid_rl_zed_box`. All three are in `data/recordings/`; the
filenames are in UTC.

| File | Length | Autonomous |
| --- | --- | --- |
| `auto_battlebot_mr_stabs_mk2_fid_rl_zed_box_2026-10-25_22-09-22.mcap` | 252 s | 60 s |
| `auto_battlebot_mr_stabs_mk2_fid_rl_zed_box_2026-10-26_00-28-40.mcap` | 112 s | 94 s |
| `auto_battlebot_mr_stabs_mk2_fid_rl_zed_box_2026-10-26_00-49-06.mcap` | 245 s | 213 s |

**Tracking was good.** Our robot was live in 89-93% of frames and the opponent in 83-94%. Gaps
were p50 33 ms and p90 34 ms, so almost all were single dropped frames. Pipeline latency was
43 ms median and 48.9 ms max. The ESP-NOW link had at least 99% ack and an 8-12 ms round trip.

**The command path differed from the sim.** The 27 windows where the policy commanded strong
reverse (below -0.5 for over 0.5 s, 18.8 s in total) delivered a median wheel duty of only -12%.
- **Slew limit.** `mr_stabs_mk2_fid_rl_zed_box.toml` extends `mr_stabs_mk2_fid_zed_box.toml`,
  which sets `max_motor_rpm_per_sec = 2000` (1.33/s normalized). The policy asked for -0.60 on
  average and -0.33 was sent. The sim has no slew limit.
- **Motor guard.** The firmware thermal guard runs on placeholder limits. It reached 60 °C
  against `rise_max_c` 50 and was in LIMITING state 37-47% of the time. Inside the reverse
  windows its scale had a median of 0.48.
- **Edge gap.** There were 13 wall-stuck episodes, 28 s in total. The robot sat 50-80° off the
  wall normal and the policy sent +0.19 linear with a small angular command. The firmware yaw loop
  asked for 230-400 °/s and got about 0, with one wheel near 30% and the other near 0. The field
  is bordered by 2x4s, and the wedge drops into the gap between the board's rounded bottom edge
  and the floor. The robot tries to turn in place instead of backing off first. The sim's walls
  are flat boxes on a flat floor, so the policy never met the gap.
- **Yaw-rate observation.** The filter's `our_yaw_rate` comes from the geometry-only plant
  (k_ang 60 rad/s). It correlates 0.26-0.30 with the heading derivative of the tracked pose,
  while the IMU correlates 0.7-0.8. Its p99 is about 2900 °/s against roughly 600 °/s true, and
  the normalized value is above 1 in 15-23% of steps.

**The opponent model couldn't teach pins.** The training opponent is a mocap body that stops at
contact, so Mk2 can never push, lift or pin it. In 00-49, Mk2 spent 14.0% of autonomous time
(29.8 s) inside Mk3's ±45° front arc and only 6.3% with its wedge on her side or back. It came
within 0.35 m 132 times, 37 per minute.

**Setup problems.** In the 22-09 session our robot sat outside the 1.5 m field box in 8.4% of
steps, so the wall rays read 0. A phantom `neutral_robot_1` with 122 m sigma sat in the tracks of
the later two sessions; it didn't reach the observation.

**Nothing could score any of this.** The sim scorer (`auto_battlebot/rl/metrics.py`) and the field
scorer (`auto_battlebot/rl/score_field_session.py`) are separate implementations. The field scorer's
docstring calls its thresholds "unvalidated against video". The two `review.csv` files it wrote
for 00-28 (12 hits) and 00-49 (36 hits) have an empty `confirmed` column. Warp eval is not
reproducible: the same 128 scenarios gave 3.0-5.2 hits per hit taken across runs
(`docs/experiments/control/rl_navigation_report.md:188-191`). No navigation decision
so far used a significance test.

## Requirements

1. **Mr Stabs Mk2 (wedge) in the test box.** Mk2 will not fight in competition again. Success is
   impressing our human driver, who drives Mrs Buff Mk3 as the opponent. That means many back
   wedges and pins, and almost no time in the spinner's front arc. NHRL judging does not apply.
2. **Mrs Buff Mk3 and Mk4 (vertical spinners), later.** Avoid the opponent's weapon and land hits
   at good moments. NHRL judges score Attack (5 points, active-weapon attacks only) and
   Condition (6 points), penalize obvious evasion, and cap pins at 10 s. Knockouts win outright.
3. **The policy handles varying conditions.** Unknown field conditions and a measured, changing
   battery. Wheel damage is deferred until Mk2 is proven in the test box.
4. **RL is the method.** PPO is the main line. No separate weapon-arc safety filter and no
   explicit online plant estimator: avoidance and adaptation live inside the policy.
5. **Every change is scored.** Sim results gate field sessions; field results decide what counts
   as better. The user validates the field ground truth by hand, the way every image of the
   detector GT was reviewed.

Radios: Mk2 uses ESP-NOW for autonomy (the SA switch hands control to ESP-NOW while commands are
fresh, `ESPNOW_COMMAND_SUBSTITUTION = true` in `firmware/mr_stabs_mk2/src/main.cpp`), with
Crossfire as the failsafe. Mk3 and Mk4 use ELRS only.

## Order of work

0. [Phase 0: bench checks](#phase-0-bench-checks). Reproduce 10/25 in lockstep, run the step
   test, add the thermal trip. No driver time.
1. [Phase 1: cheap field fixes](#phase-1-cheap-field-fixes), then one baseline field session with
   `rl_nav_p4` unchanged plus two calibration controllers.
2. [Phase 2: measurement](#phase-2-measurement). k, tolerances and gated buckets are frozen here,
   before any candidate exists.
3. [Phase 3: sim gate](#phase-3-sim-gate).
4. [Phase 4: fidelity and residual sim](#phase-4-fidelity-and-residual-sim).
5. [Phase 5: sim changes and retrain ladder](#phase-5-sim-changes-and-retrain-ladder). One rung
   at a time, each gated on lockstep.

Phases 2 and 3 can start on day one. The phase 5 sim work (5.1-5.6) can be built in parallel
too; only the training runs wait for the gate.

## Phase 0: bench checks

### 0.1 Reproduce 10/25 in lockstep

The phase 1 session changes four things at once (slew, guard, yaw rate, monitor), so it cannot
say which one caused the reverse freezes. Lockstep can.

- Run `rl_nav_p4` in lockstep in four arms on the same seeds: slew 2000 with a guard scale of
  0.48, slew 2000 alone, guard 0.48 alone, neither. If the lockstep command path skips either
  stage, add it to `auto_battlebot/rl/lockstep_world.py` behind a scenario parameter.
- Measure the 10/25 statistic: median delivered duty in windows where the policy commands below
  -0.5 for over 0.5 s (field: -12%).
- Pass: the arm with both stages lands near the field number and the arm with neither does not.
  That is attribution, and the first fidelity result. If the freezes do not reproduce, the
  command path is not the whole story and phase 1 needs another look before the session.
- Write it up in `docs/experiments/control/nav_eval/`.

### 0.2 Forward step test

`rl_navigation_report.md` ("Flip risk without the slew limit") says a held forward step of 0.8
flips the fitted robot, the 0.6 threshold depends on an armature sysid could not identify
(1.10 against 0.51), and the real robot lifts its nose more than the fit. Removing the slew
limit (1.1) is blocked on this test.

- The user drives 0.5, 0.6 and 0.7 forward steps from rest on a soft surface, slew limit off,
  recorded to MCAP. Human-driven only.
- Refit the armature from the pitch response. Phase 4's response fidelity needs the same number.
- Decision: if 0.6 lifts the nose past about 30°, keep a slew limit, pick the loosest value that
  holds the nose down, and model that value in the sim instead of removing it.

### 0.3 Thermal trip

Loosening the guard (1.2) on placeholder limits leaves the motors unprotected for a 3 minute
session.

- Add a hard trip to the firmware: above a set estimated rise it cuts drive and latches until
  disarm. A trip ends the bout and the scorer drops it. A latch is binary, so it does not scale
  commands and does not break command-chain parity.
- The trip threshold needs one real number. After a 3 minute driven session the user measures
  the motor can temperature and compares it with the logged `motor_rise_left/right`.

## Phase 1: cheap field fixes

No retraining. `rl_nav_p4` stays as it is.

### 1.1 Slew limit off for RL

- Set `max_motor_rpm_per_sec = 0` in every profile that loads `navigation/mr_stabs_mk2_rl.toml`
  or `navigation/mrs_buff_mk3_rl.toml`. `_common.toml` already defaults it to 0.
- Blocked on the step test (0.2). The policy trained with no slew limit and a -0.05
  `command_change` penalty, so removing the limit matches training, but only if the real robot
  survives the steps the policy sends. If 0.2 says keep a limit, set that value here and in the
  sim.

### 1.2 Motor guard loosened until it never binds

- Raise the firmware guard limits used for test-box sessions so `guard_scale` stays at 1 through a
  3 minute session. The guard then counts as "off" for command-chain parity (1.4), and the sim
  needs no guard model.
- The thermal trip (0.3) is the protection while the scaling guard is loose.
- Log motor temperature rise (`motor_rise_left/right`) in every session.
- Pass: `guard_scale` = 1 in at least 99% of autonomous time, checked from the logs by
  `PerformanceMonitor` and the field scorer.

### 1.3 Filter yaw rate

Two fixes, landed separately:
- **IMU fusion (first).** Fuse the telemetry `yaw_rate` (deg/s clockwise, about 100 Hz over
  ESP-NOW) into the our-robot filter as a measurement, with its transport delay handled. This
  fixes the number `rl_nav_p4` observes (normalized by /15) and moves its input closer to
  training, so it goes into the baseline session.
  - Check: the filter yaw rate correlates with the tracked heading derivative at least as well
    as the IMU (0.7-0.8 on the 10/25 sessions), and its p99 is within the IMU's range.
- **Fitted plant (second, its own A/B).** Replace the geometry-only plant (k_ang 60 rad/s) with
  the fitted grey-box plant. This also changes the filter's position and heading error through
  dropouts, and the training EKF error table was built on the old filter. Run a paired lockstep
  A/B of `rl_nav_p4` with and without it. It ships into the baseline session only if the A/B is
  not worse; otherwise it waits for phase 5, where the error table gets refit
  (`fit_provisional_ekf_error.py --lockstep`).
- Any change to the C++ filter needs the Python mirror and golden fixture regenerated.

### 1.4 PerformanceMonitor

A C++ component that publishes warnings to the dashboard banner and to `/diagnostics` in the
MCAP. It replaces a pre-session checklist: there's no time for one at competitions.

**At load:**
- Hash the resolved command chain: `max_motor_rpm_per_sec`, guard mode, `drive_mode` (5.4),
  mixer mode, transmitter type, and firmware protocol version. Compare it with the hash the policy
  bundle stored at export. **A mismatch blocks the RL policy from loading** and lists the keys
  that differ.
- Log the firmware build hash and protocol version from telemetry.

**Continuously, over rolling 5 s windows, warn-only:**

| Check | Warns when |
| --- | --- |
| Slew binding | policy command and sent command differ by more than 0.2 for over 10% of ticks |
| Guard binding | `guard_scale` below 0.9 for over 5% of ticks |
| Calibration | our robot's pose outside the field box for over 2% of ticks |
| Phantom track | a track with sigma above 1 m that isn't moving while reported |
| Yaw-rate disagreement | IMU yaw rate vs tracked heading derivative correlation below a set threshold (the filter fuses the IMU after 1.3, so filter vs IMU would agree by construction) |
| Thermal trip | the firmware trip (0.3) latched; red, ends the bout |
| Link | ESP-NOW ack rate, command age, or telemetry drops past set limits |
| Tracking | our-robot or opponent coverage below 85% |

- Each warning records a severity, its start and end time, and the values that triggered it. The
  field scorer (2.5) excludes bouts that had red warnings by default.
- Thresholds are defaulted fields in the component's config struct, per the repo convention.
- The 22-09 calibration problem and the phantom `neutral_robot_1` are the first test cases: replay
  those MCAPs' tracks through the checks and confirm both warnings fire.

### 1.5 Baseline field session

Run a session with `rl_nav_p4` and the fixes above. Score it with the existing
`score_field_session.py` until phase 2's scorer exists, then rescore it. The result becomes the
`rl_nav_p4` field baseline that phase 5 must beat.

The same session also runs two **calibration controllers**, interleaved with `rl_nav_p4`:
`rl_nav_d_v2_8k` and one classical navigation (MotionProfile), each on the command chain it was
built for. They are not
candidates. Phase 4 needs at least three controllers with both a sim score and a field score to
check that the sim ranks them the way the field does; two points cannot show that.

Collect as many bouts as the driver has patience for. The baseline is reused in every later
comparison, so its bout count sets the width of every later interval.

## Phase 2: measurement

### 2.1 Shared metric library

- Lives in `auto_battlebot/eval/nav/`, so both `auto_battlebot/rl/` and `scripts/` import it.
- **One code path for sim and field.** It takes time series of tracked poses and our own commands
  from either lockstep output or an MCAP. `metrics.py` and `score_field_session.py` become
  callers instead of keeping their own copies.
- **Events use only signals both sides have.** The field has no contact sensor and no view of
  Mk3's sticks (she is on ELRS). So every event is defined from tracked poses, our commands and
  our telemetry. Sim-only signals (MuJoCo contacts, the opponent's commanded motion) never enter
  an event definition; they are ground truth for validating the detectors (2.4) and inputs to
  training rewards (5.1).
- **Robot profile as data** (`auto_battlebot/eval/nav/profiles/`): our weapon geometry (Mk2 wedge
  box; Mk3 disk sweep later), the opponent's weapon arc, and which event types apply. Only the Mk2
  profile is implemented and validated now. Mk3 adds a profile instead of a rewrite.
- **Events** are intervals with a type, a start, an end and properties. Types:
  - `hit_taken`: our hull inside the opponent's weapon arc while the robots close
  - `back_wedge`: our wedge box within the contact distance of the opponent's side or back
  - `pin`: a `back_wedge` where the opponent's tracked speed stays under a threshold, or the
    opponent moves with us while we push, for over 1 s. Capped at 10 s per pin, as NHRL does, so
    one long hold cannot carry a bout
  - `stuck`: low speed near a wall for over 1 s, with subtype `wedge_in_edge_gap`
  - `flip`, `hazard_entry`, `disengaged` (more than 15 s without engaging)
- The weapon arc and the contact distance are calibrated against hand labels (2.3) and sim
  contact truth (2.4).

### 2.2 Metrics

- **Headlines, both must pass:**
  - **Net exchange per minute** = (back-wedge seconds + pin seconds per minute) − k × (hits
    taken per minute). k starts so that one hit taken cancels about 3 s of control.
  - **Weapon-arc time share.** Q1 asked for the weapon avoided 99% of the time, and `rl_nav_p4`
    sat at 14.0%. A composite can hide arc time behind wedge seconds, so arc time gets its own
    gate and its own absolute target.
- **Guardrails** (must not regress beyond a tolerance): stuck seconds per minute, flips, hazard
  entries, disengaged seconds.
- **Diagnostics** (reported, never gated): time to first contact, approach-angle histogram,
  command saturation, and the slew and guard binding fractions.
- **Frozen before any candidate runs:** k, each guardrail tolerance, the absolute floors, and the
  list of gated buckets (3.4). k is fit once, to the driver's ratings from the baseline session
  (1.5), and then written into the Mk2 profile. Changing any of them later means a new metric
  version and rescoring every stored run. Tuning them after seeing a candidate's bouts picks the
  metric to fit the answer.

### 2.3 Event labeler

A web app in `auto_battlebot/nav/labeler/`. It follows the in-repo editor convention
(no Label Studio).

- **Views:** overhead video from the MCAP camera frames; a top-down track view with the opponent's
  weapon arc and our wedge drawn on; an event timeline underneath. All three stay synced.
- **Proposals:** the automatic event detectors from 2.1 pre-fill events. The user confirms or
  rejects them, adjusts edges, and adds missed events. Keyboard-driven, with 0.5x-2x playback.
- **Full coverage:** the user reviews every ~10 s segment of autonomous time and marks it `pass`.
  Only `pass` segments count as ground truth. The labels are stored next to the recording with a
  `validation_state.json`-style gate, the same pattern `score.py` uses for detector GT.
- **First session:** 10/25 00-49, 213 s of autonomy, about 22 segments, roughly an hour of
  review. Then 00-28.
- The existing `review.csv` clips from `score_field_session.py --clips` are imported as starting
  proposals.

### 2.4 Validating the scorer

For each event type, compute the automatic detector's precision and recall against the hand
labels, the way the detector GT validated the engines. Tune the weapon arc, contact distance and
merge window until precision and recall are acceptable on 00-49. Hold out 00-28 to confirm.
Report the numbers in `docs/experiments/control/nav_eval/`.

Two more sources, because 10/25 alone is too thin:
- **Sim contact truth.** In lockstep, MuJoCo knows every wedge contact. Run the pose-only
  `back_wedge` and `pin` detectors on lockstep tracks (with the sensing model's noise on) and
  score them against the contact truth. This gives thousands of labelled events for free and
  shows how much the contact-distance proxy smears at tracking noise levels.
- **Pin positives.** 00-49 has 6.3% wedge time and close to no pins, so it cannot validate the
  pin detector. Record one short session where the user drives Mk2 and deliberately pins Mk3,
  and label it. This is detector data only. Human-driven Mk2 is still not a performance bar
  (Q36).

Set the pass bar before tuning: precision and recall of at least 0.8 per gated event type on the
held-out session. An event type that cannot reach it is reported but not gated.

### 2.5 Field session app and bout scoring

A separate web app, `auto_battlebot/nav/field_session/`, to set up and score field
bouts.

**Setup:**
- picks the bout order for blind A/B: the driver isn't told which controller is running
- shows the start position for each bout from a fixed list (facing, side-on, Mk2 in corner),
  rotated per bout
- records the opponent brief (one stated style per session, e.g. "hunt Mk2 aggressively")
- records the battery voltage at the start of each bout
- takes the driver's 1-5 rating after each bout, plus one forced choice from the second bout on:
  "better or worse than the last bout?" Pairwise preference separates two controllers in fewer
  bouts than a 1-5 scale does
- schedules bouts in adjacent candidate/baseline pairs on the same start position, order
  randomized inside the pair, so battery, driver fatigue and floor state cancel within a pair

**Scoring:**
- splits each MCAP into bouts automatically at arm/disarm and autonomy toggles, so any recording
  can be scored, including ad hoc runs
- a bout is 60 s by protocol, scored from arm to disarm
- tags each bout with the controller, bundle hash, `drive_mode`, battery and any active
  `PerformanceMonitor` warnings
- appends the bouts to the results store (3.2) as `split = field`
- a field result exists as soon as the MCAP is copied off the box

**Statistics:** blind interleaved bouts compare candidate against baseline with a paired bootstrap
over bout pairs, matched by start position. Ad hoc bouts use an unpaired bootstrap over bouts.

Sessions stay ad hoc: whenever convenient, with no minimum per session. Pairs for the same
candidate and baseline pool across sessions in the results store. The app prints
**"no verdict"** until a comparison has 10 pairs, because a percentile bootstrap on fewer pairs
reports intervals narrower than the truth and would pass candidates on noise. Under 10 pairs it
still shows the point estimate, the interval, and a sign test on the pairwise preferences, all
marked provisional.

## Phase 3: sim gate

### 3.1 Scenario manifest

- Buckets are **queries over tags, not directories**. Each suite version is one table, checked
  into git as JSONL or Parquet, since scenarios are just parameters and seeds.
- One row per scenario: `scenario_id`, `seed`, and tag columns: `opponent`, `opponent_speed`,
  `start` (open, near_wall, corner, facing_weapon, wedge_in_edge_gap), `arena_m`, `battery_v`,
  `friction`, `dropout`, `drive_mode`, `split` (train, eval, holdout).
- The suite version is the content hash of the table. Changing the suite means a new version,
  never an edit. The first version is `nav_suite_v1`.
- Large assets (human replay tracks, recorded MCAPs) stay in `data/` and are referenced by content
  hash.
- The **held-out split** uses human-replay tracks from sessions never used in training.

### 3.2 Results store

- One Parquet dataset. Every eval run appends rows: `suite_hash`, `scenario_id`, `controller`,
  `bundle_hash`, `git_hash`, `sim_hash`, `metric_version`, `drive_mode`, and the metrics from
  2.2.
- `sim_hash` covers everything that changes what a scenario means without changing its row: the
  MJCF, the plant fit, the sensing model, the residual model (phase 4) and the opponent
  controllers' code. Comparisons refuse to pair rows with different `sim_hash` or
  `metric_version`. Without it a residual refit silently makes old baseline rows incomparable.
- Field bouts (2.5) go in the same store with `split = field`.
- Queried with DuckDB, e.g. "per-bucket net exchange for every policy since 11-01". Reports and
  the results table are queries over it.
- No server and no account. MLflow, W&B and DVC were considered and rejected for that reason.

### 3.3 Lockstep runner

- `auto_battlebot/rl/lockstep_eval.py` is the gate. It runs the real `build/auto_battlebot` against
  CPU MuJoCo (`auto_battlebot/rl/lockstep_world.py`), seeded per scenario. It already works with
  any C++ navigation type through `--nav`.
- Change it to read scenarios from the manifest and write rows to the results store.
- Measure the noise floor with an A/A run: the same bundle against itself on the full suite,
  twice. Pass: the paired CI on every headline contains 0, and its half-width is small against
  the effect the gate has to detect. Record the half-width; it is the gate's resolution.
  Bit-identical runs are welcome but not required. The binary is threaded, and chasing the last
  source of nondeterminism can cost days that an honest A/A interval makes unnecessary. If the
  A/A interval excludes 0, find the source before trusting the gate.
- Warp is for training only.
- Runtime measured so far: 128 twenty-second episodes take 408 s with MotionProfile and 1692 s
  with Pursuit, on 6 workers.

### 3.4 Statistics

- Run candidate and baseline on the same seeds and scenarios, take per-scenario differences, and
  bootstrap the mean difference (95% CI), per bucket and pooled.
- Reuse `auto_battlebot/eval/significance.py`: a shared resample index matrix, ratio of sums per
  resample, a percentile CI on the delta, and a better/worse/ns verdict by polarity. It resamples
  per scenario in sim and per bout in the field instead of per frame.
- **Sim acceptance:** both pooled headline CIs are on the better side of 0, no gated bucket's
  headline CI is below −tolerance, and no guardrail regresses beyond its tolerance.
- **Gated buckets are a fixed list of at most six**, written into the suite before any candidate
  runs: open floor, near wall, corner, `wedge_in_edge_gap`, held-out human replay, low battery.
  Any other slice is a diagnostic. With every tag combination gated, some bucket fails by chance
  on every run and the gate stops meaning anything.

### 3.5 Regression tiers and the results record

- **Smoke tier:** a ~5 min lockstep subset (64-128 short episodes) on every navigation,
  firmware-mixer or config commit. A script runs it, manually or from a hook.
- **Full suite:** on demand for any field candidate, through the GPU queue if it's heavy.
- **Results record:** `docs/experiments/control/nav_eval/results.md` is generated from the store. It has
  one row per run (git hash, controller, suite version, headline ± CI, guardrails), plus a short
  report with plots for each candidate. Never under `runs/`.

## Phase 4: fidelity and residual sim

The sim gate only counts once the sim is shown to predict the field. Re-run all three checks
whenever firmware, the plant fit or the config chain changes.

1. **Static parity.** The deployed command chain matches the sim's: slew, guard, mixer,
   `drive_mode`. This is the same hash `PerformanceMonitor` checks. A lockstep test fails if they
   differ.
2. **Response fidelity.** Compare step and chirp responses from a human-driven sysid MCAP with the
   sim plant driven by the same commands. Gate on per-wheel speed error (from `motor_speed_left/right`)
   and command-to-motion latency. Score closed-loop wherever the firmware heading hold is active.
3. **Behaviour fidelity.** For the same controller, compare the sim and field distributions of
   speed, yaw rate, time in the weapon arc, wall contact and stuck time. A large gap means the
   sim gate is not predicting the field.
4. **Rank agreement.** The gate's job is to rank controllers, so test that directly. Score every
   controller that has field bouts (`rl_nav_p4`, the two calibration controllers from 1.5, then
   each candidate) on both headlines in sim and in the field. Pass: the sim order matches the
   field order wherever the field CIs separate. A sim that matches distributions but ranks
   controllers wrongly is useless as a gate, and the reverse is tolerable.

**Residual sim (Swift-style).** After the fidelity checks show where the gap is, fit a small
"real minus sim" residual model from field MCAPs and add it to the training sim. Refit after
each field session that adds data; every refit changes `sim_hash` (3.2), so the baseline is
rerun. This follows Kaufmann et al., *Nature* 2023 (Swift, champion-level drone racing), which
fit residual dynamics and perception models from flight logs.

- In scope: wheel response (command to per-wheel speed, from eRPM at about 100 Hz) and
  perception error (tracked pose against the filter's own smoothed estimate, plus the dropout
  process).
- Not in scope: a contact residual. Swift had motion capture at hundreds of Hz. Here contact is
  seen through 30 Hz poses with no force measurement, and a model fit to that learns tracking
  noise. Contact stays domain-randomized: wall and floor friction, restitution, the edge-gap
  geometry (5.5).

## Phase 5: sim changes and retrain ladder

### 5.0 The ladder

The sim changes below are built in parallel, but they are **trained one rung at a time**, each
rung gated on lockstep before the next is added. A rung is hours at about 91k steps/s. A single
retrain with nine changes that fails the gate leaves nine suspects, and this PPO stack has
already overwritten learned behaviours, stalled at curriculum 0.4 and collapsed its std to
0.029 (`rl_navigation_report.md`).

| Rung | Adds | Passes when |
| --- | --- | --- |
| R1 | Obs spec v2 (5.3) and `drive_mode` (5.4) on the **old** world: mocap opponent, flat walls | Matches `rl_nav_p4` on the suite: no headline CI on the worse side of 0. This isolates the spec, the C++ mirror and the mode plumbing from every behaviour change |
| R2 | Physics opponent, pin reward, hit impulse (5.1) | Back-wedge and pin seconds beat R1 with the CI above 0; arc time not worse |
| R3 | Edge gap, arena weighting, dropout, battery sag (5.5, 5.6) | `wedge_in_edge_gap` bucket stuck time beats R2; no gated bucket regresses |
| R4 | Mk3 hunter in the league (5.2) | Arc time beats R3 on the held-out human-replay bucket |

- The `drive_mode` bake-off (5.4) runs at R1, where it is cheapest. R2-R4 train only the winner.
- Each rung fine-tunes from the previous one with the anchor (`anchor_coef` in
  `auto_battlebot/rl/learners/ppo.py`) on the behaviours it must keep, and resets log-std first.
  An earlier run showed 0.08 held the old behaviours where 0.03 drifted and 0.5 erased the new
  one.
- R3 is the first rung worth a field session: it has the physics opponent and the wall fix,
  which are the two things the driver complained about. R4 is trained only if R3's arc time
  misses its target in the field.
- Rungs add scenarios, so each rung's suite is a new version (`nav_suite_v1`, `_v2`, ...) that
  contains the previous one. The baseline is rerun on each.

### 5.1 Physics opponent

- Replace the mocap opponent with a dynamic body: 3 lb mass, friction, and a drive force limit.
  Steer it with the existing scripted, replay and policy controllers through velocity actuators.
- Mk2 can then push, lift and pin it.
- Pin reward: the opponent's commanded motion is cancelled while our wedge has contact. The
  reward may use sim-only signals; the scored `pin` event (2.1) may not.
- **Drive force limit from a measurement.** Measure Mk3's stall push against Mk2's (a luggage
  scale against a wall is enough) and randomize around the measured ratio. Set too weak, pins
  are free and the policy farms them; set too strong, it never learns one.
- **Hit impulse.** When Mk2's hull is inside the weapon arc at contact, apply a randomized
  impulse and spin to Mk2 (magnitudes bracketed from what the 10/25 tracks show after a hit).
  Recovery after a hit is much of what the driver sees, and a mocap or passive opponent never
  produces it.
- A replayed human track cannot react to being pushed. On contact, the replay controller hands
  over to a scripted escape-and-resume behaviour, then rejoins the track.

### 5.2 Opponent league

- (b) Add the driver's 10/25 Mk3 tracks to `simulation/opponent_tracks/replay_tracks.npz`.
- (c) Keep scripted pursuers at about 30% of episodes so earlier behaviours don't regress.
- (a) Train a Mk3 policy to land weapon hits on the current Mk2. The policy-opponent plumbing
  exists for Mk3 (`auto_battlebot/rl/envs/policy_opponents.py`). This is rung R4. It is a second
  RL problem, and nothing has beaten the 8-10 rad/s scripted pursuers yet, so (b) and (c) on the
  physics opponent go first. Then: train (a) against the R3 Mk2, and train Mk2 against the
  frozen (a) plus (b) and (c).
- **The held-out split needs more data.** Three sessions from one evening, with one already
  spent on scorer tuning, is not a holdout. Record one dedicated session of the user driving Mk3
  against a human-driven or parked Mk2, never used in training, as the holdout's source.

### 5.3 Observation spec v2

One spec change, so it means one retrain:
- the IMU gyro yaw rate replaces the filter yaw rate, in the observation and in its history. In
  sim it comes from MuJoCo's gyro with matched noise and delay
- `pack_volts` moves from critic-only to the actor's observation
- per-wheel speed from eRPM (`motor_speed_left/right`), with the telemetry rate, delay and
  quantization measured on the robot
- the history grows from 8 steps (160 ms) to 0.5-1 s
- bump the spec version in `auto_battlebot/rl/obs_spec.py` and the C++ mirror; the bundle
  self-test covers the parity

### 5.4 drive_mode

- `enum class DriveMode { DIRECT_WHEELS, YAW_LOOP }` in `include/enums/drive_mode.hpp`, included
  from `include/enums.hpp` and parsed with `PARSE_ENUM`.
- In `DIRECT_WHEELS` the policy commands left and right wheel duty and the firmware yaw loop is
  bypassed. In `YAW_LOOP` the policy commands linear velocity and yaw rate through the firmware
  heading loop. Human driving always uses the yaw loop.
- The mode goes to the firmware with each ESP-NOW command, so switching needs no reflash. This
  means a protocol version bump in `firmware/shared/espnow_protocol`.
- Each bundle records the mode it trained with. The command-chain hash includes it, so a mismatch
  blocks loading (1.4).
- The sim models both modes. Train one policy per mode at rung R1, then compare them on the same
  suite as a paired bake-off.
- The bake-off has to be fair to both arms:
  - **`DIRECT_WHEELS` needs the ESC output floor in the sim.** The Mk2 ESC clamps at DShot step
    65: any command from 0.2% to 6.5% spins the wheel at about 280 rad/s. A policy trained on a
    linear low end will find its small corrections do nothing like what it expects.
  - **`DIRECT_WHEELS` loses the inner loop.** The firmware yaw loop rejects disturbances at its
    own rate; the policy closes the loop at its tick rate through about 55 ms of delay.
    Randomize command latency and per-wheel gain in this arm.
  - **`YAW_LOOP` must be a repaired loop.** On 10/25 the loop asked for 230-400 °/s and
    delivered one wheel at 30% and the other near 0. Find out why first (mixer saturation
    against the ESC floor is the likely cause, unconfirmed) and fix or model it. Racing the
    10/25 loop against a new mode decides the bake-off before it runs.
- The bake-off verdict needs field pairs as well as the sim gate, since this is the choice the
  sim is least able to judge.
- **When the bake-off is done, delete the enum and the toggle and keep only the winner.**

### 5.5 Edge gap and walls

- Model the 2x4 border: a notch at the base of every wall, made by the board's rounded bottom
  edge plus a floor gap. Randomize the fillet radius at 3-6 mm and the gap at 2-5 mm until the
  test box is measured.
- Add `wedge_in_edge_gap` starts (Mk2 nose-first in the gap at 30-80° to the wall) to training and
  as a manifest bucket.
- The policy has to learn to back off first and then turn. Turning in place with the wedge in the
  gap is what looked stuck on 10/25. No hand-written unstick routine.
- Check that `still_per_s` and the contact terms make backing out pay off. If the policy still
  pushes, look at the reward before adding new terms.

### 5.6 Other environment changes

- **Arena:** about 70% of Mk2 episodes at 1.4-1.6 m and 30% across the full 1.5-2.5 m range, plus
  more episodes starting near walls and in corners. Measure wall friction.
- **Dropout:** turn tracking dropout on (`simulation/sensing_models/provisional_rl.toml`, now
  `enabled = false`), sampled from the measured gap distribution of the latest recordings. Check
  the newest recordings first, since tracking improved; the 10/25 numbers are in
  [What went wrong](#what-went-wrong-on-2026-10-25). Keep the C++ stop after
  `max_our_unmeasured_s` (0.15 s).
- **Battery:** pack voltage sags with current draw during the episode.
- **Wheel damage:** deferred until Mk2 is proven in the test box.

### 5.7 Training method

- PPO stays the main line (`auto_battlebot/rl/learners/ppo.py`).
- Scripted demonstrations, concretely. PPO is on-policy and has no replay buffer to seed, so
  "seeding" means the existing anchor term: on masked worlds the loss pulls the policy toward a
  scripted controller's action. Write two scripted teachers, "get behind and wedge" and "back
  out of the wall, then turn", anchor on them only in the scenarios each covers (flank-open
  starts, `wedge_in_edge_gap` starts), and decay `anchor_coef` to 0 over the rung so the policy
  can beat the teacher. No human driving. If a rung passes its gate without a teacher, skip it.
- If adaptation stays weak, add an asymmetric teacher-student: a teacher with privileged sim state
  (true battery, friction, opponent pose), and a student that learns to copy it from real
  observations only.
- Deferred: model-based RL (TD-MPC2, DreamerV3). The sim already runs at about 91k steps/s, so
  sample cost is not the bottleneck.

### 5.8 Gate and field session

Every rung has to pass its row in 5.0 and the sim gate (3.4) against `rl_nav_p4` on the current
suite. R3 is the first that goes to a field session; R1 and R2 go to the field only for the
`drive_mode` bake-off pairs.

## Acceptance

**Sim gate (per candidate):** see 3.4 and the rung table in 5.0.

**Final Mk2 demo:** over at least 10 blind interleaved bout pairs, pooled across sessions:
- net exchange per minute beats `rl_nav_p4` with the paired CI above 0
- weapon-arc time share beats `rl_nav_p4` with the paired CI below 0, and is under an absolute
  ceiling. Q1's goal is 1%. The ceiling used for acceptance is fixed after the baseline session
  shows what `rl_nav_p4` does with the command path fixed, and before any candidate is tested
- hits taken per minute and stuck seconds per minute are at or below `rl_nav_p4`
- absolute floors (e.g. stuck under 5% of the bout, hits per minute at most X) are fixed from the
  numbers of the phase 1 baseline session, before any candidate is tested
- the driver's median rating is at least 4, and the driver prefers the candidate in more than
  half of the pairs

Human-driven Mk2 is not used as a reference bar.

## Out of scope

- Wheel damage randomization and per-wheel strength inference, until Mk2 is proven.
- A weapon-arc safety filter (control barrier function) and an explicit online plant estimator.
- MPPI, hierarchical controllers, and classical controllers as bake-off candidates. One classical
  controller does run in the baseline session as a calibration point for rank agreement (1.5).
- A learned contact residual (phase 4).
- Mrs Buff Mk3 and Mk4 navigation work beyond keeping the metric library robot-agnostic.
- Hardware other than the new compute enclosure and power delivery, unless a navigation test is
  blocked by it.

## Open items

- Measure the test box edge gap: the fillet radius, the gap width, whether the wedge drops in or
  catches on a lip, and whether it's the same on every side and in the corners. A photo of the
  wedge in the gap would help set the contact geometry.
- Find the newest field recordings for the dropout distribution. The dev box has nothing after
  10/26, and the ZED Box was unreachable on 2026-11-14.
- Confirm which firmware build was flashed for 10/25, and the ESP-NOW staleness timeout before
  control falls back to the sticks.
- Measure the eRPM telemetry rate, delay and quantization on the robot before modelling them.
- Confirm Mrs Buff Mk3's front/back keypoint convention matches the direction her weapon faces in
  the tracks, before the weapon arc is calibrated.
- Measure Mk3's and Mk2's stall push force (5.1).
- Measure the motor can temperature after a 3 minute session, to set the thermal trip (0.3).
- Find why the firmware yaw loop delivered 30% and 0 at the wall on 10/25 (5.4).
- Confirm the fitted grey-box plant passed its fit gate before it goes into the our-robot filter
  (1.3). The plant slot was empty pending that gate when the Kalman seam landed.
- Confirm which classical navigation has a working Mk2 field profile for the calibration bouts
  (1.5).

## Work packages

Each package is sized for one agent and one branch. The package row plus the plan sections it
names are the whole brief; an agent picking one up reads those sections, `CLAUDE.md`, and nothing
else is assumed.

### Rules for every agent

- Work in your own git worktree branched from `main`. Agents cannot edit the shared checkout.
  Hand back a branch that merges with `git merge --ff-only`. A fresh worktree has an empty
  `data/`, no `venv/` and no `third_party/foxglove`: symlink them from the main checkout.
- Stay inside your package's "Owns" paths. If you need a change in another package's paths, say
  so in your hand-back instead of making it.
- Never modify anything under `data/`. Analysis output goes in `docs/experiments/control/nav_eval/` or
  the scratchpad, never `runs/`.
- Never send drive commands to a robot. Anything that needs the robot to move, a firmware flash,
  a measurement or a label is a user task (table below): prepare the tool and the instructions,
  then stop and hand back.
- Training goes through `training/gpu_queue.py` with `--work` and `--profile`. Only WP-21
  submits training jobs.
- Cap builds at `-j6`. Run `./scripts/lint` before handing back.
- A C++ filter or obs-spec change regenerates its Python mirror and golden fixture in the same
  branch.
- Report numbers with their source run or recording. If a check in "Done when" fails, hand back
  the failure; do not loosen the check.

### Packages

| ID | Package | Plan | Owns | Depends on | Done when |
| --- | --- | --- | --- | --- | --- |
| WP-01 | Lockstep reproduction of 10/25 | 0.1 | `auto_battlebot/rl/lockstep_world.py` (slew and guard-scale stages), a new `auto_battlebot/nav/repro_1025.py`, report | none | Four-arm table of reverse-window duty against the field's -12%, written up |
| WP-02 | Thermal trip and loosened guard | 0.3, 1.2 | `firmware/mr_stabs_mk2/`, guard fields in `firmware/shared/espnow_protocol/` | U2 for the threshold | Firmware builds; trip latches in a bench test with the threshold forced low; trip state in telemetry |
| WP-03 | Step-test analysis and armature refit | 0.2 | `auto_battlebot/nav/step_test.py`, armature in `auto_battlebot/mujoco_sim/` | U1 | Pitch response per step size, refit armature, a keep-or-remove verdict on the slew limit |
| WP-04 | RL profile slew setting | 1.1 | profiles loading `navigation/*_rl.toml` | WP-03 | Value set per the WP-03 verdict; sim matches it |
| WP-05 | IMU yaw-rate fusion in the our-robot filter | 1.3 | `include/robot_filter/`, `src/robot_filter/`, the filter's Python mirror and fixture, tests | none | Correlation at least 0.7 on the three 10/25 MCAPs replayed; p99 inside the IMU's range; tests pass |
| WP-06 | Fitted plant in the our-robot filter | 1.3 | same as WP-05, `include/plant/` | WP-05 merged, WP-14 | Paired lockstep A/B of `rl_nav_p4` with a ship or wait verdict |
| WP-07 | PerformanceMonitor | 1.4 | new component (check `include/health/` first and extend it if it fits), `include/remote/protocol.hpp` rows, bundle hash in `auto_battlebot/rl/export/` | none | Mismatch blocks policy load and lists keys; both 10/25 setup problems fire in replay; warnings land in `/diagnostics` |
| WP-08 | Metric library and Mk2 profile | 2.1, 2.2 | `auto_battlebot/eval/nav/`; `auto_battlebot/rl/metrics.py` and `auto_battlebot/rl/score_field_session.py` become callers | none | One function scores a lockstep episode and an MCAP bout; old callers give the same numbers through it; mypy clean |
| WP-09 | Event labeler | 2.3 | `auto_battlebot/nav/labeler/` | WP-08 event schema | 00-49 loads with synced video, tracks and timeline; proposals editable; per-segment pass gate persists across restart |
| WP-10 | Scorer validation | 2.4 | `auto_battlebot/nav/validate_scorer.py`, report | WP-08, WP-09, WP-14, U3, U4 | Precision and recall per event type on hand labels and on sim contact truth; thresholds frozen in the profile |
| WP-11 | Field session app and bout splitter | 2.5 | `auto_battlebot/nav/field_session/` | WP-08, WP-13 | The three 10/25 MCAPs split into bouts and land in the store; pair scheduler and "no verdict" rule work on synthetic bouts |
| WP-12 | Scenario manifest `nav_suite_v1` | 3.1, 3.4 | `auto_battlebot/eval/nav/suite/`, the manifest file | none | Manifest with tags, content hash, gated bucket list, held-out split marked |
| WP-13 | Results store, statistics, report generator | 3.2, 3.4, 3.5 | `auto_battlebot/eval/nav/store.py`, `stats.py`, `docs/experiments/control/nav_eval/results.md` generator | none | Append and query round-trip; pairing refuses mismatched `sim_hash`; bootstrap reuses `auto_battlebot/eval/significance.py` |
| WP-14 | Lockstep runner on the manifest, A/A, smoke tier | 3.3, 3.5 | `auto_battlebot/rl/lockstep_eval.py`, smoke script | WP-08, WP-12, WP-13 | `rl_nav_p4` scored on `nav_suite_v1`; A/A half-width recorded; smoke tier under 5 min |
| WP-15 | Fidelity checks and rank agreement | phase 4 | `auto_battlebot/nav/fidelity/`, report | WP-14, WP-11, U5 | Four checks reported for `rl_nav_p4` and both calibration controllers |
| WP-16 | Residual model: wheel response and perception | phase 4 | `auto_battlebot/rl/` residual module, sensing model TOML | WP-15 | Residual cuts the response-fidelity error on a held-out MCAP; `sim_hash` changes; baseline rerun |
| WP-17 | Observation spec v2 | 5.3 | `auto_battlebot/rl/obs_spec.py`, `include/rl_policy/`, `src/rl_policy/`, sim gyro and eRPM models | U6 for telemetry timing | Bundle self-test passes parity on v2; v1 bundles still load |
| WP-18 | `drive_mode`, both arms | 5.4 | `include/enums/drive_mode.hpp`, `firmware/shared/espnow_protocol/`, Mk2 firmware mixer path, ESC floor in the sim | WP-02 merged (same firmware), U7 | Both modes drive in lockstep; ESC floor reproduces the 280 rad/s step; yaw-loop wall fault explained |
| WP-19 | Physics opponent, pin reward, hit impulse | 5.1 | `auto_battlebot/rl/envs/` opponent bodies and controllers, reward terms | U8 for push force | Mk2 pushes and pins the opponent in a scripted lockstep episode; idle Mk2 is not flipped by the opponent |
| WP-20 | Edge gap, arena weighting, dropout, battery sag | 5.5, 5.6 | MJCF wall geometry, start samplers, `simulation/sensing_models/` | U9, U10 | Scripted "turn in place" sticks in the gap and "back off then turn" escapes, in lockstep; dropout matches the measured gap histogram |
| WP-21 | Training ladder R1-R4 | 5.0, 5.7 | GPU queue submissions, `data/models/nav_policies/` exports, rung reports | WP-14, then WP-17 to WP-20 per rung | Each rung's row in 5.0 passes, or a report says which check failed |
| WP-22 | Mk3 hunter policy | 5.2 | `auto_battlebot/rl/envs/policy_opponents.py`, hunter training config | WP-19, R3 done, only if R3 misses its arc-time target | Hunter beats the R3 Mk2 on hits landed with the CI above 0 |

WP-17 to WP-20 all edit `auto_battlebot/rl/envs/warp_nav.py` and `lockstep_world.py`. They can be
built at the same time, but they merge in ladder order (17, 18, 19, 20), each rebased on the one
before. WP-01 touches `lockstep_world.py` first and merges before any of them.

### Waves

| Wave | Start when | Packages |
| --- | --- | --- |
| A | now | WP-01, WP-02, WP-05, WP-07, WP-08, WP-12, WP-13 |
| B | their dependencies merge | WP-03, WP-04, WP-06, WP-09, WP-11, WP-14, and the builds of WP-17 to WP-20 |
| C | baseline session scored | WP-10, WP-15, WP-16, WP-21 rung R1 |
| D | each rung passes | WP-21 R2 to R3, the field session, then WP-22 and R4 if needed |

Wave A is seven agents with no shared files except `auto_battlebot/eval/nav/`, where WP-08 owns
the package root and WP-12 and WP-13 own their own subpaths.

### User tasks

These cannot be delegated. Each one unblocks the packages listed.

| ID | Task | Unblocks |
| --- | --- | --- |
| U1 | Drive the 0.5, 0.6, 0.7 forward step test on a soft surface, slew limit off | WP-03, WP-04 |
| U2 | Flash the WP-02 firmware; measure motor can temperature after a 3 minute driven session | WP-02 threshold, the baseline session |
| U3 | Label 00-49, then 00-28, in the labeler (about an hour each) | WP-10 |
| U4 | Record and label a short session of human-driven Mk2 pinning Mk3 | WP-10 pin detector |
| U5 | Baseline field session: `rl_nav_p4` and two calibration controllers, interleaved, rated | WP-15, k and the floors, every later comparison |
| U6 | Run the eRPM telemetry capture script on the robot (rate, delay, quantization) | WP-17 |
| U7 | Bench-reproduce the yaw loop at the wall with the diagnostics page open | WP-18 |
| U8 | Measure Mk3 and Mk2 stall push force | WP-19 |
| U9 | Measure the test box edge gap and photograph the wedge in it | WP-20 |
| U10 | Bring the ZED Box online so the newest recordings can be pulled for the dropout distribution | WP-20 |
| U11 | Record one Mk3 driving session for the held-out split | WP-12 holdout, R4 |
| U12 | Field sessions for the `drive_mode` bake-off and for R3 | acceptance |

U1, U2 and U5 gate the most work. U3 is the longest single sitting.

## Decision log

Answers from the 2026-11-14 grilling session, in order. Q1-Q12 were answered in chat; Q13-Q53 on
the grill form. The raw state is in `~/.cache/grill-form/state.json` on the dev box, which the
next grilling session overwrites; this table is the durable record.

| Q | Question | Answer |
| --- | --- | --- |
| Q1 | What does "impressive" mean? | Mk2: avoid the weapon 99% of the time and get lots of pins. Mk3/Mk4: avoid the weapon, score hits at opportune times; knockouts win outright, aggression counts otherwise (NHRL judging). |
| Q2 | What did the failure look like? | The RL policy drove confidently. It sometimes got stuck on the wall or froze while commanding full reverse. Mk3 (human-driven) hit it too often; some good back wedges, wanted more. |
| Q3 | Commit to one controller? | RL is the only path. The policy must handle unknown field conditions, a measured changing battery, and a degraded or missing wheel. |
| Q4 | Commit to one robot? | Demonstrate on Mk2 first, then apply to Mk3 and eventually Mk4. |
| Q5 | Is ESP-NOW in the control path? | Mk2 uses ESP-NOW with Crossfire as failsafe. Mk3/Mk4 use ELRS only. |
| Q6 | Is tracking good at driving speed? | Check the 10/25 recordings. Verified: 89-94% live, p90 gap 34 ms. |
| Q7 | Freeze the side quests? | Yes, except hardware: the new compute enclosure and power delivery. |
| Q8 | What earns Mk2 points? | Mk2 is a wedge and won't compete again; success is impressing the human driver in the test box. Mk3 is a vertical spinner. |
| Q9 | One "hit taken" metric for sim and field? | Yes. |
| Q10 | How to adapt to damage and battery? | Battery and per-wheel eRPM in the observation, plus a longer history. BLHeli reports eRPM; AM32 on Mk3/Mk4 can add it. |
| Q11 | Reproducible gate before field tests? | Yes. |
| Q12 | Make ESP-NOW the control link? | It already is: SA plus a fresh ESP-NOW command drives. Verified in firmware. |
| Q13 | Physics opponent body? | Yes, dynamic body, all controllers kept. |
| Q14 | League opponent? | Yes: train a Mk3 hunter, then Mk2 against it plus human replay plus scripted pursuers. |
| Q15 | Dropout in training? | Yes, from the measured distribution; keep the 0.15 s stop. Check the latest recordings, since dropout improved. |
| Q16 | eRPM, damage, battery sag in sim? | eRPM and battery. Leave damage out until Mk2 is proven in the test box. |
| Q17 | Which eval is the gate? | Lockstep with bootstrap CIs; Warp for training only. |
| Q18 | Field-test cadence? | Ad hoc. |
| Q19 | Slew limit? | Remove (0) and add a command-chain parity check. |
| Q20 | Motor guard? | Loosen for test-box sessions only. |
| Q21 | Firmware yaw loop in the RL path? | Direct left/right wheel commands in autonomy. |
| Q22 | Yaw-rate observation? | IMU yaw rate in obs spec v2; also fix the filter yaw rate. |
| Q23 | Arena distribution? | About 70% at 1.4-1.6 m, 30% full range, wall and corner starts. |
| Q24 | Pre-session checklist? | Asked for more explanation; became Q30. |
| Q25 | Requirement is behaviour, method open? | Keep RL as the commitment. |
| Q26 | Which RL variants? | PPO main line; add teacher-student if adaptation is weak; seed with scripted demos. |
| Q27 | Weapon-arc safety filter? | No, keep avoidance inside the controller. |
| Q28 | Explicit online plant estimator? | No, adaptation stays implicit in the RL history. |
| Q29 | How to fix the filter yaw rate? | Both now: fuse the IMU and switch to the fitted plant. |
| Q30 | Checklist, explained? | Capture it as warning diagnostics instead; competitions have no time for long checklists. Became Q43. |
| Q31 | Guard vs parity? | Loosen until it never binds; verify from logs. |
| Q32 | What triggers a field session? | Whenever convenient, no minimum. |
| Q33 | Three tiers, one metric library? | Yes. The user must be in the loop building the scorer's data, the way they reviewed every detector GT image. |
| Q34 | Hand-labeled field events? | Yes, in-repo labeler, starting with 10/25. |
| Q35 | Headline metric? | Net exchange headline plus guardrails plus diagnostics. |
| Q36 | Human-driven Mk2 as the bar? | No, only `rl_nav_p4` as the baseline. |
| Q37 | Versioned scenario suite? | Yes, with buckets and a held-out split, but not as named directories of blob files. Became Q45. |
| Q38 | Statistics? | Paired bootstrap, per-bucket and pooled acceptance. |
| Q39 | Fidelity checks? | All three. |
| Q40 | Field bout protocol? | Standardized blind interleaved bouts, with a separate web app to set up and score the runs. |
| Q41 | Regression tiers? | Smoke per commit, full on demand, results table. |
| Q42 | Final acceptance? | Beat `rl_nav_p4` with CI, floors fixed from the baseline session, median rating at least 4. |
| Q43 | Runtime warnings? | Yes, named `PerformanceMonitor`. A command-chain mismatch blocks the RL policy from loading; the rest warn. |
| Q44 | Labeler coverage? | Full coverage with a per-segment pass gate. |
| Q45 | Scenario and results management? | Tagged manifest plus a DuckDB/Parquet results store. |
| Q46 | Field scoring? | Auto-split bouts, tag them, append to the results store. |
| Q47 | Order of work? | Cheap field fixes first; the rest in the proposed order. |
| Q48 | Robot-agnostic eval? | Robot profile as data; only Mk2 implemented now. |
| Q49 | Residual sim correction? | Yes, after the fidelity tier. |
| Q50 | Model the edge gap? | Yes: the gap, wedge-caught starts, and an eval bucket. |
| Q51 | Gap dimensions? | Not measured yet. The field is bordered by 2x4s, so the board's rounded edge plus 2-5 mm. The wedge ends up in the gap and the robot turns in place instead of backing off, so it looks stuck. |
| Q52 | Keep direct wheel commands? | Yes, with an easy toggle back to the firmware yaw loop. |
| Q53 | How the toggle works? | Config enum, per-mode bundles, hash-checked. Remove the enum and toggle once the bake-off picks a winner. |

## Review amendments

Changes from the 2026-11-14 review of this plan. The grill answers above stand as recorded; this
table says where the plan now differs from them and why. Strike any row to restore the original.

| # | Touches | Change | Why |
| --- | --- | --- | --- |
| A1 | Q47 | New phase 0 before the cheap fixes: lockstep reproduction of 10/25, forward step test, thermal trip | The field session changes four things at once and attributes nothing; lockstep attributes for free |
| A2 | Q19 | Slew removal is blocked on the step test, and a limit may stay if the nose lifts | The RL report says a 0.8 step flips the fitted robot and the real one lifts more |
| A3 | Q20, Q31 | A latching thermal trip is added under the loosened guard | Placeholder limits loosened until they never bind protect nothing |
| A4 | Q29 | IMU fusion and the fitted plant land separately; the plant needs its own lockstep A/B | The plant changes position error too, which the training error table was built on |
| A5 | Q43 | The yaw-rate warning compares the IMU with the tracked heading, not with the filter | The filter fuses the IMU after 1.3, so the old check passes by construction |
| A6 | Q9, Q33 | Events are defined only from poses, our commands and our telemetry | The field has no contact sensor and cannot see Mk3's sticks |
| A7 | Q34, Q36 | Scorer validation adds sim contact truth and a human-driven pin session | 00-49 has close to no pins; human driving is detector data only, still not a bar |
| A8 | Q35 | Weapon-arc time is a second headline; k, tolerances and floors freeze after the baseline session | Q1 asked for 99% avoidance; tuning k after seeing candidates fits the metric to the answer |
| A9 | Q32, Q38, Q42 | Sessions stay ad hoc, pairs pool across sessions, no verdict under 10 pairs; pairwise preference added | A bootstrap on a handful of pairs reports intervals narrower than the truth |
| A10 | Q38 | At most six gated buckets, fixed in the suite | With every slice gated, some bucket fails by chance on every run |
| A11 | Q17 | A/A interval replaces the bit-identical determinism requirement | The interval is what the gate needs; bit-identity from a threaded binary can cost days |
| A12 | Q45 | `sim_hash` and `metric_version` columns in the results store | A residual refit or metric change otherwise makes old rows silently incomparable |
| A13 | Q39 | Rank agreement added as a fourth fidelity check, with two calibration controllers in the baseline session | Two controllers cannot show the sim ranks controllers the way the field does |
| A14 | Q49 | The residual model covers wheel response and perception error, not contact | 30 Hz poses with no force measurement cannot identify a contact residual |
| A15 | "retrain once" | The single retrain becomes a four-rung ladder, each rung gated | Nine changes in one retrain leave nine suspects when it fails |
| A16 | Q14 | The Mk3 hunter moves to the last rung and trains only if R3 misses its arc-time target | It is a second RL problem; the physics opponent and replay tracks may be enough |
| A17 | Q13 | Push force from a measurement, a hit impulse, and a contact reaction for replayed tracks | A weak opponent makes pins free; without an impulse the policy never practises recovery |
| A18 | Q21, Q52, Q53 | The bake-off models the ESC floor, randomizes latency for direct wheels, and repairs the yaw loop first | Racing the broken 10/25 loop decides the result before it runs |
| A19 | Q26 | "Scripted demonstrations" means the PPO anchor term on scripted teachers, decayed to 0 | PPO has no replay buffer to seed |
| A20 | Q37 | A dedicated Mk3 driving session supplies the held-out split | Three sessions from one evening is not a holdout |
