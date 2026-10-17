# Sensing model plan

Goal: one fitted model of what perception gets wrong, used in two places. The RL training
environment (`rl_policy_navigation_plan.md`) feeds it to the policy through the opponent filter
mirror. The lockstep sim server (`simulation/kinematic_sim_server.py`) feeds it to the real C++
binary, so gates G2 and G4 and any navigation comparison run against realistic perception
instead of ground truth or i.i.d. noise.

This plan owns the sensing model. It replaces the detail in `rl_policy_navigation_plan.md`
section 4 and `learned_sim_environment_plan.md` layer 2; both keep a pointer here.

## Status, 2026-10-16

Steps 1 to 4 are done. Step 5 waits for the test-box sessions (`rl_policy_navigation_plan.md`
section 9); steps 6 and 7 need no recordings. The model in use is
`simulation/sensing_models/provisional.toml`: noise from the Mr Stabs Mk2 sysid training session
for both roles, dropouts and flip onsets from the eight NHRL May and MassD KALMAN replays,
latency nominal, frame drops unmeasured.

What the fit found (`playground/sensing_model/out/fit_report.json`):

- **Dropouts.** Opponent tracks are measured 58% of the time, ours 64%. Opponent gaps: p50
  0.20 s, p90 1.30 s, p99 5.61 s. Conditioning matters for the opponent: the gap p90 is about
  2.0 s with the robots touching or the opponent against a wall, against 0.44 s on open floor,
  4.5x. For our robot the cells differ less and without a clear pattern. 7 of 12 cells borrow
  for ours and 3 for the opponent, from per-cell counts of 6 to 133.
- **Long gaps.** `2026-05-02T14-12-27` has 89 s and 125 s gaps with a robot unseen against a
  wall. Spans are clipped at the 20 s episode length (7 for ours, 5 for the opponent) so they
  do not set the top quantiles alone.
- **Flips.** Bursty. Onsets run 0.059 per second for the opponent and 0.027 for ours; flipped
  runs are one frame at the median and two at p75, from 17 replay runs and 8 sysid runs.
- **Noise level differs between sessions.** Low-speed along-track sigma is 4.6 mm on the training
  session and 1.8 mm on the holdout, 2.5x, wider than the 0.7x to 1.5x error-scale range. Gate S1
  on the test-box sessions decides whether the range or the fit moves.

The lockstep smoke test passes: Mr Stabs Mk2 on the MuJoCo plant, pursuit against a static
opponent, `mr_stabs_mk2_kinematic_sim_sensing.toml`, 450 ticks. It closes from 1.27 m to 14 mm
with true poses and with the provisional model; the model delays the first command by two ticks.

Changes from the design below, each found while building it:

- the along-track bias splits by direction of travel (section 2)
- flips are bursty, with pooled flipped-run lengths (section 2)
- the field offset is `N(0, offset_sigma_m²)` and 0 until a per-session spread exists (section 2)
- S3 reproduces a stream on the same device and batch shape only; per-world streams that survive
  a batch change would need a counter-based RNG, and nothing needs that yet (section 9)
- the lockstep config is per robot, and `GroundTruthRobotFilter` reads the measurement block
  (section 6)
- `fit_opponent_sensing_model.py` and `sensing_model_opponent.toml` stay until step 7 repoints
  the docs that cite them

Background:

- `docs/experiments/kalman_filter/opponent_filter_evaluation.md`: the first opponent measurement
  model and its limits.
- `docs/experiments/kalman_filter/opponent_motion_estimation_report.md`: the 6-state opponent
  filter and its batched mirror, `auto_battlebot/perception/opponent_filter.py`.
- `docs/sysid_guide.md`: the AprilTag ground-truth rig.

## Requirements

1. Produce what the filters receive, not what they output. Opponent measurements go through the
   filter mirror in training and through the real filter in the lockstep sim, so the policy and
   the navigation see the estimator's lag, coast, sigmas and flip handling as they are.
2. Model the heavy tail of dropouts. Match gaps run p50 0.20 s and p99 5.46 s; a model that
   cannot produce multi-second blind spells is not useful.
3. Condition dropouts on state. Tracks are lost near walls and when the two robots merge, which
   is where incursions and missed strikes happen.
4. Every parameter is fitted from a named recording or marked as a guess, and every fitted part
   passes a gate against a held-out session before training uses it.
5. One implementation. The RL environment and the sim server call the same code, so they cannot
   drift apart.
6. Randomizable per episode, per world, at no runtime cost in the batched environment.
7. Off means truth. With the model disabled both consumers see true poses, which is what RL
   curriculum stages A and B use and what control-only sim tests want.

## What exists

| Item | State | Where |
| --- | --- | --- |
| Opponent measurement fit | position and heading noise, bias and flip rate by speed bin from the 2026-10-15 sysid sessions (Mr Stabs Mk2 standing in); dropout run and gap quantiles from sysid and from eight KALMAN match replays; nominal 63 ms latency | `playground/opponent_prediction/fit_opponent_sensing_model.py`, `sensing_model_opponent.toml` |
| Sysid truth loader | offline YOLO keypoints against a tag-only smoother bundle, truth trusted near accepted tag detections only | `playground/opponent_prediction/sysid_data.py` |
| Opponent filter mirror | batched torch, parity-tested against the C++ golden fixture; `update(stamp, measured, xy, yaw, is_keypoint, position_variance)` and `coast(now)` | `auto_battlebot/perception/opponent_filter.py` |
| Sim server perception | `Perception` class: fixed latency, i.i.d. Gaussian position and yaw noise, i.i.d. per-tick dropout, flat-plane projection bias. All zero in the shipped configs | `simulation/kinematic_sim_server.py:212` |
| Sim server C++ side | `NoopKeypointModel` + `GroundTruthRobotFilter`, which bypasses the real `RobotFrontBackFilter` and `KalmanMotionEstimator` | `config/simulation/kinematic_sim.toml`, `include/robot_filter/ground_truth_robot_filter.hpp` |
| Measurement variance on the robot | `measurement_noise_for`: keypoint or blob sigma plus `position_sigma_per_meter` times camera range | `src/robot_filter/kalman_motion_estimator.cpp:315` |
| Our-robot EKF coast | `max_coast_s = 0.2`, opponent `opponent_max_coast_s = 0.1` | `config/_common.toml:149` |

Gaps this plan fills:

1. The opponent fit uses Mr Stabs as the opponent and field-frame residuals. The -12.5 mm
   cross-axis bias at top speed is probably the sessions' driving direction, not the detector.
2. Dropouts are pooled quantiles with no conditioning, and two tables (sysid, match) disagree 2x
   at p90 with nothing choosing between them.
3. No our-robot error model. The our-robot EKF runs the plant model inside it and has no Python
   mirror, so RL needs its output approximated.
4. Nothing samples from the fitted TOML. No batched sampler exists.
5. The lockstep sim never runs the real filters, so G2 and G4 cannot test what the robot runs.
6. No gate compares sampled measurements, or the filter output they produce, with a held-out
   session.

## Design

### 1. Data flow

```
                       RL training (torch, GPU, N worlds)
true poses ──> measurement model ──┬─ opponent ──> filter mirror ──> rendered opponent ──> obs
                                   └─ our robot ─> dropout flags ─┐
true poses ──> our-robot EKF error model <────────────────────────┘ ──> rendered our robot ─> obs

                       lockstep sim (same sampler, batch 1, CPU)
true poses ──> measurement model ──> front/back keypoints in pixels ──> real C++ binary
                                     (SimKeypointModel)                  RobotFrontBackFilter
                                                                         KalmanMotionEstimator
```

The measurement model is the shared core. The our-robot error model exists only for RL, because
the lockstep sim runs the real our-robot EKF. That split also gives a cross-check: the lockstep
sim produces real-EKF output under sampled measurements, which the error model must match
(gate S4).

### 2. Measurement model

Applies to every tracked robot: our robot, opponents, and the house bot. Per world and per robot,
it turns a true pose stream into a delayed, noisy, gappy measurement stream.

**Frame clock.** A frame is captured at `t_k = t_0 + k / f`, `f` drawn per episode. A frame is
lost outright with probability `p_drop`, independent of the dropout process below (camera
frame drops, measured from `frame_meta` intervals).

**Dropout: two-state semi-Markov process per robot.** States are measured (M) and dropped (D).
On entering a state the process draws its duration by inverse transform from that state's
empirical distribution:

```
d = F_{s,c}⁻¹(u),    u ~ U(0, 1)
```

`F⁻¹` interpolates linearly through the fitted quantiles (p1, p5, p10, p25, p50, p75, p90, p95,
p99), in log duration so the tail interpolates sensibly, and is clipped to the observed maximum.
`s` is the state. `c` is the context bin read at state entry:

| Context | Bins (m) |
| --- | --- |
| distance between this robot and the nearest other robot, edge to edge | `[0, 0.05, 0.2, 0.5, inf)` |
| distance from this robot's centre to the nearest wall | `[0, 0.15, 0.3, inf)` |

That is 12 cells per state. A cell with fewer than 30 observed durations borrows from its
neighbour along the wall axis first, then the robot axis, then the pooled table. The fit records
which cells borrowed.

A plain Markov chain is not enough: a fixed per-frame switch probability forces exponential
durations, and an exponential with the match median of 0.20 s has a p99 of 1.33 s against the
measured 5.46 s.

Correlation between the two robots comes through the shared distance context: when they touch,
both enter the merged cell. Whether that is enough is checked in gate S1 (joint dropout rate).

**Position noise, body frame.** Residuals are rotated into the robot's own frame (along track,
cross track) before fitting, because motion blur and keypoint lag act along the direction of
travel:

```
z_p = p(t_k) + R(θ(t_k)) · (b(v) + s_e · n) + o
n ~ N(0, diag(σ_a(v)², σ_c(v)²))
```

- `v` is true speed, bins `[0, 0.25, 0.5, 1.0, 1.5, inf)` m/s
- `b(v)` is the median residual per bin, `σ` the MAD-based robust sigma (1.4826 x MAD)
- the along-track bias has a forward and a reverse value, picked by the sign of the forward
  speed. The 2026-10-15 training session puts it at +4.4 mm forward and +13.0 mm in reverse
  above 1.5 m/s; in the field frame the same effect read as a -12.8 mm cross-axis bias, which
  was the session's driving direction
- `s_e` is the per-episode error scale
- `o` is a per-episode constant field-frame offset, `N(0, offset_sigma_m²)` per axis: the
  field-lock and calibration error that shifts every measurement the same way. Both sysid
  sessions share one calibration day (median field residual 2.2 to 2.4 mm in x), so its spread is
  unmeasured and `offset_sigma_m` is 0 until the test-box sessions give one per session

**Heading noise and flips.**

```
z_θ = wrap(θ(t_k) + s_e · e + π · F_k),    e ~ N(0, σ_θ(v)²)
```

`F_k` is the flip indicator. Two forms, chosen by step 2's data check:

- independent: `F_k ~ Bernoulli(q(v))`, what the current fit assumes
- bursty: a second two-state semi-Markov process (normal, flipped) with its own run-length
  tables, used if flipped frames come in runs whose length distribution differs from geometric
  with the same rate (two-sample KS, p < 0.01)

The data chose bursty (step 2): on the training session 4 of the 8 flip runs last two frames,
where independent flips at the measured 0.1% rate would make a two-frame run about one time in
1,000. The flipped-run table pools the sysid runs with the runs bounded by heading-jump pairs in
the match replays; the normal-run table is exponential at each role's onset rate (half its
heading-jump rate).

Flip rates are per label. Unlisted labels use the generic `OPPONENT` rate. Labels on the filter's
heading-blind list emit no heading row, the same as on the robot.

**Latency.** A frame captured at `t_k` is delivered at `t_k + L`, still stamped `t_k`. `L` is
drawn per episode; a per-frame jitter `j ~ U(-J, J)` is added if step 3 shows one is needed.
Deliveries keep capture order. The delivery queue length is `ceil(L_max * f_max) + 1` frames
(6 at 80 ms and 60 Hz).

**What a measurement carries.** `(stamp t_k, measured flag, x, y, θ, is_keypoint)` per robot.
`is_keypoint` is always true in v1: blob fallbacks are not modelled. The position variance handed
to the mirror is computed exactly as `measurement_noise_for` does on the robot, from the same
config values and the camera range, so the filter weighs simulated measurements the way it
weighs real ones.

**Keypoint height bias.** Kept from the existing sim server: the flat-plane projection pulls
measurements away from the camera by `h / z_cam` of their horizontal offset. Fitted data already
contains this bias when the recordings share the camera geometry, so it is applied only when the
simulated camera differs from the fitting camera (lockstep NHRL configs), using the difference
between the two.

### 3. Our-robot EKF error model (RL only)

The RL observation reads our pose, forward speed and yaw rate from the EKF track. The error model
produces that rendered output directly:

```
x̂_k = x_k + e_k
e_k  = [along, cross, θ, v, ω] error, each component an AR(1) process:
e_k,i = ρ_i · e_{k-1,i} + s_e · σ_i(v) · √(1 − ρ_i²) · ε,    ε ~ N(0, 1)
ρ_i  = exp(−Δt / τ_i)
```

`√(1 − ρ²)` keeps each component's long-run std at `σ_i(v)` whatever the correlation time `τ_i`
is. Components are independent in v1; step 5 checks cross-correlations and adds a covariance if
any pair exceeds 0.3. Position components rotate with our true heading.

During a dropout of our robot (the measurement model's flags, so our dropouts share the same
fitted process), the rendered pose coasts at constant velocity from the last estimate:

```
x̂(t) = x̂(t_d) + v̂(t_d) · (t − t_d)    for t − t_d ≤ max_coast_s (0.2 s)
stale = 1                                 after that
```

On the robot the EKF coasts on the plant model with the commands, not at constant velocity. Gate
S4 compares coast error by gap age; if the real EKF coasts measurably better, replace the
constant-velocity coast with truth plus a fitted error growth `σ_coast(age)`.

### 4. Randomization

Each world draws a parameter set at reset. The ranges below are centred on the fit. Widen a range
only to cover measured spread between sessions or venues, and only after the gates pass, because
every widening makes the policy more cautious.

| Parameter | Range | Basis |
| --- | --- | --- |
| frame rate `f` | 30 to 60 Hz | camera modes in use |
| frame-drop probability | 0.5x to 2x the fit | May had 25%; measure the new sessions |
| latency `L` | 40 to 80 ms | RL plan; refit from step 3 |
| error scale `s_e` | 0.7x to 1.5x | RL plan |
| constant offset `o` | `N(0, offset_sigma_m²)` per axis, sigma from the per-session spread | field lock |
| flip rate | 0.5x to 5x the fit | fitted on Mr Stabs; unseen opponents may flip far more |
| dropout duration stretch | D durations x 0.7 to 2, M durations / the same factor | venue spread; sysid vs match differ 2x at p90 |
| our-robot `σ_i` | 0.5x to 1.5x | wide until S4 passes, then narrow |
| our-robot `τ_i` | 0.5x to 2x | same |

The episode's parameter set goes to the RL critic as privileged input and never to the actor.

### 5. Code

**`auto_battlebot/perception/sensing_model.py`**, next to the filter mirror it feeds:

```python
@dataclass
class SensingModelFile:
    """Parsed sensing model TOML: tables, bins, ranges, spec version."""
    @classmethod
    def load(cls, path: Path) -> SensingModelFile: ...

class MeasurementModel:
    """Batched measurement model: worlds x robots, torch, one generator per instance."""
    def __init__(self, spec: SensingModelFile, worlds: int, robots: int,
                 device: torch.device, generator: torch.Generator, enabled: bool = True): ...
    def reset(self, mask: torch.Tensor) -> None:
        """Draw per-world parameters, restart dropout and flip states, clear delivery queues."""
    def step(self, now: torch.Tensor, truth: TruePoses, context: Context) -> Deliveries:
        """Capture frames due by `now`, queue them, return the ones delivered by `now`."""
    @property
    def params(self) -> SensingParams:
        """Per-world draws, for the critic and for logging."""

class OurEkfErrorModel:
    def __init__(self, spec: SensingModelFile, worlds: int, device, generator): ...
    def reset(self, mask: torch.Tensor) -> None: ...
    def step(self, now: torch.Tensor, truth: TruePoses,
             measured: torch.Tensor) -> RenderedOurRobot: ...
```

`step` runs at the physics or tick rate and emits only when a frame boundary has passed, so it
works at the RL environment's 2 ms step and the sim server's tick alike. All randomness comes
from the instance's `torch.Generator`, so a seed reproduces a run.

The RL environment may later fold `step` into its captured Warp graph. If it does, the Warp port
must reproduce the torch version on a seeded fixture to float tolerance, the same rule as the
filter mirror.

**`playground/sensing_model/`**: `fit_sensing_model.py` (generalizes
`fit_opponent_sensing_model.py`, which step 7 deletes), `check_sensing_model.py` (gates S1 to S4,
figures), `sample_sensing_model.py` (draws a few episodes and plots measured vs true for eyeball
checks). The sysid session loader it shares with `playground/opponent_prediction/` moved to
`auto_battlebot/perception/sysid_sessions.py`.

**Fitted files: `simulation/sensing_models/<venue>.toml`**, one per venue (`test_box`
first; `provisional.toml` until then). Not under `config/`: the app never reads them. The fit
validates its output through the loader before writing it. Schema, as built:

```toml
spec_version = 1
venue = "provisional"
note = "..."                          # sources, and what is unmeasured
fitted_from = ["2026-10-15_00-29-00", "2026-05-02T11-45-08_KALMAN", "..."]
frame_rate_hz = 30.0                  # nominal; randomization draws its own
latency_s = 0.063
frame_drop_prob = 0.0

[robots.opponent]                     # one table per role; "ours" has the same shape
speed_bins_mps = [0.0, 0.25, 0.5, 1.0, 1.5]

[robots.opponent.position]
sigma_along_m = [...]
sigma_cross_m = [...]
bias_along_m = [...]                  # forward
bias_along_reverse_m = [...]
bias_cross_m = [...]

[robots.opponent.heading]
sigma_rad = [...]
flip_model = "bursty"                 # or "independent" with flip_rate = [...] by speed bin
flip_scale_by_label = {}              # optional per-label multiplier

[robots.opponent.heading.flip_runs]
quantiles = [0.0, 0.01, ..., 0.99, 1.0]
normal_s = [...]
flipped_s = [...]

[robots.opponent.dropout]
robot_distance_bins_m = [0.0, 0.05, 0.2, 0.5]
wall_distance_bins_m = [0.0, 0.15, 0.3]
quantiles = [0.0, 0.01, 0.05, 0.1, 0.25, 0.5, 0.75, 0.9, 0.95, 0.99, 1.0]
measured_s = [[...]]                  # one row per (robot bin, wall bin) cell, row-major
dropped_s = [[...]]
borrowed = ["robot1_wall1:wall", ...]

[robots.ours.ekf_error]               # step 5; OurEkfErrorModel refuses a file without it
sigma = { along = [...], cross = [...], theta = [...], v = [...], omega = [...] }
tau_s = { along = 0.1, cross = 0.1, theta = 0.05, v = 0.08, omega = 0.03 }

[randomization]
frame_rate_hz = [30.0, 60.0]
latency_s = [0.04, 0.08]
frame_drop_scale = [0.5, 2.0]
error_scale = [0.7, 1.5]
offset_sigma_m = 0.0
flip_scale = [0.5, 5.0]
dropout_stretch = [0.7, 2.0]
ekf_sigma_scale = [0.5, 1.5]
ekf_tau_scale = [0.5, 2.0]
```

The per-cell sample counts, the holdout session's noise and the flip analysis go to
`playground/sensing_model/out/fit_report.json`, not the model file.

Enums in the TOML (`flip_model`) are parsed into Python `Enum`s at load, so a typo fails at load
with the valid values listed.

### 6. Lockstep sim integration

The sim server's `Perception` class wraps `MeasurementModel` with `worlds = 1` on the CPU, set
by `[perception] sensing_model` and `randomize` in the sim TOML. An empty `sensing_model` keeps
truth-only poses, delivered every tick. The old i.i.d. `pos_noise_std`, `yaw_noise_std`,
`dropout_prob`, `our_dropout_prob` and `[latency] observation_ms` are gone.

The lockstep sim ticks once per camera frame (`[sim] dt` 33 ms), so the model's frame rate is
forced to the tick rate and the latency lands on whole ticks: the nominal 63 ms arrives as 66.7 ms.
A dropped frame still reaches the C++ side as a tick with no detections, because the runner needs
a frame every tick to advance the clock. `frame_drop_prob` is 0 in the provisional model, so this
does not arise yet.

On the C++ side, a new `SimKeypointModel` (`include/keypoint_model/sim_keypoint_model.hpp`)
turns each delivered measurement into front and back keypoints `keypoint_offset_m` either side of
the measured pose, projected into pixels through the frame's `CameraInfo` and the sim camera
transform, with no detection boxes. The sim's depth image is the bare floor, so the height gate
abstains on box-less detections and stamps each keypoint with the floor's height, zero; projecting
at z = 0 keeps the round trip exact. The real `RobotFrontBackFilter` and `KalmanMotionEstimator`
then run unchanged. `config/simulation/mr_stabs_mk2_kinematic_sim_sensing.toml` selects it. It
extends `simulation/_simulation` rather than `kinematic_sim`, because the merge drops a section
whole when its `type` changes and extending `kinematic_sim` would mean restating all of
`[robot_filter]`; it restates the sim and Mr Stabs parts instead. The existing `kinematic_sim.toml`
keeps `GroundTruthRobotFilter` for control-only tests, which now reads the delivered measurement
block, so a sensing model degrades it too.

Protocol changes (`simulation/protocol.py`, `include/simulation/sim_connection.hpp`):

- each frame carries the measurement's capture stamp `t_k` separately from `sim_time`, so the
  real filter sees the delivery delay as the robot does
- each robot slot carries `(measured, x, y, θ)`; a dropped robot is `measured = 0` instead of NaN
- ground truth stays in the response for scoring, in its own block, never read by the pipeline

The round trip field -> pixels -> field is exact for a flat floor. A unit test checks it to
1e-6 m through the real `FrontBackKeypointConverter`.

`FrameIdAssigner` rejects a measurement more than `max_jump_distance` (0.3 m) from its track's
last position, in every motion-estimator mode, and the opponent filter mirror does not model it.
At the modelled noise (5 to 12 mm) it never trips, so the mirror and the robot agree; it would
matter once false positives or identity swaps are injected (Not modelled).

### 7. Replay opponents

The RL `replay` behaviour and the sim server's `replay` opponent drive the opponent along human
tracks from NHRL, MassD and the new recordings. Most of those tracks came out of perception, so
they already contain noise and gaps, and the measurement model would add both a second time.

Before use, each track is RTS-smoothed (a forward pass and a backward pass of a
constant-velocity Kalman filter), interpolated only across gaps under 0.5 s, and cut into
separate segments at longer gaps. The smoothed track is then treated as truth. Tracks from
recordings with tag truth use the tag bundle directly.

### 8. Fitting

| Source | Truth | Gives |
| --- | --- | --- |
| test-box sessions (`rl_policy_navigation_plan.md` section 9): Mr Stabs Mk2 alone, live YOLO and EKF on the ZED Box, tag 76 | AprilTag bundle, offline | Mr Stabs noise and flips from live detections, our-robot dropouts against walls, latency, frame drops, our-robot EKF error, a second field lock |
| opponent sessions (same section), once a second robot exists | AprilTag bundles on both robots | opponent noise, flips and conditioned dropouts from a real opponent |
| 2026-10-15 sysid sessions | AprilTag bundle (Mr Stabs only) | position and heading noise and flip runs for Mr Stabs; usable now |
| NHRL May and MassD KALMAN replays | none | dropout runs and gaps conditioned on distance (both robots' tracks known), flip rate by label |

Rules that apply to every fit:

- Live detections only for the measurement model. The sysid YOLO detections were run offline on
  the recorded video; the test-box sessions record the live detections, which include the
  Jetson's frame drops and timing. Laptop replays of SVOs are never used for noise: their frames
  are affine-warped 2 to 3% against live frames.
- Join measurements to truth on frame interval, never nearest timestamp. SVO image stamps sit
  about half a frame before pipeline stamps.
- Truth is trusted only within the bundle's `TRUTH_MAX_GAP_NS` of an accepted tag detection.
- Residuals are rotated into the true robot frame before binning.
- Robust estimates: median for bias, 1.4826 x MAD for sigma. Flips (|heading error| > pi/2) are
  excluded from `σ_θ` and counted separately.
- Our-robot EKF error: resample the rendered EKF output and truth to 50 Hz on measured spans,
  take residuals, estimate `ρ` from the lag-1 autocorrelation per component, `τ = −Δt / ln ρ`.
  Fit coast error separately, by gap age.
- Latency: `frame_meta.image_stamp_ns` to the stamp at which the filter consumed the frame, both
  logged live.
- Hold out one test-box session whole. It is never fitted on; the gates score against it.

### 9. Gates

| Gate | Passes when | Decides |
| --- | --- | --- |
| S1 marginals | on the held-out session, replaying its true poses through the model: measured-run and gap CDFs (pooled and per populated context cell), position error p50 and p90 per axis, and joint dropout rate of both robots each within 20% of the real session; flip rate within the real count's 95% Poisson interval | whether the measurement model is trusted (RL gate G1) |
| S2 filter output | the same sampled measurements run through the opponent filter mirror: rendered position error p50 and p90 at the 0.1 s and 0.35 s horizons, heading error p50, and stale fraction, each within 20% of the mirror run on the session's real measurements | whether what the policy sees is right, which matters more than S1 |
| S3 determinism | a seeded `MeasurementModel` reproduces its stream exactly on the same device and batch shape | runs can be repeated |
| S4 our-robot EKF | sampled `OurEkfErrorModel` error against the real EKF error on the held-out session: per-component p50 and p90 within 20%, autocorrelation at 0.1 s within 0.1, coast error by gap age within 20% at 0.1 and 0.2 s. Cross-checked against the real EKF in the lockstep sim under sampled measurements | whether the RL our-robot observation is trusted |

A gate that fails names the component. Fix that component and re-run; never widen a randomization
range to cover a failed gate.

### 10. Tests

- `tests/python/test_sensing_model.py`: inverse-CDF sampler reproduces its quantiles over 1e5
  draws to 2%; the AR(1) process has the configured stationary std and lag-1 correlation;
  delivery queue keeps capture order and delays by exactly `L`; disabled mode returns truth;
  borrowed cells load; a bad `flip_model` fails at load with the valid values listed.
- `tests/keypoint_model/test_sim_keypoint_model.cpp`: field -> pixels -> field round trip to 1e-6 m;
  a `measured = 0` slot yields no keypoints.
- Lockstep smoke test: one 20 s episode with `mr_stabs_mk2_kinematic_sim_sensing.toml`, the real filter
  produces opponent tracks and the MCAP records `kalman_motion_estimator` diagnostics.

## Steps

Steps 1 to 4 need no new recordings.

1. **Sampler and file format.** Done 2026-10-16. `sensing_model.py` with tests, loading a v1 file converted from
   the existing `sensing_model_opponent.toml` (pooled dropout, field-frame noise). Gate S3. This
   unblocks the RL environment work against a provisional model.
2. **Refit from the sysid sessions.** Done 2026-10-16. Body-frame residuals and flip run lengths for Mr Stabs.
   Decide independent vs bursty flips. Confirms or kills the speed-dependent cross bias.
3. **Conditioned dropouts from the match replays.** Done 2026-10-16. Compute robot-to-robot and robot-to-wall
   distance for every measured and dropped span in the eight KALMAN replays, build the context
   tables, record borrowed cells.
4. **Lockstep integration.** Done 2026-10-16. `SimKeypointModel`, the protocol changes, the new sim config, the
   replacement of `Perception`. Lockstep smoke test.
5. **Fit on the test-box sessions** once they exist: Mr Stabs noise and flips from live
   detections, standing in for both roles; latency; frame drops; our-robot EKF error; the field
   offset between sessions. Gates S1, S2 and S4 on the held-out session. Opponent-specific noise
   waits for the opponent sessions.
6. **Replay opponent smoothing** (section 7), for RL stage D and the lockstep `replay` opponent.
7. **Pointers.** The RL plan half is done 2026-10-16: `rl_policy_navigation_plan.md` section 4
   links here and no longer names a fit script. Still to do: `learned_sim_environment_plan.md`
   layer 2.

## Not modelled

- **False positives.** Static scenery (the floor banner) and spurious detections are gated by the
  filter and the depth height gate on the robot. They are not injected. If the replays show the
  opponent filter accepting false positives, add a Poisson false-measurement process near
  fixed field points.
- **Identity swaps.** Association is position-only, so two crossing opponents can swap. The RL
  plan uses one opponent, so this is out of scope until multi-opponent training.
- **Blob fallbacks.** Measurements without keypoints (no heading, blob sigma) are not emitted. Add
  them if the test-box sessions show a meaningful blob share.
- **Spinners above about 90 rad/s.** Heading aliases at 30 Hz. Covered by the heading-blind list,
  not by the noise model.

## Risks

- **Mr Stabs as the opponent.** Until the opponent sessions exist, every opponent noise and flip
  number comes from Mr Stabs. Mrs Buff Mk3 and real opponents will differ, and the 0.5x to 5x
  flip range is the only cover.
- **Sparse context cells.** Merged-robot and against-the-wall cells are the ones that matter and
  the ones with the least data. The borrowing rule keeps the sampler defined but can hide the
  effect the conditioning exists for. Report sample counts per cell in the fit output.
- **One venue.** The test box and ZED Box camera geometry are not NHRL's. The first NHRL
  outing should record a tagged session and fit `nhrl_cage_1.toml`.
- **Filter changes.** Gate S2 runs through the current filter. Any change to the opponent filter
  re-runs S2, and any change to the model's spec version means an RL retrain.

## Next steps

1. Steps 6 and 7, which need no recordings.
2. Record the test-box sessions (`rl_policy_navigation_plan.md` step 0) once the box is free,
   for step 5 and the gates.
