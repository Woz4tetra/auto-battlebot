# Sensing model plan

Goal: one fitted model of what perception gets wrong, used in two places. The RL training
environment (`rl_policy_navigation_plan.md`) feeds it to the policy through the opponent filter
mirror. The lockstep sim server (`simulation/kinematic_sim_server.py`) feeds it to the real C++
binary, so gates G2 and G4 and any navigation comparison run against realistic perception
instead of ground truth or i.i.d. noise.

This plan owns the sensing model. It replaces the detail in `rl_policy_navigation_plan.md`
section 4 and `learned_sim_environment_plan.md` layer 2; both keep a pointer here.

Background:

- `docs/experiments/kalman_filter/opponent_filter_evaluation.md`: the first opponent measurement
  model and its limits.
- `docs/plans/opponent_motion_estimation_plan.md`: the 6-state opponent filter and its batched
  mirror, `auto_battlebot/perception/opponent_filter.py`.
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
- `s_e` is the per-episode error scale
- `o` is a per-episode constant field-frame offset: the field-lock and calibration error that
  shifts every measurement the same way. Its scale is fitted from the spread of per-session
  median residuals

**Heading noise and flips.**

```
z_θ = wrap(θ(t_k) + s_e · e + π · F_k),    e ~ N(0, σ_θ(v)²)
```

`F_k` is the flip indicator. Two forms, chosen by step 2's data check:

- independent: `F_k ~ Bernoulli(q(v))`, what the current fit assumes
- bursty: a second two-state semi-Markov process (normal, flipped) with its own run-length
  tables, used if flipped frames come in runs whose length distribution differs from geometric
  with the same rate (two-sample KS, p < 0.01)

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
| constant offset `o` | radius drawn from the fitted per-session spread, uniform direction | field lock |
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
`fit_opponent_sensing_model.py`, which it replaces), `check_sensing_model.py` (gates S1 to S4,
figures), `sample_sensing_model.py` (draws a few episodes and plots measured vs true for eyeball
checks).

**Fitted files: `simulation/sensing_models/<venue>.toml`**, one per venue (`apartment_floor`
first). Not under `config/`: the app never reads them. Header fields: spec version, fitting
recordings, git hash, which context cells borrowed, gate results.

```toml
spec_version = 1
venue = "apartment_floor"
fitted_from = ["auto_battlebot_..._2026-10-2x_...mcap", "..."]

[frames]
rate_hz = [30.0, 60.0]
drop_prob = 0.02

[latency]
range_s = [0.04, 0.08]
jitter_s = 0.0

[robots.opponent.position]
speed_bins_mps = [0.0, 0.25, 0.5, 1.0, 1.5]
sigma_along_m = [...]
sigma_cross_m = [...]
bias_along_m = [...]
bias_cross_m = [...]

[robots.opponent.heading]
sigma_rad = [...]
flip_model = "independent"            # or "bursty"
flip_rate_by_label = { OPPONENT = 0.005, MRS_BUFF_MK3 = 0.0 }

[robots.opponent.dropout]
robot_distance_bins_m = [0.0, 0.05, 0.2, 0.5]
wall_distance_bins_m = [0.0, 0.15, 0.3]
quantiles = [0.01, 0.05, 0.1, 0.25, 0.5, 0.75, 0.9, 0.95, 0.99]
measured_s = [[...]]                  # one row per (robot bin, wall bin) cell
dropped_s = [[...]]
borrowed = [...]

[robots.ours]                         # same tables as opponent, plus:
[robots.ours.ekf_error]
sigma = { along_m = [...], cross_m = [...], theta_rad = [...], v_mps = [...], omega_radps = [...] }
tau_s = { along = 0.0, cross = 0.0, theta = 0.0, v = 0.0, omega = 0.0 }

[randomization]
error_scale = [0.7, 1.5]
flip_scale = [0.5, 5.0]
dropout_stretch = [0.7, 2.0]
offset_radius_m = 0.0
```

Enums in the TOML (`flip_model`) are parsed into Python `Enum`s at load, so a typo fails at load
with the valid values listed.

### 6. Lockstep sim integration

The sim server's `Perception` class is replaced by `MeasurementModel` with `worlds = 1` on the
CPU. A config with no `[sensing_model]` file keeps the current behaviour of truth-only poses.

On the C++ side, a new `SimKeypointModel` (`include/keypoint_model/sim_keypoint_model.hpp`)
turns each delivered measurement into front and back keypoints at the configured keypoint offsets
along the measured heading, projected into pixels through the frame's `CameraInfo` and the
sim camera transform. The real `RobotFrontBackFilter` and `KalmanMotionEstimator` then run
unchanged. A new config, `config/simulation/kinematic_sim_sensing.toml`, selects
`SimKeypointModel` and `RobotFrontBackFilter`. The existing `kinematic_sim.toml` keeps
`GroundTruthRobotFilter` for control-only tests.

Protocol changes (`simulation/protocol.py`, `include/simulation/sim_connection.hpp`):

- each frame carries the measurement's capture stamp `t_k` separately from `sim_time`, so the
  real filter sees the delivery delay as the robot does
- each robot slot carries `(measured, x, y, θ)`; a dropped robot is `measured = 0` instead of NaN
- ground truth stays in the response for scoring, in its own block, never read by the pipeline

The round trip field -> pixels -> field is exact for a flat floor with the configured keypoint
height. A unit test checks it to 1e-6 m.

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
| apartment-floor sessions (`rl_policy_navigation_plan.md` section 9), live YOLO and EKF on the ZED Box, tags on both robots | AprilTag bundles, offline | everything: both robots' noise, flips, conditioned dropouts, latency, frame drops, our-robot EKF error |
| 2026-10-15 sysid sessions | AprilTag bundle (Mr Stabs only) | position and heading noise and flip runs for Mr Stabs; usable now |
| NHRL May and MassD KALMAN replays | none | dropout runs and gaps conditioned on distance (both robots' tracks known), flip rate by label |

Rules that apply to every fit:

- Live detections only for the measurement model. The sysid YOLO detections were run offline on
  the recorded video; the apartment sessions record the live detections, which include the
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
- Hold out one apartment session whole. It is never fitted on; the gates score against it.

### 9. Gates

| Gate | Passes when | Decides |
| --- | --- | --- |
| S1 marginals | on the held-out session, replaying its true poses through the model: measured-run and gap CDFs (pooled and per populated context cell), position error p50 and p90 per axis, and joint dropout rate of both robots each within 20% of the real session; flip rate within the real count's 95% Poisson interval | whether the measurement model is trusted (RL gate G1) |
| S2 filter output | the same sampled measurements run through the opponent filter mirror: rendered position error p50 and p90 at the 0.1 s and 0.35 s horizons, heading error p50, and stale fraction, each within 20% of the mirror run on the session's real measurements | whether what the policy sees is right, which matters more than S1 |
| S3 determinism | a seeded `MeasurementModel` reproduces its draws bit for bit on CPU and to 1e-6 on GPU; batch-1 and batch-N give the same stream for the same per-world seed | the sim server and the RL env can share results |
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
- Lockstep smoke test: one 20 s episode with `kinematic_sim_sensing.toml`, the real filter
  produces opponent tracks and the MCAP records `kalman_motion_estimator` diagnostics.

## Steps

Steps 1 to 4 need no new recordings.

1. **Sampler and file format.** `sensing_model.py` with tests, loading a v1 file converted from
   the existing `sensing_model_opponent.toml` (pooled dropout, field-frame noise). Gate S3. This
   unblocks the RL environment work against a provisional model.
2. **Refit from the sysid sessions.** Body-frame residuals and flip run lengths for Mr Stabs.
   Decide independent vs bursty flips. Confirms or kills the speed-dependent cross bias.
3. **Conditioned dropouts from the match replays.** Compute robot-to-robot and robot-to-wall
   distance for every measured and dropped span in the eight KALMAN replays, build the context
   tables, record borrowed cells.
4. **Lockstep integration.** `SimKeypointModel`, the protocol changes, the new sim config, the
   replacement of `Perception`. Lockstep smoke test.
5. **Fit on the apartment-floor sessions** once they exist: both robots, latency, frame drops,
   our-robot EKF error. Gates S1, S2 and S4 on the held-out session.
6. **Replay opponent smoothing** (section 7), for RL stage D and the lockstep `replay` opponent.
7. **Pointers.** Replace the sensing detail in `rl_policy_navigation_plan.md` section 4 and
   `learned_sim_environment_plan.md` layer 2 with a link here, and point the RL plan's
   `fit_sensing_model.py` path at `playground/sensing_model/`.

## Not modelled

- **False positives.** Static scenery (the floor banner) and spurious detections are gated by the
  filter and the depth height gate on the robot. They are not injected. If the replays show the
  opponent filter accepting false positives, add a Poisson false-measurement process near
  fixed field points.
- **Identity swaps.** Association is position-only, so two crossing opponents can swap. The RL
  plan uses one opponent, so this is out of scope until multi-opponent training.
- **Blob fallbacks.** Measurements without keypoints (no heading, blob sigma) are not emitted. Add
  them if the apartment sessions show a meaningful blob share.
- **Spinners above about 90 rad/s.** Heading aliases at 30 Hz. Covered by the heading-blind list,
  not by the noise model.

## Risks

- **Mr Stabs as the opponent.** Until the apartment sessions exist, every opponent noise and flip
  number comes from Mr Stabs. Mrs Buff Mk3 and real opponents will differ, and the 0.5x to 5x
  flip range is the only cover.
- **Sparse context cells.** Merged-robot and against-the-wall cells are the ones that matter and
  the ones with the least data. The borrowing rule keeps the sampler defined but can hide the
  effect the conditioning exists for. Report sample counts per cell in the fit output.
- **One venue.** The apartment floor and ZED Box camera geometry are not NHRL's. The first NHRL
  outing should record a tagged session and fit `nhrl_cage_1.toml`.
- **Filter changes.** Gate S2 runs through the current filter. Any change to the opponent filter
  re-runs S2, and any change to the model's spec version means an RL retrain.

## Next steps

1. Step 1: write `auto_battlebot/perception/sensing_model.py` and its tests against the
   converted provisional file.
2. Step 2: refit the sysid residuals in the body frame and check flip run lengths.
3. Record the apartment-floor sessions (`rl_policy_navigation_plan.md` step 0), which steps 5 and
   the gates need.
