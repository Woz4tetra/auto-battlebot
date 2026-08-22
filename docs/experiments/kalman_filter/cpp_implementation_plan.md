# Kalman filter C++ implementation plan

How the filter from [`kalman_filter_plan.md`](kalman_filter_plan.md) lands in this codebase.
Written while the velocity jig experiments are still in flight
([`velocity_jig_status_2026-08-21.md`](velocity_jig_status_2026-08-21.md)): the plant fit fails
C1-C3 and the model ladder is unresolved, so this plan builds everything that does not depend on
the model choice and leaves the plant model as a swappable slot to fill once a rung is selected.

## Status, 2026-08-21

Phases 1-3 are implemented; 290 tests pass and `scripts/lint` is clean. Along the way
`RobotFrontBackSimpleFilter` was renamed to `RobotFrontBackFilter` (files, config type string,
tests, `config/_common.toml`); the sections below use the old name where they describe the
pre-change code. What landed:

- Seam: `motion_estimator_interface.hpp`, `DeadReckoningMotionEstimator` (default, behavior
  unchanged), nested `[robot_filter.motion_estimator]` config.
- EKF core: `ekf_math.hpp` (Joseph update, gating, angle wrap, covariance floor),
  `command_ring_buffer.hpp`, synthetic NEES Monte Carlo test.
- `KalmanMotionEstimator`: opponent CV+CWNA filter with gating, reinit, max-coast hold,
  snapshot retrodiction, control-rate coasting via `coast()`; our robot dead-reckons
  identically to the default arm (verified by an equivalence test).
- Guarded slot: `plant_model_interface.hpp`; `our_robot_mode = "ekf"` is a config validation
  error until a ladder rung is selected.

Phase 4 stays blocked on the jig fit. Phase 5's C7/C8 need the joint jig and camera session.
Opponent `q` is a placeholder (30 m^2/s^3) until measured from the May replays per plan 3.2.

## What can proceed now and what is blocked

The EKF machinery is model-agnostic. Prediction/correction cycle, Joseph-form updates, innovation
gating, command ring buffer, fixed-lag retrodiction, reinitialization, and the synthetic NEES test
all work against any process model, including a test stub. The opponent filter (CV plus
white-noise acceleration, Part 3) is fully specified today and needs nothing from the jig; its
`q` comes from fight replays. Only the our-robot plant model is blocked on the fit.

| Piece | Blocked on jig? |
|---|---|
| Propagation seam extraction | no |
| EKF core + numerics + NEES test | no |
| Opponent CV+CWNA filter, `q` from replays | no |
| Measurement model (R, heading-flip gate) | no (R refinement needs the 2.4 joint session) |
| Our-robot plant model | yes, rung not selected |
| A/B against baseline (C7) | yes, needs joint session |

## Where the swap happens

**Decision: swap at the `RobotTemporalMotionFilter` seam inside `RobotFrontBackSimpleFilter`,
not at the `RobotFilterInterface` level.**

The plan doc's Part 4.1 table names `robot_ekf_filter.hpp` "registered in the filter factory",
which reads as a new top-level filter. I recommend against that. `RobotFrontBackSimpleFilter`
(`src/robot_filter/robot_front_back_simple_filter.cpp`, ~500 lines) is mostly measurement
front-end: keypoint-to-pose conversion, blob merging and suppression, field-bounds filtering,
global FrameId assignment, the our-robot anchor. The Kalman filter replaces none of that. It
replaces exactly one call, `temporal_motion_filter_.update_with_prediction(...)` at
`robot_front_back_simple_filter.cpp:154`.

Swapping at that seam:

- reuses the association front-end untouched, with its existing tests
- makes the C7 A/B a one-line config change where both arms share identical association, so any
  measured difference is attributable to state estimation alone
- matches the plan's own framing: "replacing `RobotFrontBackSimpleFilter`'s dead reckoning"

A new top-level filter would duplicate the front-end or force a larger refactor to share it.
Either costs more than the seam and buys nothing.

## Interfaces

### 1. `MotionEstimatorInterface`, the seam

`include/robot_filter/motion_estimator_interface.hpp`. Owns per-track state between the
association front-end and the emitted `RobotDescription`s.

```cpp
class MotionEstimatorInterface {
  public:
    virtual void reset() = 0;
    // Control-rate tick: record the command, advance internal state to `now`.
    virtual void predict(double now, const CommandFeedback &command_feedback) = 0;
    // Perception-rate: fold in associated measurements, return one description per live track.
    virtual std::vector<RobotDescription> update(std::vector<RobotDescription> measurements,
                                                 double timestamp,
                                                 FrameIdAssigner &frame_id_assigner,
                                                 const FieldDescription &field,
                                                 const MotionEstimatorContext &context) = 0;
};
```

`MotionEstimatorContext` carries what the current signature lacks and the EKF needs:
`tf_fieldcenter_from_camera` and `CameraInfo` (for range-dependent R rotated into the field
frame), plus the existing scalar knobs (`field_bounds_margin_meters`, `our_robot_hold_window_s`).

Two implementations:

- `DeadReckoningMotionEstimator`: wraps the existing `RobotTemporalMotionFilter` verbatim.
  Default. Behavior change: none.
- `KalmanMotionEstimator`: phase 2 below.

`RobotFrontBackSimpleFilter` holds a `std::unique_ptr<MotionEstimatorInterface>` selected by a
nested config table, following the `ConfigFactory`/`REGISTER_CONFIG` pattern already used for
the filter itself (`src/robot_filter/config.cpp`) and the drive processor factory:

```toml
[robot_filter]
type = "RobotFrontBackSimpleFilter"

[robot_filter.motion_estimator]
type = "DeadReckoningMotionEstimator"   # or "KalmanMotionEstimator"
```

Note `predict()` becomes real at this seam. Today `RobotFrontBackSimpleFilter::predict` only
stashes the command and all propagation happens inside `correct()`. The interface contract in
`robot_filter_interface.hpp` already permits predict-advances-state; the EKF uses it so the
estimate coasts at control rate through perception gaps instead of freezing between corrections.

### 2. `PlantModelInterface`, the swappable model slot

`include/robot_filter/plant_model_interface.hpp`. The our-robot process model.

```cpp
struct PlantState {  // mirrors plant.py: field-frame pose, body velocities
    double x, y, theta, v, w;
};

class PlantModelInterface {
  public:
    virtual void reset() = 0;
    // Propagate from t0 to t1 given the command history (already delay-shifted by the caller's
    // ring buffer), and write the state-transition Jacobian for the covariance update.
    virtual PlantState propagate(const PlantState &state, const CommandHistory &commands,
                                 double t0, double t1,
                                 Eigen::Matrix<double, 5, 5> &jacobian) = 0;
    // Continuous-time process noise, state- and input-dependent per plan 1.7 step 3.
    virtual Eigen::Matrix<double, 5, 5> process_noise(const PlantState &state,
                                                      const CommandHistory &commands,
                                                      double dt) = 0;
};
```

**No production implementation ships until a ladder rung is selected.** The config factory for
this slot exists but registers nothing; selecting `KalmanMotionEstimator` for the our-robot
track without a plant model is a config validation error with a message saying why. Tests use a
stub implementation (constant velocity, analytic Jacobian) that lives in `tests/`.

### On the six model complexities

The ladder is M0 through M6 in `auto_battlebot/plant.py` (`MODEL_LADDER`): M0 static map, then
six additions (delay, lag, asymmetry, gain asymmetry, coupling, two drift terms). Python
implements this as one parameterized code path where `ModelStructure` flags collapse disabled
terms to zero, "not five separate models that could disagree for uninteresting reasons"
(plant.py docstring).

The C++ side should mirror that: **one** `GreyBoxPlantModel` class behind
`PlantModelInterface`, configured by the fitted `plant_params.toml` plus a structure name. Rung
selection is data, not code. Do not write six C++ classes. The interface still earns its place:
it lets the EKF be built and NEES-tested now against a stub, and it keeps the door open for a
model that is not a rung of this ladder at all (the plan's own fallback if the grey box keeps
failing acceptance).

## `KalmanMotionEstimator` design

Per-track estimators behind one class implementing the seam:

- **Our robot** (`Group::OURS`): 5-state EKF `[x, y, theta, v, w]`, prediction by
  `PlantModelInterface` with 2 ms substeps, commands read from a ring buffer at `t - L_d`.
- **Opponents** (`Group::THEIRS`): 4-state linear KF `[px, py, vx, vy]`, CV plus white-noise
  acceleration, per-class `q` from config. Heading derived from the velocity vector, flagged
  low-confidence at low speed, as Part 3.1 specifies.

Shared machinery, all model-agnostic:

- **Measurement model.** Keypoint pose `h(x) = [px, py, theta]`; blob centroid `[px, py]` with
  larger R. R built in camera coordinates (range and cross-range variances) and rotated into
  field frame per measurement using the context transform. Initial variances are config
  placeholders; the 2.4 joint session replaces them with measured values.
- **Heading-flip gate.** Innovation near pi (or pi/2) accepts the position rows only and counts
  the event. The jig's k_ang finding (camera 61.5 vs gyro 32.16 rad/s, ratio 1.91) says flips
  are real and frequent, so this counter doubles as the perception-bug diagnostic the status doc
  asks for.
- **Innovation gating.** Chi-square at the measurement dimension. Rejected measurements are
  counted in diagnostics, never silently dropped.
- **Dropout policy.** Predict through gaps with growing covariance; past `max_coast_s` (default
  0.4, the measured p90-p95 of dropout gaps) hold position while covariance keeps growing.
- **Reinitialization.** N consecutive gated frames or trace(P) over threshold resets to the next
  accepted measurement with large P.
- **Retrodiction.** Fixed-lag buffer of the last ~20 (state, P, stamp) entries plus commands.
  A measurement stamped in the past rewinds to its stamp, corrects, re-propagates to now.
  `correct()` already receives capture-time stamps (`keypoints.header.stamp`), so the stamp path
  needs verification, not construction.
- **Numerics.** Joseph-form update, symmetry enforcement, angle wrapping in state and
  innovation, floor on diag(P). Fixed-size Eigen matrices throughout, no heap allocation in the
  loop.

Cost: per tick, tens of 5x5 matrix multiplies per track (16 substeps at 33 ms frame period),
single-digit microseconds on the Orin. Retrodiction re-propagation is bounded by the buffer
length. Nothing here threatens the 60 ms budget.

### Interaction with existing gating

The front-end already rejects jumps (`max_jump_distance`, `max_consecutive_jump_rejects` in
`RobotFrontBackSimpleFilterConfiguration`) and the keypoint tracker has its own candidate
selection. Adding innovation gating makes three layers that can fight: a measurement the
front-end rejects never reaches the filter, and one it accepts can still be gated. Resolution:
when `KalmanMotionEstimator` is active, the innovation gate owns outlier rejection and the
front-end jump rejection is configured off for that arm of the A/B. Decide from the C7 numbers
whether it stays off.

### Covariance output, a gap in the plan

The plan's stated point is an estimate that "carries an explicit uncertainty", but
`RobotDescription` has no covariance field and no downstream consumer is specified; C8 only
checks the nav sweep for non-regression. Adding fields to `RobotDescription` touches
serialization and every consumer, and nothing reads them yet. So: keep covariance internal to
the estimator for now, surface trace(P), NIS, gate counts, and coast state through the
diagnostics logger, and keep `is_stale` semantics for compatibility. Promoting covariance into
`RobotDescription` becomes its own change when target selection or navigation grows a use for
it.

## Phases

Each phase builds and passes tests on its own; phases 1 and 2 do not change behavior for any
existing config.

**Phase 1, seam extraction.**
`motion_estimator_interface.hpp`, `DeadReckoningMotionEstimator` wrapping
`RobotTemporalMotionFilter`, nested config plumbing, factory registration. Default config
selects dead reckoning. Existing `test_robot_front_back_simple_filter.cpp` passes unchanged.

**Phase 2, EKF core.**
`ekf.hpp` (predict/update/gate/reinit as a small header-only class over fixed-size Eigen types),
command ring buffer, fixed-lag buffer. GoogleTests: Joseph symmetry, angle wrap, gate behavior,
and the synthetic NEES consistency test from plan 2.6 (100 Monte Carlo runs with known Q and R,
mean NEES inside the 95% chi-square band) using the stub plant model.

**Phase 3, `KalmanMotionEstimator` with opponents live.**
Seam implementation, measurement model with placeholder R, opponent CV+CWNA filter, config for
`q` per opponent class, diagnostics counters. Our-robot track configured to dead-reckon via the
existing path until a plant model exists (the estimator takes a per-group mode). This inverts
the plan's 4.2 ordering (our robot first, opponents second) because the opponent filter is
unblocked and exercises every piece of shared machinery on real replays via NIS (C6).

**Phase 4, plant model. Blocked on rung selection.**
`GreyBoxPlantModel` mirroring `auto_battlebot/plant.py` exactly: per-sign deadzone and gain,
delay from the ring buffer, asymmetric exact-discretization first-order lag, coupling and drift
terms per the selected structure, arc integration with the `STRAIGHT_W` fallback, 2 ms substeps.
Golden-vector GoogleTest against Python-generated vectors at 1e-9, plus a finite-difference
check on the analytic Jacobian. Consumes the fitted `plant_params.toml` directly.

**Phase 5, validation.**
C7 A/B on the joint-session recording, replay regression per existing skills, C8 nav sim sweep.

## Risks

- **Seam signature churn.** The context struct is new; if the R model turns out to need more
  camera geometry than `CameraInfo` plus the transform, the seam changes again. Kept small on
  purpose; extend it when the 2.4 session data says what R actually needs.
- **Retrodiction and the FrameId assigner.** Rewinding filter state does not rewind assignment
  decisions. If an association was wrong, retrodiction faithfully re-propagates a wrong update.
  Mitigation is the same as the plan's: gating plus reinitialization, and the assigner stays
  upstream and untouched.
- **Two clocks in the loop.** `predict(now)` uses the control-loop clock; measurements carry
  capture stamps. Any offset between them lands directly in the retrodiction rewind. Verify the
  stamp path first, as plan 2.2 already instructs.

## Next steps

1. Phase 1: extract the seam, wrap the existing filter, wire the nested config. No behavior
   change, existing tests green.
2. Phase 2: EKF core plus the synthetic NEES test with a stub model.
3. Phase 3: opponent CV+CWNA filter live behind a config flag; measure `q` from the May replay
   set per plan 3.2.
4. Run `cmake -S . -B build` after adding the new `.cpp` files; the source glob does not
   re-run on its own.
5. Phase 4 waits for a fit that passes C1-C3 and a selected rung; the slot and its config error
   message are already in place.
