# Plant model PoC implementation plan

Implement `PlantModelInterface` (`include/robot_filter/plant_model_interface.hpp`) with the
stage A parameters as a proof of concept. Config-selected for tests, sim, and playback only.
The production factory slot stays empty until the fit passes acceptance, per the interface
comment.

## Inputs, already done

- `playground/calibration/out/plant_stageA.toml`: stage A per-phase values from all four
  sessions (2026-08-19 through 2026-08-23), written with `--no-joint`.
- Delay 0.0522 s from the onset stack (3486 edges, SNR 22.4, clock measured). Corroborated
  by the tau-pinned stage B profile minimum (54 ms) and the stage 2 camera measurement
  (59 ms, capture-time stamped). The 12 ms joint-fit minimum was the delay-tau tradeoff:
  freeing the taus let the optimizer double them (0.149 to 0.283, 0.174 to 0.45) and absorb
  the delay as lag.
- Model structure M4: drift terms zeroed, everything else live.
- Reference implementation: `auto_battlebot/plant.py`. The C++ model mirrors it exactly.

## Steps

### 1. Parameter loading

- `PlantParams` struct mirroring the `[plant]` table, loaded with toml++ the same way
  `load_robot_filter_from_toml` does in `src/robot_filter/config.cpp`.
- Refuse a file with no `[plant]` table. Ignore `[plant.provenance]`.

### 2. `JigPlantModel`

- `include/robot_filter/jig_plant_model.hpp`, `src/robot_filter/jig_plant_model.cpp`.
- `propagate()`, mirroring `plant.py` term for term:
  - Read the command at `t - delay_s` from the span (oldest first, zero-order hold).
    `command_ring_buffer.hpp` already exists; it must retain at least 52 ms plus the
    propagate span.
  - Per-sign deadzone removal, per-sign gain (`k_fwd`/`k_rev`, `k_ang`).
  - Steer-brake factor `max(0, 1 - c_sb * |u_ang_eff|)`, clamped exactly like
    `plant.py:251`. c_sb 2.70 crosses zero at `|u_ang_eff|` of about 0.37, so the clamp is
    load-bearing, not defensive.
  - Asymmetric first-order lag: accel vs decel tau chosen per channel by whether the
    target speed moves away from or toward zero.
  - Arc integration of the pose at 2 ms substeps.
- Jacobian: central differences over the 5 states. The model is piecewise (deadzone,
  clamp, sign-selected tau), so the analytic form is branch-following; numeric is the
  defensible PoC choice. Revisit if profiling says so.
- `reset()`: trivial, the model holds no state beyond its parameters.

### 3. Process noise

- Run `fit_process_noise.py` against `plant_stageA.toml` to produce `process_noise.toml`.
  It maps holdout residuals to a per-axis growth law and continuous-time Q, with
  NEES/coverage checks.
- Fallback if that stalls: diagonal Q seeded from the stage A holdout table
  (position RMSE 22 mm at 100 ms, 163 mm at 400 ms; heading 7.4 deg at 100 ms, 36 deg at
  400 ms).
- Expect the covariance to come out wide. Lag-1 residual autocorrelation is 0.91, so the
  residuals are structured, and tuning Q down to flatter the consistency stats would just
  hide that.

### 4. Tests

- `tests/test_jig_plant_model.cpp`.
- Golden test: a small Python script dumps `predict_windows` inputs and outputs for a few
  jig windows to a JSON fixture; the C++ `propagate()` must match within integration
  tolerance over a 500 ms window (same 2 ms substep, so the tolerance can be tight).
- Unit tests: deadzone edges, sign asymmetry, steer-brake clamp at zero, delay lookup at
  both ends of the command span, Jacobian against a finite difference of `propagate()`
  itself.
- CMake globs are not `CONFIGURE_DEPENDS`: run `cmake -S . -B build-test` after adding the
  new `.cpp`.

### 5. Wiring

- A config section selects the plant model and points at the params file. No production
  registration; sim and playback configs opt in.

## Validation

```bash
./scripts/build_and_test.sh --gtest_filter=JigPlantModel*
./scripts/lint
```

Then a playback replay with the our-robot filter reading the plant for prediction, checking
coast behavior across real perception dropouts (p90 gap 340 ms).

## Known limits

- Acceptance still fails at the 400 ms coast horizon: 163 mm and 36 deg on holdout against
  targets of 80 mm and 8 deg. At EKF predict horizons the model is good: 4.6 mm at 35 ms,
  22 mm at 100 ms.
- Heading is the weak axis. Pending structural work: the coupling shape (linear loss is
  wrong above `|u_ang_eff|` 0.37), per-side `k_ang` (spreads of +/- 11 on n=16), and the
  unresolved 2x disagreement with the stage 2 camera `k_ang` (31.7 vs 61.5 rad/s).
- `c_sb` 2.70 exceeds the joint-fit bound of 1.5 in `PARAM_BOUNDS`. Consumers clamp, so
  the value is usable as-is; fixing the bound or the loss shape is fit-side work, not
  filter-side.

## Next steps

1. Implement steps 1-2 and the golden test; that alone proves the interface fits.
2. Run `fit_process_noise.py` and wire the Q it produces.
3. Playback replay on an NHRL recording with the plant-backed predict step.
