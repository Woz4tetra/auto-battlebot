# Domain mix arms

`training/yolo/make_domain_mix_arms.py` builds the training lists for the synthetic domain mix arms (`ARMS`).

## Arm fields

- `randomized`: randomized frames drawn from the pool. `None` takes all of them.
- `domain`: domain frames drawn from the pool after the filters.
- `venues`: defaults to `CAGE_VENUES`, not every venue loaded. Arms that want the basement too pass `ALL_VENUES`.
- `real_repeat`: writes the 452 real frames that many times over. Every other count is a draw from a pool, so this is the one place an arm carries a frame twice on purpose.
- `extra_real`: adds the `--extra-real` frames, written once whatever `real_repeat` says.

## Invariants

- An arm with no venue filter draws from the two renders the 2026-09-13 grid was built over (`CAGE_VENUES`).
- `main` checks that the unique frame count drops by exactly `real_repeat - 1` copies of the real frames. Oversampled real frames are the only intended repeats, so any other shortfall means a source was drawn twice.
- `run_domain_mix_arm.sh` stamps `DATE` before training, not after, so an arm that runs past midnight keeps one date.

## Arm rationale

- `d50000_cagehigh` is the pose-only arm for step 1b of `pose_only_perception_plan_2026-09-19.md`. It takes both cage renders and the 10,000 basement frames. With unequal venues an even interleave runs out of basement frames at 30,000, so this arm takes every frame and is not a prefix of any other.
- `d40000_real3x` is `d40000` with the real frames written three times. The real share holds at 2.28 percent instead of falling to 0.77. It tests whether diluting the 452 real frames costs `d40000` its small-robot recall, against the training schedule as the other candidate.
- The step 4 grid builds a candidate for each plausible mix, because the plan's "best mix" is not known until the Q1 to Q3 arms are scored. The candidate matching the winner gets trained. `swap_half` led the 2026-09-16 scoring, which is why its damage-free twin (`nodamage`) is in the grid.

## Render report gates

`training/synthetic/domain_render_report.py` checks a render before the budget goes on.

- `VIEW_COUNT_SLACK = 1`: a view's frame count may sit this far from an exact third, from the uneven split and rounding.
- `MAX_DROP_FRACTION = 0.25`: the step 3 gate. Past this share of attempted frames dropped, the mat margin or the distractor count needs a look.

Tests: `tests/python/test_make_domain_mix_arms.py`.
