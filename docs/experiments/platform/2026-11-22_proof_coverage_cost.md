# Proof coverage cost (2026-11-22)

Phase 4 of `docs/plans/verification_plan.md` asked for the per-gtest-case cost of
`scripts/verify --coverage` first: under 30 min for a full run keeps per-case `touches` sets,
otherwise per test binary.

## Setup

- Dev box `pathfinder` (32 threads, 31 GB), commit `9e6ef270` plus the phase 4 working tree
- `build-coverage/`: Debug, `BUILD_TESTING=ON`, `-DAUTO_BATTLEBOT_COVERAGE=ON` (gcc 13.3 `--coverage`)
- `venv/bin/python -m auto_battlebot.verify.coverage`, all four suites, one run

## Results

| Suite | Proofs | Wall time | Granularity |
| --- | --- | --- | --- |
| gtest | 1,063 cases | 329 s (0.31 s per case) | case |
| pytest | 902 tests | 50 s | case (slow tests left out) |
| firmware native | 211 cases | 26 s | suite |
| playback | 3 clips, 78 proof ids | 226 s | clip |
| total | 2,254 proof ids | 631 s | |

- A full gtest pass takes 5.5 min, inside the 30 min limit, so gtest `touches` stay per case
- The median proof touches 3 first-party files; the largest touches 139
- One gtest case run alone takes about 0.09 s; reading its 264 gcda files and diffing them against
  the no-test baseline takes the rest
- `tests/rl_policy/allocation_counter.cpp` appears in every gtest case: it replaces global
  `operator new`, so its counters move in every test

Counting every object with a non-zero arc counter marked 146 files as executed for a test that
only reads a constant table, through static initializers and COMDAT template copies from system
headers. Diffing against a run of no tests and attributing each moved function to its
own source file from the `.gcno` cut that case to 3 files: the header it reads, its test file, and
`allocation_counter.cpp`.

## Staleness check

`gtest:VizTopicsTest.RowsMatchTheirIds` passed through `scripts/verify --only` with `touches`
`include/publisher/viz_topics.hpp`, `tests/publisher/test_foxglove_publisher.cpp`,
`tests/rl_policy/allocation_counter.cpp`. Appending a newline to the test file turned it stale,
listing the changed files; restoring the file returned it to passing.
